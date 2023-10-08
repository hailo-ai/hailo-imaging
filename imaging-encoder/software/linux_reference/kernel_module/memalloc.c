/*
 *  H2 Encoder device driver (kernel module)
 *  
 *  COPYRIGHT(C) 2014 VERISILICON
 *
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License 
 * as published by the Free Software Foundation; either version 2 
 * of the License, or (at your option) any later version. 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, 
 * USA. 
 */

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

/* Our header */
#include "hx280enc.h"

#ifndef HLINA_START_ADDRESS
#define HLINA_START_ADDRESS             0x02000000
#endif

#ifndef HLINA_SIZE
#define HLINA_SIZE                      96
#endif

#ifndef HLINA_TRANSL_OFFSET
#define HLINA_TRANSL_OFFSET             0x0
#endif

/* the size of chunk in MEMALLOC_DYNAMIC */
#define CHUNK_SIZE                      (PAGE_SIZE * 4)

/* memory size in MBs for MEMALLOC_DYNAMIC */
unsigned int alloc_size = HLINA_SIZE;
unsigned int alloc_base = HLINA_START_ADDRESS;

/* user space SW will substract HLINA_TRANSL_OFFSET from the bus address
 * and decoder HW will use the result as the address translated base
 * address. The SW needs the original host memory bus address for memory
 * mapping to virtual address. */
unsigned int addr_transl = HLINA_TRANSL_OFFSET;


static DEFINE_SPINLOCK(mem_lock);

typedef struct hlinc {
        u32 bus_address;
        u16 chunks_reserved;
        const struct file *filp; /* Client that allocated this chunk */
} hlina_chunk;

static hlina_chunk *hlina_chunks = NULL;
static size_t chunks = 0;

static int AllocMemory(unsigned *busaddr, unsigned int size,
                       const struct file *filp);
static int FreeMemory(unsigned long busaddr, const struct file *filp);
static void ResetMems(void);

long memalloc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
        int ret = 0;
        MemallocParams memparams;
        unsigned long busaddr;

        //pr_info("ioctl cmd 0x%08x\n", cmd);

        /*
         * extract the type and number bitfields, and don't decode
         * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
         */
        if(_IOC_TYPE(cmd) != MEMALLOC_IOC_MAGIC)
                return -ENOTTY;
        if(_IOC_NR(cmd) > MEMALLOC_IOC_MAXNR)
                return -ENOTTY;

        if(_IOC_DIR(cmd) & _IOC_READ)
                ret = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
        else if(_IOC_DIR(cmd) & _IOC_WRITE)
                ret = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
        if(ret)
                return -EFAULT;

        spin_lock(&mem_lock);

        switch (cmd) {
	case MEMALLOC_IOCGMEMBASE:
		__put_user(alloc_base, (unsigned long *) arg);
		break;
        case MEMALLOC_IOCHARDRESET:
                //PDEBUG("HARDRESET\n");
                ResetMems();
                break;
        case MEMALLOC_IOCXGETBUFFER:
                ret = copy_from_user(&memparams, (MemallocParams*)arg,
                                     sizeof(MemallocParams));
                //pr_info("MEMALLOC_IOCXGETBUFFER: size=%d\n", memparams.size );
                if(ret) break;

                ret = AllocMemory(&memparams.busAddress, memparams.size, filp);

                //pr_info("MEMALLOC_IOCXGETBUFFER: size=%d , busAddress=0x%x, ret=%d\n", memparams.size, memparams.busAddress, ret );
                memparams.translationOffset = addr_transl;

                ret |= copy_to_user((MemallocParams*)arg, &memparams,
                                    sizeof(MemallocParams));

                break;
        case MEMALLOC_IOCSFREEBUFFER:
                //PDEBUG("FREEBUFFER\n");

                __get_user(busaddr, (unsigned long *) arg);
                ret = FreeMemory(busaddr, filp);
                break;
        }

        spin_unlock(&mem_lock);

        return ret ? -EFAULT: 0;
}


int memalloc_release(struct inode *inode, struct file *filp)
{
        int i = 0;

        for(i = 0; i < chunks; i++) {
                spin_lock(&mem_lock);
                if(hlina_chunks[i].filp == filp) {
                        printk(KERN_WARNING "memalloc: Found unfreed memory at release time!\n");

                        hlina_chunks[i].filp = NULL;
                        hlina_chunks[i].chunks_reserved = 0;
                }
                spin_unlock(&mem_lock);
        }
        PDEBUG("memalloc_release: dev closed\n");
        return 0;
}

void memalloc_cleanup(void)
{
        if(hlina_chunks != NULL)
                vfree(hlina_chunks);

        PDEBUG("module removed\n");
        return;
}



int memalloc_init(unsigned int  _alloc_base, unsigned int _alloc_size)
{
       int result;
		        
        alloc_base = _alloc_base;
        alloc_size = _alloc_size;
		
	printk("memalloc: Linear Memory Allocator\n");
        printk("memalloc: Linear memory base = 0x%08x\n", alloc_base);

        chunks = (alloc_size * 1024 * 1024) / CHUNK_SIZE;

        printk(KERN_INFO "memalloc: Total size %u MB; %lu chunks"
               " of size %lu\n", alloc_size, chunks, CHUNK_SIZE);

        hlina_chunks = (hlina_chunk *) vmalloc(chunks * sizeof(hlina_chunk));
        if (hlina_chunks == NULL) {
                printk(KERN_ERR "memalloc: cannot allocate hlina_chunks\n");
                result = -ENOMEM;
                goto err;
        }

        ResetMems();

        return 0;

err:
        if(hlina_chunks != NULL)
                vfree(hlina_chunks);

        return result;
}

/* Cycle through the buffers we have, give the first free one */
static int AllocMemory(unsigned *busaddr, unsigned int size,
                       const struct file *filp)
{

        int i = 0;
        int j = 0;
        unsigned int skip_chunks = 0;

        /* calculate how many chunks we need; round up to chunk boundary */
        unsigned int alloc_chunks = (size + CHUNK_SIZE - 1) / CHUNK_SIZE;

        *busaddr = 0;
        //pr_info("AllocMemory: alloc_chunks=%d, tottal_chunk=%d\n", alloc_chunks, chunks);

        /* run through the chunk table */
        for(i = 0; i < chunks;) {
                skip_chunks = 0;
                /* if this chunk is available */
                if(!hlina_chunks[i].chunks_reserved) {
                        /* check that there is enough memory left */
                        if (i + alloc_chunks > chunks)
                                break;

                        /* check that there is enough consecutive chunks available */
                        for (j = i; j < i + alloc_chunks; j++) {
                                if (hlina_chunks[j].chunks_reserved) {
                                        skip_chunks = 1;
                                        /* skip the used chunks */
                                        i = j + hlina_chunks[j].chunks_reserved;
                                        break;
                                }
                        }

                        /* if enough free memory found */
                        if (!skip_chunks) {
                                *busaddr = hlina_chunks[i].bus_address;
                                hlina_chunks[i].filp = filp;
                                hlina_chunks[i].chunks_reserved = alloc_chunks;
                                break;
                        }
                } else {
                        /* skip the used chunks */
                        i += hlina_chunks[i].chunks_reserved;
                }
        }

        if(*busaddr == 0) {
                printk("memalloc: Allocation FAILED: size = %d\n", size);
                return -EFAULT;
        } else {
                //PDEBUG("MEMALLOC OK: size: %d, reserved: %ld\n", size,
                //       alloc_chunks * CHUNK_SIZE);
        }

        return 0;
}

/* Free a buffer based on bus address */
static int FreeMemory(unsigned long busaddr, const struct file *filp)
{
        int i = 0;


        for(i = 0; i < chunks; i++) {
                /* user space SW has stored the translated bus address, add addr_transl to
                 * translate back to our address space */
                if(hlina_chunks[i].bus_address == busaddr + addr_transl) {
                        if(hlina_chunks[i].filp == filp) {
                                hlina_chunks[i].filp = NULL;
                                hlina_chunks[i].chunks_reserved = 0;
                        } else {
                                printk(KERN_WARNING "memalloc: Owner mismatch while freeing memory!\n");
                        }
                        break;
                }
        }
        return 0;
}

/* Reset "used" status for all of proc buffers */
static void ResetProcMems(const struct file *filp)
{
        int i = 0;

        for(i = 0; i < chunks; i++) {
            if(hlina_chunks[i].filp == filp) {
                hlina_chunks[i].filp = NULL;
                hlina_chunks[i].chunks_reserved = 0;
            }
        }
}

/* Reset "used" status */
static void ResetMems(void)
{
        int i = 0;
        unsigned int ba = alloc_base;

        for(i = 0; i < chunks; i++) {
                hlina_chunks[i].bus_address = ba;
                hlina_chunks[i].filp = NULL;
                hlina_chunks[i].chunks_reserved = 0;

                ba += CHUNK_SIZE;
        }
}

/* module description 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Verisilicon");
MODULE_DESCRIPTION("H2 Encoder driver");
*/