/*
 *  H2 Encoder device driver (kernel module)
 *
 *
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
#include <linux/dma-mapping.h>
#include <linux/cma.h>
#include <linux/hashtable.h>
#include <linux/hash.h>
#include <linux/dma-mapping.h>
#include <linux/cma.h>
#include <linux/hashtable.h>
#include <linux/hash.h>
#include <linux/dma-mapping.h>
#include <linux/cma.h>
#include <linux/hashtable.h>
#include <linux/hash.h>

/* Our header */
#include "hx280enc.h"

#define MEM_ALIGN_SIZE (PAGE_SIZE * 4)
#define MEM_HASHTABLE_BITS 8

typedef struct hlinc {
        struct hlist_node node;
        dma_addr_t bus_address;
        void* virt_address;
        u32 size;
        int owner_pid; /* Client that allocated this chunk */
} hlina_chunk;

static DEFINE_MUTEX(mem_mutex);
static struct device *memalloc_dev = NULL;
/* memory size in MBs for MEMALLOC_DYNAMIC */
static unsigned int max_alloc_size = 0;
static unsigned int allocated_size = 0;

static struct hlist_head hlina_chunks[1 << MEM_HASHTABLE_BITS];

static int AllocMemory(unsigned *busaddr, unsigned int size);
static int FreeMemory(unsigned long busaddr);
static void ResetMems(void);

//chunks are allocated and freed by those functions
static void cmem_free(hlina_chunk *chunk);
static int cmem_alloc(u32 size, hlina_chunk **chunk_out);


static long memalloc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
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

        switch (cmd) {
        case MEMALLOC_IOCHARDRESET:
                mutex_lock(&mem_mutex);
                ResetMems();
                mutex_unlock(&mem_mutex);
                break;
        case MEMALLOC_IOCXGETBUFFER:
                ret = copy_from_user(&memparams, (MemallocParams*)arg,
                                     sizeof(MemallocParams));
                if(ret) {
                        printk(KERN_ERR "MEMALLOC_IOCXGETBUFFER: failed to copy params from user\n");
                        printk(KERN_ERR "MEMALLOC_IOCXGETBUFFER: failed to copy params from user\n");
                        printk(KERN_ERR "MEMALLOC_IOCXGETBUFFER: failed to copy params from user\n");
                        break;
                }

                mutex_lock(&mem_mutex);
                ret = AllocMemory(&memparams.busAddress, memparams.size);
                mutex_unlock(&mem_mutex);
                memparams.translationOffset = 0;
                ret |= copy_to_user((MemallocParams*)arg, &memparams,
                                    sizeof(MemallocParams));

                break;
        case MEMALLOC_IOCSFREEBUFFER:
                if (get_user(busaddr, (unsigned long *) arg)) {
                        printk(KERN_ERR "MEMALLOC_IOCSFREEBUFFER: failed to get busaddr from user\n");
                        return -EFAULT;
                }
                mutex_lock(&mem_mutex);
                ret = FreeMemory(busaddr);
                mutex_unlock(&mem_mutex);
                break;
        }

        return ret ? -EFAULT: 0;
}


int memalloc_release(struct inode *inode, struct file *filp)
{
    ResetMems();
    return 0;
}

void memalloc_cleanup(void)
{
    memalloc_dev = NULL;
    ResetMems();
}

int memalloc_init(struct device *mem_dev, unsigned int max_alloc_sz)
{
    int ret;
    unsigned int bus_addr;
    unsigned int size = 16*1024*1024; // 16MB
    u64 mask = dma_get_required_mask(mem_dev);
    if(dma_set_mask_and_coherent(mem_dev,mask)) {
        printk(KERN_ERR "memalloc: dma_set_mask(%lld) failed!!\n", mask);
        return -EINVAL;
    }

    max_alloc_size = max_alloc_sz;
    memalloc_dev = mem_dev;
	printk(KERN_INFO "memalloc: CMEM Memory Allocator\n");
    printk(KERN_INFO "memalloc: CMEM memory max size = 0x%08x\n", max_alloc_size);

    hash_init(hlina_chunks);

    ResetMems();

    ret = AllocMemory(&bus_addr, size);
    if(ret != 0) {
        printk(KERN_ERR "memalloc: Test Initialization FAILED\n");
        return ret;
    }

    FreeMemory(bus_addr);
    return 0;
}

// static int memalloc_release(struct inode *inode, struct file *filp)
// {
//     ResetMems();
//     return 0;
// }

static int AllocMemory(unsigned *busaddr, unsigned int size)
{
    int ret;
    hlina_chunk *chunk;
    ret = cmem_alloc(size, &chunk);
    if(ret != 0) {
        printk(KERN_ERR "%s: Allocation FAILED: size = %d\n", __func__, size);
        return ret;
    }
    *busaddr = chunk->bus_address;
    chunk->owner_pid = current->tgid;
    hash_add(hlina_chunks, &chunk->node, hash_32(chunk->bus_address, MEM_HASHTABLE_BITS));

    if(*busaddr == 0) {
            kfree(chunk);
            printk(KERN_ERR "%s: Allocation FAILED: size = %d\n", __func__, size);
            ret = -EFAULT;
    } else {
            allocated_size += chunk->size;
            printk(KERN_DEBUG "%s - after allocating %d bytes (effective %d) , total allocation is %d\n", __func__,
                   size, chunk->size, allocated_size);
    }

    return ret;
}

/* Free a buffer based on bus address */
static int FreeMemory(unsigned long bus_address)
{
    int cur_pid = current->tgid;

    hlina_chunk *tmp;
    /* Search for the chunk with the given bus address */
    hash_for_each_possible(hlina_chunks, tmp, node, hash_32(bus_address, MEM_HASHTABLE_BITS)) {
        if(tmp->bus_address == bus_address && tmp->owner_pid == cur_pid) {
            printk(KERN_DEBUG "%s - freeing chunk addr %llx of size %d for proc %d\n",
                   __func__, (unsigned long long)tmp->virt_address, tmp->size, tmp->owner_pid);
            hash_del(&tmp->node);
            allocated_size -= tmp->size;
            cmem_free(tmp);
            return 0;
        }
    }

    printk(KERN_ERR "%s: No address %lu or pid %d found while freeing memory!\n",
            __func__, bus_address, cur_pid);

    return -EINVAL;
}

/* Force release of all allocated cmem buffers for each pid.
 * pid == 0 means all the buffers
 */
/* Force release of all allocated cmem buffers for each pid.
 * pid == 0 means all the buffers
 */
static void ResetProcMems(const int cur_pid)
{
    unsigned int bkt;
    hlina_chunk *tmp;
    int total_leaked_chunks = 0;
    hash_for_each(hlina_chunks, bkt, tmp, node) {
        if(cur_pid == 0 || tmp->owner_pid == cur_pid) {
            printk(KERN_DEBUG "%s - Freeing chunk of size %d for proc %d\n",
                   __func__, tmp->size, tmp->owner_pid);
            total_leaked_chunks += tmp->size;
            hash_del(&tmp->node);
            cmem_free(tmp);
        }
    }

    if(total_leaked_chunks > 0) {
        printk(KERN_INFO "%s - Forced free of %d bytes for proc %d\n",
               __func__, total_leaked_chunks, cur_pid);
    }

    allocated_size -= total_leaked_chunks;
    if(allocated_size < 0) {
        printk(KERN_ERR "%s - allocated_size is negative: %d\n", __func__, allocated_size);
        allocated_size = 0;
    }
}

static void ResetMems(void)
{
    ResetProcMems(0);
}

static void cmem_free(hlina_chunk *chunk) {
    if (chunk && chunk->virt_address) {
        dma_free_coherent(memalloc_dev, chunk->size, chunk->virt_address, chunk->bus_address);
        allocated_size -= chunk->size;
        printk(KERN_DEBUG "%s: released %d bytes, total allocated size = %d max_allocate = %d\n", __func__,
                chunk->size, allocated_size, max_alloc_size);
        printk(KERN_DEBUG "%s: virt=0x%llx bus=0x%llx size=0x%x\n", __func__,
                (unsigned long long)chunk->virt_address, (unsigned long long)chunk->bus_address, chunk->size);
    } else {
        printk(KERN_ERR "%s - chunk is NULL or invalid\n", __func__);
    }

    kfree(chunk);

    if (allocated_size < 0) {
        printk(KERN_ERR "%s - allocated_size is negative: %d\n", __func__, allocated_size);
        allocated_size = 0;
    }
}

static int cmem_alloc(u32 size, hlina_chunk **chunk_out)
{
    hlina_chunk *chunk;

    size = ((size + MEM_ALIGN_SIZE - 1) / MEM_ALIGN_SIZE) * MEM_ALIGN_SIZE; // Align to MEM_ALIGN_SIZE

    if(allocated_size + size > max_alloc_size) {
            printk(KERN_ERR "memalloc: Allocation FAILED: total allocated size = %d exceeds max size = %d\n",
                    allocated_size + size, max_alloc_size);
            return -ENOMEM;
    }

    chunk = (hlina_chunk*)kzalloc(sizeof(*chunk), GFP_KERNEL);
    if (!chunk) {
            printk(KERN_ERR "cmem_alloc: Allocation FAILED: could not allocate chunk structure\n");
            *chunk_out = NULL;
            return -ENOMEM;
    }

    chunk->virt_address = dma_alloc_coherent(memalloc_dev, size, &chunk->bus_address,
                                             GFP_KERNEL);
    if (!chunk->virt_address) {
            printk(KERN_ERR "cmem_alloc: Allocation FAILED: could not allocate contiguous %d bytes\n", size);
            kfree(chunk);
            *chunk_out = NULL;
            return -ENOMEM;
    }

    allocated_size += size;
    printk(KERN_DEBUG "%s: Allocated size %d, total allocated size = %d max_allocate = %d\n", __func__,
           size, allocated_size, max_alloc_size);

    printk(KERN_DEBUG "%s: virt=0x%llx bus=0x%lx size=0x%x\n", __func__,
           (unsigned long long)chunk->virt_address, (unsigned long)chunk->bus_address, size);
    chunk->size = size;
    *chunk_out = chunk;

    return 0;
 }
