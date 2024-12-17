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
 *
 * Copyright (c) 2015 - 2017 Cadence Design Systems, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
/* needed for __init,__exit directives */
#include <linux/init.h>
/* needed for remap_page_range 
	SetPageReserved
	ClearPageReserved
*/
#include <linux/mm.h>
/* obviously, for kmalloc */
#include <linux/slab.h>
/* for struct file_operations, register_chrdev() */
#include <linux/fs.h>
/* standard error codes */
#include <linux/errno.h>

#include <linux/dma-direct.h>
#include <linux/dma-buf.h>

#include <linux/moduleparam.h>
/* request_irq(), free_irq() */
#include <linux/interrupt.h>
#include <linux/sched.h>

#include <linux/semaphore.h>
#include <linux/spinlock.h>
/* needed for virt_to_phys() */
#include <asm/io.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>

#include <asm/irq.h>

#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/parser.h>

#include <linux/interrupt.h>
#include <linux/cdev.h>

#include <linux/clk.h>
#include <linux/hash.h>
#include <linux/hashtable.h>


//#define HX280ENC_DEBUG

/* our own stuff */
#include "hx280enc.h"
#include "memalloc.c"

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Verisilicon");
MODULE_DESCRIPTION("H2 Encoder driver");

#define ENCODER_DEVICE_NAME "hx280enc"
#define ENCODER_DEVICE_MAXCNT 1


/* this is ARM Integrator specific stuff */
#define HW_INTERRUPTS__H265_INT_IRQ    (109)
#define INTEGRATOR_LOGIC_MODULE0_BASE   0x7c030000
/*
#define INTEGRATOR_LOGIC_MODULE1_BASE   0xD0000000
#define INTEGRATOR_LOGIC_MODULE2_BASE   0xE0000000
#define INTEGRATOR_LOGIC_MODULE3_BASE   0xF0000000
*/

/*
#define INT_EXPINT1                     10
#define INT_EXPINT2                     11
#define INT_EXPINT3                     12
*/
/* these could be module params in the future */

#define ENC_IO_BASE                 INTEGRATOR_LOGIC_MODULE0_BASE
#define ENC_IO_SIZE                 (376 * 4)    /* bytes */

#define ENC_HW_ID1                  0x48320100
#define ENC_HW_ID2                  0x80006000

#define HX280ENC_BUF_SIZE           0
#define HX280ENC_BKT_NUM            10
#define HX280ENC_HASH_BITS          32

#define HX280ENC_FRAME_TIME_TENTH_SEC 10
#define HX280ENC_TMO_SAFTY_FACTOR   3

long irq_timeout_jiffies = (HZ / 100) * (HX280ENC_FRAME_TIME_TENTH_SEC * HX280ENC_TMO_SAFTY_FACTOR);
unsigned long base_port = INTEGRATOR_LOGIC_MODULE0_BASE;
int irq = HW_INTERRUPTS__H265_INT_IRQ;

/* module_param(name, type, perm) */
module_param(base_port, ulong, 0);
module_param(irq, int, 0);

/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
static int hx280enc_major = 0;
static int hx280enc_minor = 0;
static struct class *hx280enc_class;
static unsigned int devise_register_index = 0;

/* here's all the must remember stuff */
typedef struct
{
    char *buffer;
    unsigned int buffsize;
    unsigned long iobaseaddr;
    unsigned int iosize;
    volatile u8 *hwregs;
    unsigned int irq;
    struct fasync_struct *async_queue;
    unsigned int mem_base;
    unsigned int mem_size;
    struct device*       dma_dev;
    struct cdev cdev;
	dev_t devt;
    struct clk *hclk;
    struct clk *clk;
    struct class *class;
    struct hlist_head proc_refcount[1 << (HX280ENC_BKT_NUM)];
    struct hlist_head shared_dmabufs[1 << (HX280ENC_BKT_NUM)];
    struct mutex mlock;
    struct mutex mlock_shared_dmabufs;
} hx280enc_t;

struct hx280enc_h_node
{
    int pid;
    unsigned int refcount;
    struct hlist_node node;
};

struct hx280enc_dmabuf_node
{
    int fd;
    int tgid;
    struct dma_buf *dmabuf;
    struct dma_buf_attachment *attachment;
    struct sg_table *sgt;
    struct hlist_node node;
};

/* dynamic allocation? */
static hx280enc_t hx280enc_data;


static int ReserveIO(void);
static void ReleaseIO(void);
static void ResetAsic(hx280enc_t * dev);
static long hx280enc_virt_to_phys(unsigned long virt,unsigned long size, phys_addr_t *paddr);
static long hx280enc_pfn_virt_to_phys(struct vm_area_struct *vma, unsigned long vaddr,
    unsigned long size, phys_addr_t *paddr);
static int _hx280enc_release_all_dmabufs(int tgid);
static int hx280enc_share_dmabuf(int fd, unsigned long *o_paddr);
static int hx280enc_unshare_dmabuf(int fd);


#ifdef HX280ENC_DEBUG
static void dump_regs(unsigned long data);
#endif

/* IRQ handler */
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
static irqreturn_t hx280enc_isr(int irq, void *dev_id, struct pt_regs *regs);
#else
static irqreturn_t hx280enc_isr(int irq, void *dev_id);
#endif

static int enc_irq = 0;
volatile unsigned int asic_status = 0;

DEFINE_SPINLOCK(owner_lock);
DECLARE_WAIT_QUEUE_HEAD(enc_wait_queue);

static int CheckEncIrq(hx280enc_t *dev)
{
    unsigned long flags;
    int rdy = 0;
    spin_lock_irqsave(&owner_lock, flags);

    if(enc_irq)
    {
        /* reset the wait condition(s) */
        enc_irq = 0;
        rdy = 1;
    }

    spin_unlock_irqrestore(&owner_lock, flags);

    return rdy;
}
unsigned int WaitEncReady(hx280enc_t *dev)
{
    

	if(!wait_event_timeout(enc_wait_queue, CheckEncIrq(dev), irq_timeout_jiffies))
   {
	   pr_err("%s - wait_event_timeout timed out\n", __func__);
	   return -ETIMEDOUT;
   }

   //pr_info("wait_event_interruptible DONE\n");

    return 0;
}
static long hx280enc_ioctl(struct file *filp,
                          unsigned int cmd, unsigned long arg)
{
    int err = 0;

    unsigned int tmp;
    PDEBUG("ioctl cmd 0x%08ux\n", cmd);
    /*
     * extract the type and number bitfields, and don't encode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if(_IOC_TYPE(cmd) != HX280ENC_IOC_MAGIC)
        return -ENOTTY;
    if(_IOC_NR(cmd) > HX280ENC_IOC_MAXNR)
        return -ENOTTY;

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */
    if(_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok((void *) arg, _IOC_SIZE(cmd));
    else if(_IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok((void *) arg, _IOC_SIZE(cmd));
    if(err)
        return -EFAULT;

    switch (cmd)
    {
    case HX280ENC_IOCGHWOFFSET:
        __put_user(hx280enc_data.iobaseaddr, (unsigned long *) arg);
        break;

    case HX280ENC_IOCGHWIOSIZE:
        __put_user(hx280enc_data.iosize, (unsigned int *) arg);
        break;
    
    case HX280ENC_IOCG_CORE_WAIT:
    {
        //pr_info("HX280ENC_IOCG_CORE_WAIT\n");
        tmp = WaitEncReady(&hx280enc_data);
        if(tmp)
            return tmp;
        __put_user(asic_status, (unsigned int *)arg);
        break;
    }
    case HX280ENC_IOCXVIRT2BUS:
        address_translation_t *addresses = (address_translation_t *)arg;
        long rc = -EINVAL;
        phys_addr_t paddr = 0;
        rc = hx280enc_virt_to_phys(addresses->vaddr, addresses->size, &paddr);
        if (rc < 0)
        {
            pr_err("%s: Could not retrieve physical address of address %08lx, size %08lx", 
                    __func__, addresses->vaddr, addresses->size);
            return rc;
        }
        __put_user(paddr, (unsigned long *)&(addresses->paddr));
        break;
    case HX280ENC_IOC_SHARE_DMABUF:
        dmabuf_mapping_t *dmabuf_mapping = (dmabuf_mapping_t *)arg;
        unsigned long dmabuf_phys = 0;
        if (hx280enc_share_dmabuf(dmabuf_mapping->fd, &dmabuf_phys))
        {
            pr_err("%s: Could not share dmabuf for fd %d", __func__, dmabuf_mapping->fd);
            return -EINVAL;
        }
        __put_user(dmabuf_phys, (unsigned long *)&(dmabuf_mapping->paddr));
        break;
    case HX280ENC_IOC_UNSHARE_DMABUF:
        int *fd = (int *)arg;
        mutex_lock(&hx280enc_data.mlock_shared_dmabufs);
        if(hx280enc_unshare_dmabuf(*fd))
        {
            pr_err("%s: Could not unshare dmabuf for fd %d", __func__, *fd);
            mutex_unlock(&hx280enc_data.mlock_shared_dmabufs);
            return -EINVAL;
        }
        mutex_unlock(&hx280enc_data.mlock_shared_dmabufs);
        break;
    default:
        return memalloc_ioctl(filp, cmd, arg);
        
    }
    return 0;
}

static int _hx280enc_get_proc_refcount_node(int pid, struct hx280enc_h_node **refcount_node) {
    struct hx280enc_h_node *curr;
    hash_for_each_possible(hx280enc_data.proc_refcount, curr, node, hash_64(pid, HX280ENC_HASH_BITS)) {
        if(curr->pid == pid){
            *refcount_node = curr;
            return 0;
        }
    }

    return -ENAVAIL;
}

static int hx280enc_open(struct inode *inode, struct file *filp)
{
    int result = 0;
    int cur_pid = 0;
    hx280enc_t *dev = &hx280enc_data;
    struct hx280enc_h_node *refcount_node = NULL;

    mutex_lock(&hx280enc_data.mlock);
    cur_pid = current->tgid;

    filp->private_data = (void *) dev;
    if(_hx280enc_get_proc_refcount_node(cur_pid, &refcount_node)) {
        refcount_node = kzalloc(sizeof(struct hx280enc_h_node), GFP_KERNEL);
        if (!refcount_node) {
            pr_err("%s - failed to allocate refcount node\n", __func__);
            mutex_unlock(&hx280enc_data.mlock);
            return -ENOMEM;
        }
        refcount_node->refcount = 1;
        refcount_node->pid = cur_pid;
        hash_add(hx280enc_data.proc_refcount, &refcount_node->node, hash_64(cur_pid, HX280ENC_HASH_BITS));
    }
    else {
        refcount_node->refcount++;
    }
    mutex_unlock(&hx280enc_data.mlock);

    PDEBUG("dev opened\n");
    return result;
}
static int hx280enc_release(struct inode *inode, struct file *filp)
{
    int cur_pid = 0;
    struct hx280enc_h_node *refcount_node = NULL;

    mutex_lock(&hx280enc_data.mlock);
    cur_pid = current->tgid;

    if(_hx280enc_get_proc_refcount_node(cur_pid, &refcount_node)) {
        pr_err("%s - releasing pid %d does not exist\n", __func__, cur_pid);
        mutex_unlock(&hx280enc_data.mlock);
        return -EINTR;
    }

    refcount_node->refcount--;
    if(refcount_node->refcount == 0) {
        ResetProcMems(cur_pid);
        _hx280enc_release_all_dmabufs(cur_pid);
        hash_del(&refcount_node->node);
        kfree(refcount_node);
    }
    mutex_unlock(&hx280enc_data.mlock);

    return 0;
}

bool hx280enc_cacheable(unsigned long pfn, unsigned long n_pages)
{
    unsigned long i;

    for (i = 0; i < n_pages; ++i)
        if (!pfn_valid(pfn + i))
            return false;
    return true;
}

static bool hx280enc_vma_needs_cache_ops(struct vm_area_struct *vma)
{
    pgprot_t prot = vma->vm_page_prot;

    return pgprot_val(prot) != pgprot_val(pgprot_noncached(prot)) &&
        pgprot_val(prot) != pgprot_val(pgprot_writecombine(prot));
}

static long hx280enc_virt_to_phys(unsigned long virt, unsigned long size, phys_addr_t *paddr)
{
    struct mm_struct *mm = current->mm;
    struct vm_area_struct *vma;
    unsigned long n_pages;
    long rc = -EINVAL;
    phys_addr_t phys = ~0ul;
    bool do_cache = true;

    vma = find_vma(mm, virt);
    if (!vma) {
        pr_err("%s: no vma for vaddr/size = 0x%08lx/0x%08lx\n",
             __func__, virt, size);
        return -EINVAL;
    }
    n_pages = PFN_UP(virt + size) - PFN_DOWN(virt);
    /*
    * A range can only be mapped directly if it is either
    * uncached or HW-specific cache operations can handle it.
    */
    if (vma && vma->vm_flags & (VM_IO | VM_PFNMAP)) {
        pr_debug("%s: Got driver allocated buffer (addr: 0x%lx)\n",
                __func__, virt);
        rc = hx280enc_pfn_virt_to_phys(vma, virt, size, &phys);
        if (rc == 0 && hx280enc_vma_needs_cache_ops(vma) &&
            !hx280enc_cacheable(PFN_DOWN(phys), n_pages)) {
            pr_debug("%s: needs unsupported cache mgmt\n",
                    __func__);
            rc = -EINVAL;
        }
    } else {
        pr_err("%s: User allocated buffer (addr: 0x%lx) not supported\n",
                __func__, virt);
        rc = -EINVAL;
    }

    /* We couldn't share it. Fail the request. */
    if (rc < 0) {
        pr_err("%s: couldn't map virt to phys\n",
                __func__);
        return rc;
    }

    if (rc == 0 && vma && !hx280enc_vma_needs_cache_ops(vma))
        do_cache = false;

    if (do_cache)
        dma_sync_single_for_device(hx280enc_data.dma_dev, phys_to_dma(hx280enc_data.dma_dev, phys),
                                   size, DMA_TO_DEVICE);

    *paddr = phys;
    return 0;
}

static int _hx280enc_get_dmabuf_node(int fd, struct hx280enc_dmabuf_node **dmabuf_node) {
    struct hx280enc_dmabuf_node *curr;
    hash_for_each_possible(hx280enc_data.shared_dmabufs, curr, node, hash_64(fd, HX280ENC_HASH_BITS)) {
        if (curr->fd == fd) {
            *dmabuf_node = curr;
            return 0;
        }
    }
    return -ENAVAIL;
}

static int _hx280enc_release_all_dmabufs(int tgid) {
    struct hx280enc_dmabuf_node *curr;
    int fd;
    int ret;
    mutex_lock(&hx280enc_data.mlock_shared_dmabufs);
    hash_for_each(hx280enc_data.shared_dmabufs, fd, curr, node) {
        if (curr->tgid == tgid) {
            ret = hx280enc_unshare_dmabuf(curr->fd);
            if(ret) {
                pr_warn("%s - pid: %d failed to unshare dmabuf for fd %d\n", __func__, tgid, curr->fd);
            }
        }
    }
    mutex_unlock(&hx280enc_data.mlock_shared_dmabufs);
    return 0;
}

static int hx280enc_share_dmabuf(int fd, unsigned long *o_paddr)
{
    struct dma_buf *dmabuf;
    struct dma_buf_attachment *attachment;
    struct sg_table *sgt;
    unsigned long paddr;
    struct hx280enc_dmabuf_node *dmabuf_node;
    int ret = 0;

    ret = _hx280enc_get_dmabuf_node(fd, &dmabuf_node);
    if (!ret) {
        pr_err("dmabuf for fd %d already shared\n", fd);
        return -EINVAL;
    }

    if (fd < 0) {
        pr_err("Invalid fd (%d)\n", fd);
        ret = -EINVAL;
        goto out;
    }
    // Get a reference to the dma_buf object
    dmabuf = dma_buf_get(fd);
    if (IS_ERR(dmabuf)) {
        pr_err("Failed to get dmabuf object for fd %d\n", fd);
        ret = PTR_ERR(dmabuf);
        goto out;
    }
    // Attach the dma_buf to get the scatter-gather table
    attachment = dma_buf_attach(dmabuf, hx280enc_data.dma_dev);
    if (IS_ERR(attachment)) {
        pr_err("Failed to attach dmabuf for fd %d\n", fd);
        ret = PTR_ERR(attachment);
        goto put_dmabuf;
    }
    // Get the scatter-gather table
    sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
    if (IS_ERR(sgt)) {
        pr_err("Failed to map dmabuf for fd %d\n", fd);
        ret = PTR_ERR(sgt);
        goto detach_dmabuf;
    }
    // Check if the buffer is contiguous
    if (sgt->nents > 1) {
        pr_err("Buffer is not contiguous for fd %d\n", fd);
        ret = -EINVAL;
        goto unmap_sgt;
    }
    // Get the physical base address of the buffer
    paddr = sg_dma_address(sgt->sgl);

    // save the dmabuf in the hash table
    dmabuf_node = kzalloc(sizeof(struct hx280enc_dmabuf_node), GFP_KERNEL);
    if (!dmabuf_node) {
        pr_err("Failed to allocate dmabuf node\n");
        ret = -ENOMEM;
        goto unmap_sgt;
    }
    dmabuf_node->fd = fd;
    dmabuf_node->dmabuf = dmabuf;
    dmabuf_node->attachment = attachment;
    dmabuf_node->sgt = sgt;
    dmabuf_node->tgid = current->tgid;
    mutex_lock(&hx280enc_data.mlock_shared_dmabufs);
    hash_add(hx280enc_data.shared_dmabufs, &dmabuf_node->node, hash_64(fd, HX280ENC_HASH_BITS));
    mutex_unlock(&hx280enc_data.mlock_shared_dmabufs);
    *o_paddr = paddr;
    return 0;
unmap_sgt:
    dma_buf_unmap_attachment(attachment, sgt, DMA_TO_DEVICE);
detach_dmabuf:
    dma_buf_detach(dmabuf, attachment);
put_dmabuf:
    dma_buf_put(dmabuf);
out:
    return ret;
}

static int hx280enc_unshare_dmabuf(int fd)
{
    // get the dmabuf node from the hash table
    struct hx280enc_dmabuf_node *dmabuf_node;
    if (!_hx280enc_get_dmabuf_node(fd, &dmabuf_node)) {
        // unmap the scatter-gather table
        dma_buf_unmap_attachment(dmabuf_node->attachment, dmabuf_node->sgt, DMA_TO_DEVICE);
        // detach the dma_buf
        dma_buf_detach(dmabuf_node->dmabuf, dmabuf_node->attachment);
        // put the dma_buf
        dma_buf_put(dmabuf_node->dmabuf);
        // remove the node from the hash table
        hash_del(&dmabuf_node->node);
        kfree(dmabuf_node);
        return 0;
    }
    pr_err("Failed to unshare dmabuf for fd: %d\n", fd);
    return -EINVAL;
}

static long hx280enc_pfn_virt_to_phys(struct vm_area_struct *vma, unsigned long vaddr,
    unsigned long size, phys_addr_t *paddr)
{
    int ret;
    unsigned long i;
    unsigned long nr_pages = PFN_UP(vaddr + size) - PFN_DOWN(vaddr);
    unsigned long pfn;

    ret = follow_pfn(vma, vaddr, &pfn);
    if (ret)
        return ret;

    *paddr = __pfn_to_phys(pfn) + (vaddr & ~PAGE_MASK);
    
    for (i = 1; i < nr_pages; ++i) {
        unsigned long next_pfn;
        phys_addr_t next_phys;

        ret = follow_pfn(vma, vaddr + (i << PAGE_SHIFT), &next_pfn);
        if (ret)
            return ret;
        if (next_pfn != pfn + 1) {
            pr_debug("%s: non-contiguous physical memory\n",
                 __func__);
            return -EINVAL;
        }
        next_phys = __pfn_to_phys(next_pfn);
        pfn = next_pfn;
    }
    pr_debug("%s: success, paddr: %pap\n", __func__, paddr);
    return 0;
}

static int hx280enc_mmap(struct file *filp,
				   struct vm_area_struct *vma)
{

	return remap_pfn_range(vma, vma->vm_start,
                    vma->vm_pgoff, vma->vm_end - vma->vm_start, pgprot_writecombine(vma->vm_page_prot));
                    //vma->vm_pgoff, vma->vm_end - vma->vm_start, pgprot_noncached(vma->vm_page_prot));
                    //vma->vm_pgoff, vma->vm_end - vma->vm_start, vma->vm_page_prot);
}
/* VFS methods */
static struct file_operations hx280enc_fops = {
	.owner= THIS_MODULE,
	.open = hx280enc_open,
	.release = hx280enc_release,
	.unlocked_ioctl = hx280enc_ioctl,
    .mmap = hx280enc_mmap,
	.fasync = NULL,
};

//int __init hx280enc_init(void)
static int vc8000e_probe(struct platform_device *pdev)
{
    int result,irq;
    struct device *dev = &pdev->dev;
    struct device_node *node;
    struct reserved_mem *rmem = NULL;
    int ret;

    if (pdev->id >= ENCODER_DEVICE_MAXCNT)
	{
		pr_err("%s:pdev id is %d error\n", __func__,pdev->id);
		return  -EINVAL;
	}

    hx280enc_data.hclk = devm_clk_get(dev, "hclk");
    if (IS_ERR(hx280enc_data.hclk)) {
        return dev_err_probe(dev, PTR_ERR(hx280enc_data.hclk), "unable to get hclk\n");
    }

    hx280enc_data.clk = devm_clk_get(dev, "clk");
    if (IS_ERR(hx280enc_data.clk)) {
        return dev_err_probe(dev, PTR_ERR(hx280enc_data.clk), "unable to get clk\n");
    }

    ret = clk_prepare_enable(hx280enc_data.hclk);
    if (ret) {
        dev_err(dev, "failed to enable hclk (error %d)\n", ret);
        return ret;
    }

    ret = clk_prepare_enable(hx280enc_data.clk);
    if (ret) {
        dev_err(dev, "failed to enable clk (error %d)\n", ret);
        clk_disable_unprepare(hx280enc_data.hclk);
        return ret;
    }

    irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

    node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (node)
		rmem = of_reserved_mem_lookup(node);
	of_node_put(node);

	if (!rmem) {
		dev_err(dev, "unable to acquire memory-region\n");
		return -EINVAL;
	}

    printk(KERN_INFO "hx280enc: module init - base_port=0x%08lx irq=%i\n",
           base_port, irq);

    pr_info("Allocated reserved memory, paddr: 0x%0llX, size =0x%llx\n", rmem->base, rmem->size);

    memalloc_init((unsigned int )rmem->base, (unsigned int) (rmem->size/(1024*1024)));


    hx280enc_data.mem_base = (unsigned int )rmem->base;
    hx280enc_data.mem_size = (unsigned int )rmem->size;
    hx280enc_data.iobaseaddr = base_port;
    hx280enc_data.iosize = ENC_IO_SIZE;
    hx280enc_data.irq = irq;
    hx280enc_data.async_queue = NULL;
    hx280enc_data.hwregs = NULL;
    hx280enc_data.dma_dev = &pdev->dev;

    if (devise_register_index == 0)
	{
        ret = alloc_chrdev_region(&hx280enc_data.devt, 0, ENCODER_DEVICE_MAXCNT, ENCODER_DEVICE_NAME);
        if (ret != 0)
        {
            pr_err("%s:alloc_chrdev_region error\n", __func__);
            return ret;
        }
        hx280enc_major = MAJOR(hx280enc_data.devt);
        hx280enc_minor = MINOR(hx280enc_data.devt);

        hx280enc_class = class_create(THIS_MODULE, ENCODER_DEVICE_NAME);
        if (IS_ERR(hx280enc_class))
        {
            pr_err("%s[%d]:class_create error!\n", __func__, __LINE__);
            return -EINVAL;
        }
    }

    hash_init(hx280enc_data.proc_refcount);
    mutex_init(&hx280enc_data.mlock);

    hash_init(hx280enc_data.shared_dmabufs);
    mutex_init(&hx280enc_data.mlock_shared_dmabufs);

    hx280enc_data.devt = MKDEV(hx280enc_major, hx280enc_minor + pdev->id);
    cdev_init(&hx280enc_data.cdev, &hx280enc_fops);
	ret = cdev_add(&hx280enc_data.cdev, hx280enc_data.devt, 1);
	if ( ret )
	{
		pr_err("%s[%d]:cdev_add error!\n", __func__, __LINE__);
		return ret;
	}
	hx280enc_data.class = hx280enc_class;    
	device_create(hx280enc_data.class, NULL, hx280enc_data.devt,
			&hx280enc_data, "%s", ENCODER_DEVICE_NAME);

    devise_register_index++;

#if 0
    result = register_chrdev(hx280enc_major, "hx280enc", &hx280enc_fops);
	if (result < 0) {
		pr_err("hx280enc: unable to get major <%d>\n",
		hx280enc_major);
		return result;
	} else if (result != 0) /* this is for dynamic major */
		hx280enc_major = result;
#endif

    result = ReserveIO();
    if(result < 0)
    {
        goto err;
    }

    ResetAsic(&hx280enc_data);  /* reset hardware */

    /* get the IRQ line */
    if(irq != -1)
    {
        /*
        result = request_irq(irq, hx280enc_isr,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
                             SA_INTERRUPT | SA_SHIRQ,
#else
                             IRQF_SHARED,
                             //0,
#endif
                             "hx280enc", (void *) &hx280enc_data);
      */
        result = devm_request_irq(&pdev->dev, irq, hx280enc_isr,
				       IRQF_SHARED, "hx280enc", (void *) &hx280enc_data);
		
        if(result == -EINVAL)
        {
            printk(KERN_ERR "hx280enc: Bad irq number or handler\n");
            ReleaseIO();
            goto err;
        }
        else if(result == -EBUSY)
        {
            printk(KERN_ERR "hx280enc: IRQ <%d> busy, change your config\n",
                   hx280enc_data.irq);
            ReleaseIO();
            goto err;
        }
        pr_info("----> irq=%d is registered: ret=%d\n", irq,result);
    }
    else
    {
        printk(KERN_INFO "hx280enc: IRQ not in use!\n");
    }

    printk(KERN_INFO "hx280enc: module inserted. Major <%d>\n", hx280enc_major);

    return 0;

  err:
    //unregister_chrdev(hx280enc_major, "hx280enc");
    cdev_del(&hx280enc_data.cdev);
	device_destroy(hx280enc_data.class, hx280enc_data.devt);
	unregister_chrdev_region(hx280enc_data.devt, ENCODER_DEVICE_MAXCNT);
	if (devise_register_index == 0)
	{
		class_destroy(hx280enc_data.class);
	}
    printk(KERN_INFO "hx280enc: module not inserted\n");
    return result;
}

//void __exit hx280enc_cleanup(void)
static int vc8000e_remove(struct platform_device *pdev)
{
    unsigned int bkt;
    struct hx280enc_h_node *tmp;

    writel(0, hx280enc_data.hwregs + 0x14); /* disable HW */
    writel(0, hx280enc_data.hwregs + 0x04); /* clear enc IRQ */

    devise_register_index--;
    /* free the encoder IRQ */
    if(hx280enc_data.irq != -1)
    {        
        //free_irq(hx280enc_data.irq, (void *) &hx280enc_data);
    }

    ReleaseIO();

    clk_disable_unprepare(hx280enc_data.clk);
    clk_disable_unprepare(hx280enc_data.hclk);

    hash_for_each(hx280enc_data.proc_refcount, bkt, tmp, node) {
        pr_info("%s - deleting hash table node\n", __func__);
        hash_del(&tmp->node);
    }
    mutex_destroy(&hx280enc_data.mlock);

    cdev_del(&hx280enc_data.cdev);
	device_destroy(hx280enc_data.class, hx280enc_data.devt);
	unregister_chrdev_region(hx280enc_data.devt, ENCODER_DEVICE_MAXCNT);
	if (devise_register_index == 0)
	{
		class_destroy(hx280enc_data.class);
	}
    //unregister_chrdev(hx280enc_major, "hx280enc");

    memalloc_cleanup();

    printk(KERN_INFO "hx280enc: module removed\n");
    return 0;
}

//module_init(hx280enc_init);
//module_exit(hx280enc_cleanup);

static int ReserveIO(void)
{
    long int hwid;

    if(!request_mem_region
       (hx280enc_data.iobaseaddr, hx280enc_data.iosize, "hx280enc"))
    {
        printk(KERN_INFO "hx280enc: failed to reserve HW regs\n");
        return -EBUSY;
    }

    hx280enc_data.hwregs =
        (volatile u8 *) ioremap(hx280enc_data.iobaseaddr,
                                        hx280enc_data.iosize);

    if(hx280enc_data.hwregs == NULL)
    {
        printk(KERN_INFO "hx280enc: failed to ioremap HW regs\n");
        ReleaseIO();
        return -EBUSY;
    }

    hwid = readl(hx280enc_data.hwregs);
    printk(KERN_INFO "hx280enc: hwid=0x%lx\n", hwid);

#if 1
    /* check for encoder HW ID */
    if( ((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID1 >> 16) & 0xFFFF))) &&
       ((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID2 >> 16) & 0xFFFF))))
    {
        printk(KERN_INFO "hx280enc: HW not found at 0x%08lx\n",
               hx280enc_data.iobaseaddr);
#ifdef HX280ENC_DEBUG
        //dump_regs((unsigned long) &hx280enc_data);
#endif
        ReleaseIO();
        return -EBUSY;
    }
#endif
    printk(KERN_INFO
           "hx280enc: HW at base <0x%08lx> with ID <0x%08lx>\n",
           hx280enc_data.iobaseaddr, hwid);

    return 0;
}

static void ReleaseIO(void)
{
    if(hx280enc_data.hwregs)
        iounmap((void *) hx280enc_data.hwregs);
    release_mem_region(hx280enc_data.iobaseaddr, hx280enc_data.iosize);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
irqreturn_t hx280enc_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
irqreturn_t hx280enc_isr(int irq, void *dev_id)
#endif
{
    unsigned int handled = 0;
    hx280enc_t *dev = (hx280enc_t *) dev_id;
    u32 irq_status;
	
    irq_status = readl(dev->hwregs + 0x04);
    //pr_info("h265 interrupt: irq_status=0x%x",irq_status);
    if(irq_status & 0x01)
    {
        /* clear all IRQ bits. (hwId >= 0x80006001) means IRQ is cleared by writting 1 */
        u32 hwId = readl(dev->hwregs);
        u32 majorId = (hwId & 0x0000FF00) >> 8;
        u32 minorId = (hwId & 0x000000FF);
        u32 wClr = (majorId >= 0x61 || (majorId == 0x60 && minorId >= 1)) ? irq_status: (irq_status & (~0x1FD));
        writel(wClr, dev->hwregs + 0x04);

        enc_irq = 1;
        asic_status = irq_status & (~0x01);
        wake_up_all(&enc_wait_queue);
        handled++;
    }
    if(!handled)
    {
        PDEBUG("IRQ received, but not x170's!\n");
    }
    return IRQ_HANDLED;
}

void ResetAsic(hx280enc_t * dev)
{
    int i;

    writel(0, dev->hwregs + 0x14);
    for(i = 4; i < dev->iosize; i += 4)
    {
        writel(0, dev->hwregs + i);
    }
}

#ifdef HX280ENC_DEBUG
void dump_regs(unsigned long data)
{
    hx280enc_t *dev = (hx280enc_t *) data;
    int i;

    PDEBUG("Reg Dump Start\n");
    for(i = 0; i < dev->iosize; i += 4)
    {
        PDEBUG("\toffset %02X = %08X\n", i, readl(dev->hwregs + i));
    }
    PDEBUG("Reg Dump End\n");
}
#endif


static const struct of_device_id vc8000e_of_match[] = {
	{ .compatible = "vivante,vc8000e" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, vc8000e_of_match);

static struct platform_driver vc8000e_platform_driver = {
	.driver = {
		.name		= "vc8000e",
		.of_match_table	= vc8000e_of_match,
	},
	.probe			= vc8000e_probe,
	.remove			= vc8000e_remove,
};

module_platform_driver(vc8000e_platform_driver);
