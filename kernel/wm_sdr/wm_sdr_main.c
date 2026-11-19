// SPDX-License-Identifier: GPL-2.0-only
/*
 * wm_sdr_main.c - /dev/wmsdr character device and ioctl dispatch.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include "wm_sdr.h"

static bool claim_pci;
module_param(claim_pci, bool, 0644);
MODULE_PARM_DESC(claim_pci,
		 "1 = claim 14c3:7981 and use the ROM FWDL loader (requires mt_wifi unloaded)");

static struct wmsdr_dev *wmsdr;

static int wmsdr_open(struct inode *inode, struct file *file)
{
	file->private_data = wmsdr;
	return 0;
}

static int wmsdr_release(struct inode *inode, struct file *file)
{
	struct wmsdr_dev *w = file->private_data;

	if (w)
		wmsdr_net_release_file(w, file);
	return 0;
}

static long wmsdr_ioctl_info(struct wmsdr_dev *w, unsigned long arg)
{
	struct wmsdr_info info;

	memset(&info, 0, sizeof(info));
	info.version = 1;
	info.backend = w->pci_claimed ? WMSDR_BACKEND_PCI : WMSDR_BACKEND_REMAP;
	info.fw_state = wmsdr_fw_state(w);
	info.fw_stat_addr = WMSDR_BAR_PHYS + WMSDR_FW_STAT;
	info.bar_phys = WMSDR_BAR_PHYS;
	info.bar_len = WMSDR_BAR_LEN;
	info.remap_ctl = WMSDR_REMAP_CTL;
	info.remap_win = WMSDR_REMAP_WIN;
	info.chip_vendor = 0x14c3;
	info.chip_device = 0x7981;
	info.loaded = 0;
	info.flags = w->dma_ready ? WMSDR_INFO_F_DMA_READY : 0;

	if (copy_to_user((void __user *)arg, &info, sizeof(info)))
		return -EFAULT;
	return 0;
}

static long wmsdr_ioctl_rd32(struct wmsdr_dev *w, unsigned long arg)
{
	struct wmsdr_word word;

	if (copy_from_user(&word, (void __user *)arg, sizeof(word)))
		return -EFAULT;
	if (!wmsdr_addr_ok(word.addr, 4) || (word.addr & 3))
		return -EINVAL;
	word.value = wmsdr_remap_read32(w, word.addr);
	if (copy_to_user((void __user *)arg, &word, sizeof(word)))
		return -EFAULT;
	return 0;
}

static long wmsdr_ioctl_wr32(struct wmsdr_dev *w, unsigned long arg)
{
	struct wmsdr_word word;

	if (copy_from_user(&word, (void __user *)arg, sizeof(word)))
		return -EFAULT;
	if (!wmsdr_addr_ok(word.addr, 4) || (word.addr & 3))
		return -EINVAL;
	wmsdr_remap_write32(w, word.addr, word.value);
	return 0;
}

static long wmsdr_ioctl_direct_rd32(struct wmsdr_dev *w, unsigned long arg)
{
	struct wmsdr_word word;

	if (copy_from_user(&word, (void __user *)arg, sizeof(word)))
		return -EFAULT;
	if (!wmsdr_direct_addr_ok(word.addr))
		return -EINVAL;
	word.value = wmsdr_direct_read32(w, word.addr);
	if (copy_to_user((void __user *)arg, &word, sizeof(word)))
		return -EFAULT;
	return 0;
}

static long wmsdr_ioctl_mem_read_common(struct wmsdr_dev *w, unsigned long arg,
					bool use_dma)
{
	struct wmsdr_mem mem;
	void *buf;
	int ret;

	if (copy_from_user(&mem, (void __user *)arg, sizeof(mem)))
		return -EFAULT;
	if (mem.len == 0 || mem.len > WMSDR_MAX_XFER)
		return -EINVAL;
	if (use_dma ? !wmsdr_wfdma_addr_ok(mem.addr, mem.len) :
		      !wmsdr_addr_ok(mem.addr, mem.len))
		return -EINVAL;

	buf = kvmalloc(mem.len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = use_dma ? wmsdr_wfdma_read(w, mem.addr, buf, mem.len) :
			wmsdr_remap_read(w, mem.addr, buf, mem.len);
	if (!ret && copy_to_user(u64_to_user_ptr(mem.user_ptr), buf, mem.len))
		ret = -EFAULT;

	kvfree(buf);
	return ret;
}

static long wmsdr_ioctl_read_mem(struct wmsdr_dev *w, unsigned long arg)
{
	return wmsdr_ioctl_mem_read_common(w, arg, false);
}

static long wmsdr_ioctl_dma_read(struct wmsdr_dev *w, unsigned long arg)
{
	return wmsdr_ioctl_mem_read_common(w, arg, true);
}

static long wmsdr_ioctl_dma_mmap(struct wmsdr_dev *w, unsigned long arg)
{
	struct wmsdr_mmap_req req;
	int ret = 0;

	if (!w->dma_ready || !w->rx_pages)
		return -EOPNOTSUPP;
	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;
	if (req.len)
		ret = wmsdr_wfdma_mmap_read(w, req.addr, req.len);
	if (ret)
		return ret;
	req.data_off = w->rx_data_off;
	req.map_size = w->rx_map_size;
	if (copy_to_user((void __user *)arg, &req, sizeof(req)))
		return -EFAULT;
	return 0;
}

static long wmsdr_ioctl_icap_dma_mmap(struct wmsdr_dev *w, unsigned long arg)
{
	struct wmsdr_icap_mmap_req req;
	int ret;

	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;
	ret = wmsdr_wfdma_icap_mmap(w, &req);
	if (copy_to_user((void __user *)arg, &req, sizeof(req)))
		return -EFAULT;
	return ret;
}

static int wmsdr_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct wmsdr_dev *w = file->private_data;

	return wmsdr_wfdma_mmap(w, vma);
}

static long wmsdr_ioctl_write_mem(struct wmsdr_dev *w, unsigned long arg)
{
	struct wmsdr_mem mem;
	void *buf;
	int ret;

	if (copy_from_user(&mem, (void __user *)arg, sizeof(mem)))
		return -EFAULT;
	if (mem.len == 0 || mem.len > WMSDR_MAX_XFER)
		return -EINVAL;
	if (!wmsdr_addr_ok(mem.addr, mem.len))
		return -EINVAL;

	buf = kvmalloc(mem.len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (copy_from_user(buf, u64_to_user_ptr(mem.user_ptr), mem.len)) {
		kvfree(buf);
		return -EFAULT;
	}

	ret = wmsdr_remap_write(w, mem.addr, buf, mem.len);
	kvfree(buf);
	return ret;
}

static long wmsdr_unlocked_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	struct wmsdr_dev *w = file->private_data;
	u32 state;

	switch (cmd) {
	case WMSDR_IOC_GET_INFO:
		return wmsdr_ioctl_info(w, arg);
	case WMSDR_IOC_RD32:
		return wmsdr_ioctl_rd32(w, arg);
	case WMSDR_IOC_WR32:
		return wmsdr_ioctl_wr32(w, arg);
	case WMSDR_IOC_READ_MEM:
		return wmsdr_ioctl_read_mem(w, arg);
	case WMSDR_IOC_WRITE_MEM:
		return wmsdr_ioctl_write_mem(w, arg);
	case WMSDR_IOC_GET_FW_STATE:
		state = wmsdr_fw_state(w);
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			return -EFAULT;
		return 0;
	case WMSDR_IOC_DIRECT_RD32:
		return wmsdr_ioctl_direct_rd32(w, arg);
	case WMSDR_IOC_DMA_READ:
		return wmsdr_ioctl_dma_read(w, arg);
	case WMSDR_IOC_DMA_MMAP:
		return wmsdr_ioctl_dma_mmap(w, arg);
	case WMSDR_IOC_ICAP_DMA_MMAP:
		return wmsdr_ioctl_icap_dma_mmap(w, arg);
	case WMSDR_IOC_NET_START:
	case WMSDR_IOC_NET_ADD_SOCK:
	case WMSDR_IOC_NET_DEL_SOCK:
	case WMSDR_IOC_NET_STOP:
	case WMSDR_IOC_NET_STAT:
		return wmsdr_net_ioctl(w, file, cmd, arg);
	case WMSDR_IOC_LOAD_IMAGE:
	case WMSDR_IOC_START:
	case WMSDR_IOC_RESTART_DL:
		return -EOPNOTSUPP;	/* P3 */
	default:
		return -ENOTTY;
	}
}

static const struct file_operations wmsdr_fops = {
	.owner		= THIS_MODULE,
	.open		= wmsdr_open,
	.release	= wmsdr_release,
	.unlocked_ioctl	= wmsdr_unlocked_ioctl,
	.mmap		= wmsdr_mmap,
	.llseek		= no_llseek,
};

static int __init wmsdr_init(void)
{
	int ret;

	wmsdr = kzalloc(sizeof(*wmsdr), GFP_KERNEL);
	if (!wmsdr)
		return -ENOMEM;

	mutex_init(&wmsdr->lock);
	wmsdr->bar = ioremap(WMSDR_BAR_PHYS, WMSDR_BAR_LEN);
	if (!wmsdr->bar) {
		pr_err("wmsdr: ioremap(0x%lx) failed\n", (unsigned long)WMSDR_BAR_PHYS);
		ret = -ENOMEM;
		goto err_free;
	}

	if (claim_pci) {
		/* PCI backend is added in P3; do not pretend it works. */
		pr_err("wmsdr: claim_pci=1 is not implemented yet\n");
		ret = -EOPNOTSUPP;
		goto err_unmap;
	}

	wmsdr->backend = "remap";

	ret = alloc_chrdev_region(&wmsdr->devt, 0, 1, WMSDR_NAME);
	if (ret)
		goto err_unmap;

	cdev_init(&wmsdr->cdev, &wmsdr_fops);
	wmsdr->cdev.owner = THIS_MODULE;
	ret = cdev_add(&wmsdr->cdev, wmsdr->devt, 1);
	if (ret)
		goto err_region;

	wmsdr->class = class_create(THIS_MODULE, WMSDR_NAME);
	if (IS_ERR(wmsdr->class)) {
		ret = PTR_ERR(wmsdr->class);
		goto err_cdev;
	}
	wmsdr->device = device_create(wmsdr->class, NULL, wmsdr->devt, NULL,
				      "wmsdr");
	if (IS_ERR(wmsdr->device)) {
		ret = PTR_ERR(wmsdr->device);
		goto err_class;
	}

	wmsdr->fw_state = wmsdr_fw_state(wmsdr);
	wmsdr_wfdma_init(wmsdr);
	wmsdr_net_init(wmsdr);
	pr_info("wmsdr: ready (remap backend, dma=%u), bar=%#lx fw_state=%u\n",
		wmsdr->dma_ready, (unsigned long)WMSDR_BAR_PHYS, wmsdr->fw_state);
	return 0;

err_class:
	class_destroy(wmsdr->class);
err_cdev:
	cdev_del(&wmsdr->cdev);
err_region:
	unregister_chrdev_region(wmsdr->devt, 1);
err_unmap:
	iounmap(wmsdr->bar);
err_free:
	kfree(wmsdr);
	wmsdr = NULL;
	return ret;
}

static void __exit wmsdr_exit(void)
{
	if (!wmsdr)
		return;
	device_destroy(wmsdr->class, wmsdr->devt);
	class_destroy(wmsdr->class);
	cdev_del(&wmsdr->cdev);
	unregister_chrdev_region(wmsdr->devt, 1);
	wmsdr_net_exit(wmsdr);
	wmsdr_wfdma_exit(wmsdr);
	iounmap(wmsdr->bar);
	kfree(wmsdr);
	pr_info("wmsdr: exit\n");
}

module_init(wmsdr_init);
module_exit(wmsdr_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CT3003 WM memory access, ROM loader and IQ snapshot driver");
