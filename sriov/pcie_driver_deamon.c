////////////////////////////////////////
///@file pcie_driver_deamon.c
///
///@brief pcie驱动实例
///
/// 修改记录:
///
///@author wangdb@sugon.com
///@date 2018-04-18
///1，增加字符设备接口
///2，probe vf，通过/sys动态配置vf数量
///3，修正一些不稳定的问题
///
///@author douxg@sugon.com
///@date 2017-08-09
///
///建立文档
////////////////////////////////////////

//系统路径下头文件
#include <linux/module.h>
#include <linux/kernel.h>           //printk, KERN_INFO
#include <linux/types.h>            //u8, u32, s32
#include <linux/timer.h>            //init_timer, add_timer
#include <linux/errno.h>
#include <linux/ioport.h>           //request_region, release_region
#include <linux/slab.h>             //kmalloc, kfree
#include <linux/vmalloc.h>          //vmalloc, vfree
#include <linux/interrupt.h>        //request_irq, free_irq
#include <linux/pci.h>              //pci_register_driver, pci_unregister_driver
#include <linux/netdevice.h>        //alloc_etherdev, free_netdev
#include <linux/cdev.h>
#include <linux/init.h>             //module_init, module_exit
#include <linux/mii.h>              //struct mii_if_info
#include <linux/delay.h>            //ndelay, udelay, mdelay, msleep etc.
#include <linux/bitops.h>           //set_bit, clear_bit, test_and_set_bit
#include <linux/dma-mapping.h>      //dma_set_mask
#include <asm/io.h>                 //ioremap, iounmap
#include <asm/irq.h>                //disable_irq, enable_irq
#include <asm-generic/iomap.h>      //iowrite32, ioread32


//////// 宏定义  ///////


///@name 硬件参数
//@{
#define BAR_0				0
#define IOC_GET_BAR0        0
#define MAX_DEVICE_NUM      4
#define MAX_VF_NUM			15u
#define MAX_VDEVICE_NUM     (MAX_DEVICE_NUM*(MAX_VF_NUM+1))
#define DRIVER_NAME			"pcie_driver_deamon"
//@}

///////  数据结构 ///////////

struct pcie_driver_deamon_priv {
    unsigned long *bar0_addr;
    char dev_name[NAME_MAX];
    struct cdev cdev;
    dev_t dev;
};

////////// 全局数据  /////////////
static char          glbl_drv_name[]= DRIVER_NAME;
static struct class *pcie_driver_deamon_class;
static dev_t glbl_base_dev;
static int phy_device_num;
static long device_indexes;
static struct pci_dev * dev_ctl[MAX_VDEVICE_NUM];

///pci设备列表
static struct pci_device_id pcie_driver_deamon_table[] =
{
    //{0x4943, 0x4e43, PCI_ANY_ID, PCI_ANY_ID},
    {0x4943, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID},
    {0,}
};

static unsigned int max_vfs=MAX_VF_NUM;
module_param(max_vfs, uint, 0);
MODULE_PARM_DESC(max_vfs, "Maximum number of virtual functions to allocate per physical function");

MODULE_DEVICE_TABLE(pci, pcie_driver_deamon_table);
MODULE_AUTHOR("wangdb");
MODULE_DESCRIPTION("Enable SRIOV for netfirm card");
MODULE_LICENSE("GPL");

///////   函数实现列表  ///////////////

///@name 标准驱动函数
//@{

///@brief 打开（启动）设备
static int pcie_driver_deamon_open(struct inode *i, struct file *f)
{
    f->private_data = pci_dev_get(dev_ctl[MINOR(i->i_rdev)]);
    pr_info("%s: open......\n", kobject_name(&i->i_cdev->kobj));
    return 0;
}

///@brief 停止（关闭）设备
static int pcie_driver_deamon_stop(struct inode *i, struct file *f)
{
    pci_dev_put(f->private_data);
    pr_info("%s: close......\n", kobject_name(&i->i_cdev->kobj));
    return 0;
}

///@brief 读取设备寄存器地址
static ssize_t pcie_driver_deamon_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    char *b;
    size_t len;
    struct pcie_driver_deamon_priv *pdev_priv;
    char *name;
    const char *pname;
    int major;
    int minor;
    void *bar0;

    if( *f_pos)
        return 0;

    b = kzalloc(1024, GFP_KERNEL);
    if (b == NULL)
        return -ENOMEM;

    pdev_priv = pci_get_drvdata(filp->private_data);

    name = pdev_priv->dev_name;
    major = MAJOR(pdev_priv->dev);
    minor = MINOR(pdev_priv->dev);
    bar0 = pdev_priv->bar0_addr;
    pname = pci_name(filp->private_data);

    len = snprintf(b, 1024, "name\tdev\tpci_name\tbar0\n");
    len += snprintf(b+len, 1024-len, "%s\t%d:%d\t%s\t%p\n", name, major, minor, pname, bar0);
    len = min(len, count);
    if( copy_to_user(buf, b, len) != 0) {
        kfree(b);
        return -EIO;
    }
    *f_pos += len;

    kfree(b);

    return len;
}

///@brief 写入设备寄存器地址
static ssize_t pcie_driver_deamon_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    pr_info("%s: write......\n", kobject_name(&filp->f_inode->i_cdev->kobj));
    return 0;
}

///@brief iotcl接口
static long pcie_driver_deamon_ioctl( struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch(cmd)
    {
        case IOC_GET_BAR0:
            //*(long *)arg = (long)bar0_addr;
            break;
        default:
            return -EOPNOTSUPP;
    }
    pr_info("%s: ioctl......\n", kobject_name(&filp->f_inode->i_cdev->kobj));
    return 0;
}


static const struct file_operations deamon_ops = {
    .open     = &pcie_driver_deamon_open,
    .release  = &pcie_driver_deamon_stop,
    .read     = &pcie_driver_deamon_read,
    .write    = &pcie_driver_deamon_write,
    .unlocked_ioctl    = &pcie_driver_deamon_ioctl,
};


///@brief 设备探查，根据pci标志寻找设备
static int pcie_driver_deamon_probe(struct pci_dev *pci_dev,
        const struct pci_device_id *pci_id)
{
    unsigned long mmio_start, mmio_len;
    int ret;
    struct pcie_driver_deamon_priv * priv;
    struct device *pdev;
    int index, totalvfs;

    if(phy_device_num > MAX_DEVICE_NUM)
        return -EBUSY;

    index = find_first_zero_bit(&device_indexes, MAX_VDEVICE_NUM);
    if(index >= MAX_VDEVICE_NUM) {
		pr_err(DRIVER_NAME": index %d out of %d\n", index, MAX_VDEVICE_NUM);
        return -EBUSY;
	}
    set_bit(index, &device_indexes);

    priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if(priv == NULL)
        return -ENOMEM;

    snprintf(priv->dev_name, NAME_MAX, "nf%d", index);
    priv->dev = MKDEV(MAJOR(glbl_base_dev), MINOR(glbl_base_dev)+index);
    pr_info(DRIVER_NAME" %s: %d:%d\n", priv->dev_name, MAJOR(priv->dev), MINOR(priv->dev));


    ///初始化设备，使得I/O, memory可用，唤醒设备
    ret = pci_enable_device(pci_dev);
    if (ret)
    {
        pr_err("Cannot enable PCI device, aborting.\n");
        kfree(priv);
        return ret;
    }

    ///网卡I/O,memory资源的起始地址
    mmio_start = pci_resource_start(pci_dev, BAR_0);
    mmio_len = pci_resource_len(pci_dev, BAR_0);

    ///申请内存空间，配置网卡的I/O，memory资源。
    ret = pci_request_regions(pci_dev, glbl_drv_name);
    if (ret)
    {
        pr_err("Cannot obtain PCI resources, aborting.\n");
        kfree(priv);
        return ret;
    }

    pci_set_master(pci_dev);


    ///映射I/O,memory地址到私有域中的硬件逻辑地址
    priv->bar0_addr = ioremap_nocache(mmio_start, mmio_len);
    if (priv->bar0_addr == NULL)
    {
        pr_err("Cannot remap MMIO, aborting\n");
        ret = -EIO;
        goto err_out;
    }
    pci_set_drvdata(pci_dev, priv);


    ///注册字符设备
    cdev_init(&priv->cdev, &deamon_ops);
    kobject_set_name(&priv->cdev.kobj, "%s", priv->dev_name);
    if( (ret=cdev_add(&priv->cdev, priv->dev, 1)) < 0)
    {
        pr_err("Cannot register char device %s, aborting.\n", priv->dev_name);
        pci_iounmap(pci_dev, priv->bar0_addr);
        goto err_out;
    }

    ///建立 /dev/nfN 文件
    pdev = device_create(pcie_driver_deamon_class, NULL, priv->dev, NULL, priv->dev_name);
    if (IS_ERR(pdev)) {
        ret = PTR_ERR(pdev);
        cdev_del(&priv->cdev);
        pci_iounmap(pci_dev, priv->bar0_addr);
        goto err_out;
    }

    dev_ctl[MINOR(priv->dev)] = pci_dev;

    ///使能sriov
    if (pci_dev->is_physfn)
    {
        phy_device_num++;

		if(max_vfs > (ret=pci_sriov_get_totalvfs(pci_dev))){
			pr_warn(DRIVER_NAME" change max_vfs: %d\n", ret);
			max_vfs = ret;
		}
		pr_info(DRIVER_NAME" set max_vfs %d\n", max_vfs);
        if( (ret=pci_sriov_set_totalvfs(pci_dev, max_vfs)) < 0)
			pr_warn(DRIVER_NAME" set max_vfs fail: %d\n", ret);

        totalvfs = pci_sriov_get_totalvfs(pci_dev);
        if(totalvfs == 0)
            return 0;

        ret = pci_enable_sriov(pci_dev, totalvfs);
        if (ret < 0)
        {
            pr_err(DRIVER_NAME" %s SRIOV enabled.\n", pci_name(pci_dev));
            pci_dev_put(pci_dev);
            device_destroy(pcie_driver_deamon_class, priv->dev);
            cdev_del(&priv->cdev);
            pci_iounmap(pci_dev, priv->bar0_addr);
            goto err_out;
        }
        pr_info(DRIVER_NAME" %s SRIOV enabled.\n", pci_name(pci_dev));
    }

    pr_info(DRIVER_NAME" Probe device %s successfully!\n", pci_name(pci_dev));

    return 0;


err_out:
    ///释放pci资源
    pci_release_regions(pci_dev);

    ///PCI设备中的设备指针赋空
    pci_set_drvdata(pci_dev, NULL);
    kfree(priv);

    pr_info(DRIVER_NAME" Failed to install pcie_driver_deamon.ko!\n");

    return ret;
}


///@brief 移除设备
static void pcie_driver_deamon_remove(struct pci_dev *pci_dev)
{
    ///释放和清空设备
    struct pcie_driver_deamon_priv *priv;

    priv = pci_get_drvdata(pci_dev);
    dev_ctl[MINOR(priv->dev)] = NULL;

    device_destroy(pcie_driver_deamon_class, priv->dev);
    cdev_del(&priv->cdev);

	pci_sriov_set_totalvfs(pci_dev, 0);
    pci_disable_sriov(pci_dev);
    pci_iounmap(pci_dev, priv->bar0_addr);
    pci_release_regions(pci_dev);
    pci_set_drvdata(pci_dev, NULL);

    clear_bit(MINOR(priv->dev), &device_indexes);
    if (pci_dev->is_physfn)
        phy_device_num--;

    pr_info(DRIVER_NAME" Remove device %s succeesfully!\n", pci_name(pci_dev));

}

static int pcie_driver_deamon_sriov_configure(struct pci_dev *dev, int num_vfs)
{
    if (num_vfs == 0) {
        pci_disable_sriov(dev);
        return 0;
    }
    else
        return pci_enable_sriov(dev, num_vfs);
}

static struct pci_driver deamon_pci_driver = {
    .name            = glbl_drv_name,
    .id_table        = pcie_driver_deamon_table,
    .probe           = pcie_driver_deamon_probe,
    .remove          = pcie_driver_deamon_remove,
    .sriov_configure = pcie_driver_deamon_sriov_configure
};

///设备驱动模块加载函数
static int __init pcie_driver_deamon_init(void)
{
    int ret;
#ifdef MODULE
    pr_info("%s is initialized now\n", glbl_drv_name);
#endif 

    max_vfs = min(max_vfs, MAX_VF_NUM);

    if((ret=alloc_chrdev_region(&glbl_base_dev, 0, MAX_VDEVICE_NUM, glbl_drv_name)) < 0)
        return ret;

    pcie_driver_deamon_class = class_create(THIS_MODULE, "netfirm");
    if (IS_ERR(pcie_driver_deamon_class)) {
        unregister_chrdev_region(glbl_base_dev, MAX_VDEVICE_NUM);
        return PTR_ERR(pcie_driver_deamon_class);
    }

    ///注册设备
    if( (ret = pci_register_driver(&deamon_pci_driver)) < 0) {
        class_destroy(pcie_driver_deamon_class);
        unregister_chrdev_region(glbl_base_dev, MAX_VDEVICE_NUM);
    }
    return ret;
}

static void __exit pcie_driver_deamon_exit(void)
{
#ifdef MODULE
    pr_info("%s is exited now\n", glbl_drv_name);
#endif
    pci_unregister_driver(&deamon_pci_driver);
    class_destroy(pcie_driver_deamon_class);
    unregister_chrdev_region(glbl_base_dev, MAX_VDEVICE_NUM);
}

module_init(pcie_driver_deamon_init);
module_exit(pcie_driver_deamon_exit);

//@}
