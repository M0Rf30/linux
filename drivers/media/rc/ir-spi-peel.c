/*
* Copyright (C) 2014  Peel Technologies Inc
* Copyright (C) 2020 XiaoMi, Inc.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include <linux/mm.h>
#include <linux/slab.h>

#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include "ir-spi-peel.h"

#include <asm/uaccess.h>
#include <asm/delay.h>

#define IR_SPI_PEEL_DRIVER_NAME		"ir-spi-peel"

#define SPI_MODE_MASK       (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

#define LR_EN       73

#define TRACE printk("%s: %d\n", __func__, __LINE__);

/*uncoment below defintion to use map
for write*/
#define USES_MMAP
struct ir_spi_peel_data {
    dev_t           devt;

    spinlock_t      spi_lock;
    unsigned        users;
    u8          *buffer;

	struct spi_device   *spi;
    struct mutex        buf_lock;
};

/*npages gets its value from the makefile
avoid changing it here*/
/*static unsigned int npages = CONFIG_NPAGES; */
static unsigned int npages = 150;
static unsigned bufsiz;  /* Default buffer size */
u32 is_gpio_used;
#ifndef CONFIG_OF
static int mode = 0, bpw = 32, spi_clk_freq = 960000;
#endif
static int lr_en, in_use , rcount;
static int prev_tx_status;  /* Status of previous transaction */
static u32 field;
static const char  *reg_id;
static struct regulator *ir_reg;
u8 *p_buf;
struct ir_spi_peel_data *peel_data_g;
#ifdef USES_MMAP
static void *kmalloc_ptr;
static int *kmalloc_area;
#endif
static struct spi_transfer t;


static int ir_regulator_set(bool enable)
{
    int rc = 0;

	if (ir_reg) {
		if (enable) {
			rc = regulator_enable(ir_reg);
		} else {
			rc = regulator_disable(ir_reg);
		}

	}
	return rc;
}


static inline int
ir_spi_peel_read(struct ir_spi_peel_data *peelir, size_t len)
{
	struct spi_message  m;
	t.rx_buf    = peelir->buffer;
	t.len       = len;
	t.tx_buf    = NULL;
	memset(peelir->buffer, 0, len); TRACE
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(peelir->spi, &m);
}

static int ir_spi_peel_read_message(struct ir_spi_peel_data *peelir,
		struct spi_ioc_transfer *u_xfers)
{
	u8          *buf;

	memset(peelir->buffer, 0, bufsiz);  /*Receive Buffer*/

	buf = peelir->buffer; TRACE
	if (u_xfers->len > bufsiz) {
		printk("%s: Requested too large data\n", __func__);
		return -EMSGSIZE;
	}

	/* Receiving IR input */

	pr_info("\n Waiting for IR data.... \n");
	pr_info("\n Press the Key\n");

	ir_spi_peel_read(peelir, bufsiz); TRACE

	/* copy any rx data to user space */
	if (u_xfers->rx_buf) {
		pr_info("\n%s:Copying data to user space\n", __func__);
		if (__copy_to_user((u8 __user *)
			(uintptr_t) u_xfers->rx_buf, buf, u_xfers->len)) {
			pr_info("\n%s:Copy to user space failed !!!\n", __func__);
			return -EFAULT;
		}
	}

	return 0;
}

static inline int
ir_spi_peel_write(struct ir_spi_peel_data *peelir, size_t len)
{
	struct spi_message m;

	t.tx_buf        = peelir->buffer;
	t.len           = len;
	t.bits_per_word     = peelir->spi->bits_per_word;
	spi_message_init(&m); TRACE
	spi_message_add_tail(&t, &m);
	return spi_sync(peelir->spi, &m);
}

static int ir_spi_peel_write_message(struct ir_spi_peel_data *peelir,
		struct spi_ioc_transfer *u_xfers)
{
	u8 *buf;
	int status = -EFAULT;

	buf = peelir->buffer; TRACE

	if (u_xfers->len > bufsiz)
		status = -EMSGSIZE;

	if (u_xfers->tx_buf)
		if (copy_from_user(buf, (const u8 __user *)
			(uintptr_t) u_xfers->tx_buf,
			u_xfers->len))

	peelir->spi->bits_per_word = u_xfers->bits_per_word;

	status = ir_spi_peel_write(peelir, u_xfers->len); TRACE
	return status;
}

static long
ir_spi_peel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int         retval = 0;
	struct ir_spi_peel_data  *peelir;
	struct spi_ioc_transfer *ioc;
	struct strIds *id;
	int rc = 0;

	printk("%s:Invoked. cmd = %d\n", __func__, cmd);
	peelir = filp->private_data; TRACE

	mutex_lock(&peelir->buf_lock);

	switch (cmd) {
	/* read ioctls */
	case SPI_IOC_WR_MSG:
		/* copy into scratch area */
		ioc = kmalloc(sizeof(struct spi_ioc_transfer), GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, sizeof(struct spi_ioc_transfer))) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		rc = ir_regulator_set(1);
		if (!rc) {
			retval = ir_spi_peel_write_message(peelir, ioc);
		}
		if (retval > 0)
			prev_tx_status = 1;
		else
			prev_tx_status = 0;
		ir_regulator_set(0);
		kfree(ioc);

		break;

	case SPI_IOC_RD_MSG:
		printk("%s: READ Invoked\n", __func__);
		if (is_gpio_used)
			gpio_set_value(lr_en, 1);   /* LR Enable high for Rx*/
		/* copy into scratch area */
		ioc = kmalloc(sizeof(struct spi_ioc_transfer), GFP_KERNEL);
		if (!ioc) {
			pr_err("%s: No memory for ioc. Exiting\n", __func__);
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg,
			sizeof(struct spi_ioc_transfer))) {
			pr_err("%s: Error performing copy from user of ioc\n",
					__func__);
			kfree(ioc);
			retval = -EFAULT;
			break;
		}
		printk("%s: Starting hw read\n", __func__);
		rc = ir_regulator_set(1);
		if (!rc) {
			retval = ir_spi_peel_read_message(peelir, ioc);
		}
		ir_regulator_set(0);
		if (is_gpio_used)
			gpio_set_value(lr_en, 0);   /* LR Enable default state*/
		break;
	case SPI_IOC_RD_IDS:
		id = kmalloc(sizeof(struct strIds), GFP_KERNEL);
		if (!id) {
			retval = -ENOMEM;
			break;
		}
		id->u32ID1 = 0xad1a4100;
		id->u32ID2 = 0x3c03d40;
		id->u32ID3 = 0xb5300000;

		if (__copy_to_user((void __user *)arg , id,
				sizeof(struct strIds))) {
			kfree(id);
			retval = -EFAULT;
			break;
		}
		break;
	}

	mutex_unlock(&peelir->buf_lock);

	return retval;
}

static int ir_spi_peel_open(struct inode *inode, struct file *filp)
{
	struct ir_spi_peel_data  *peelir;
	int         status = 0;

	peelir = peel_data_g; TRACE
	if (in_use) {
		dev_err(&peelir->spi->dev, "%s: Device in use. users = %d\n",
			__func__, in_use);
		return -EBUSY;
	}

	peelir->buffer = p_buf;
	if (!peelir->buffer) {
		if (!peelir->buffer) {
			dev_dbg(&peelir->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
		}
	}
	if (status == 0) {
		peelir->users++;
		filp->private_data = peel_data_g;
		nonseekable_open(inode, filp);
	}
	rcount = 0;

	return status;
}

static int ir_spi_peel_release(struct inode *inode, struct file *filp)
{
	int         status = 0;
	in_use = 0; TRACE
	peel_data_g->users = 0;
	filp->private_data = NULL;
	rcount = 0;
	return status;
}

#ifdef USES_MMAP
int ir_spi_peel_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	struct ir_spi_peel_data *peelir;
	long length = vma->vm_end - vma->vm_start;

	printk(KERN_INFO "mmap is invoked\n");
	peelir = (struct ir_spi_peel_data *)filp->private_data; TRACE
	/* check length - do not allow larger mappings than the number of
	pages allocated */
	if (length > bufsiz)
		return -EIO;

	ret = remap_pfn_range(vma, vma->vm_start,
			virt_to_phys((void *)kmalloc_area) >> PAGE_SHIFT,
			length,
			vma->vm_page_prot);
	if (ret < 0)
		return ret;
	return 0;
}
#endif
/*
 * sysfs layer
 */

static ssize_t ir_tx_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{TRACE
	return snprintf(buf, strlen(buf) + 1, "%d\n", prev_tx_status);
}

static ssize_t field_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{TRACE
	return snprintf(buf, strlen(buf) + 1, "%x\n", field);
}

static ssize_t field_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{TRACE
	sscanf(buf, "%x", &field);
	return count;
}

static DEVICE_ATTR(txstat, S_IRUGO, ir_tx_status, NULL);
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field_show, field_store);

static struct attribute *peel_attributes[] = {
	&dev_attr_txstat.attr,
	&dev_attr_field.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = peel_attributes,
};

/*-------------------------------------------------------------------------*/
static const struct file_operations peel_dev_fops = {
	.owner          =   THIS_MODULE,
	.open           =   ir_spi_peel_open,
	.release        =   ir_spi_peel_release,
	.unlocked_ioctl     =   ir_spi_peel_ioctl,
	.compat_ioctl       =   ir_spi_peel_ioctl,
#ifdef USES_MMAP
	.mmap       =   ir_spi_peel_mmap,
#endif
};

static struct miscdevice peel_dev_drv = {
    .minor          =       MISC_DYNAMIC_MINOR,
    .name           =       "ir_spi_peel",
    .fops           =       &peel_dev_fops,
    .nodename       =       "ir_spi_peel",
    .mode           =       0666
};

static int ir_spi_peel_probe(struct spi_device *spi)
{
    struct ir_spi_peel_data  *peelir;
    int         status;

    struct device_node *np = spi->dev.of_node;

    u32 bpw, mode; TRACE

    /* Allocate driver data */
    peelir = kzalloc(sizeof(*peelir), GFP_KERNEL);
    if (!peelir)
		return -ENOMEM;

    /* Initialize the driver data */
	peelir->spi = spi;
	spin_lock_init(&peelir->spi_lock);
	mutex_init(&peelir->buf_lock);
	spi_set_drvdata(spi, peelir);
	peel_data_g = peelir;
	in_use = 0;

	of_property_read_u32(np, "ir-spi-peel,spi-bpw", &bpw); TRACE
	of_property_read_u32(np, "ir-spi-peel,spi-clk-speed", &spi->max_speed_hz);
	of_property_read_u32(np, "ir-spi-peel,spi-mode", &mode);
	of_property_read_u32(np, "ir-spi-peel,lr-gpio-valid", &is_gpio_used);
	of_property_read_u32(np, "ir-spi-peel,peel-field", &field);
	of_property_read_u32(np, "ir-spi-peel,lr-gpio", &lr_en);
	of_property_read_string(np, "ir-spi_peel,reg-id", &reg_id);
	if (reg_id) {
		ir_reg = regulator_get(&(spi->dev), reg_id);
		if (IS_ERR(ir_reg)) {
			printk(KERN_ERR "ir regulator_get fail.\n");
			return PTR_ERR(ir_reg);
		}
	}
	printk("%s: lr-gpio-valid = %d\n", __func__, is_gpio_used);
	spi->bits_per_word = (u8)bpw;
	spi->mode = (u8)mode;

	printk("%s:lr_en = %d\n", __func__, lr_en);
	if (is_gpio_used) {
		if (gpio_is_valid(lr_en)) {
			/* configure LR enable gpio */
			status = gpio_request(lr_en, "lr_enable");
			if (status) {
				printk("unable to request gpio [%d]: %d\n",
					lr_en, status);
			}
			status = gpio_direction_output(lr_en, 0);
			if (status) {
				printk("unable to set direction for gpio [%d]: %d\n",
					lr_en, status);
			}
			gpio_set_value(lr_en, 0);
	} else
		printk("gpio %d is not valid \n", lr_en);
	}
	misc_register(&peel_dev_drv);
	/* sysfs entry */
	status = sysfs_create_group(&spi->dev.kobj, &attr_group);
	if (status)
		dev_dbg(&spi->dev, " Error creating sysfs entry ");

	return status;
}

static int ir_spi_peel_remove(struct spi_device *spi)
{
    struct ir_spi_peel_data  *peelir = spi_get_drvdata(spi);

    sysfs_remove_group(&spi->dev.kobj, &attr_group); TRACE

    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&peelir->spi_lock);
    peelir->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&peelir->spi_lock);

    /* prevent opening a new instance of the device
       during the removal of the device
     */
    if (peelir->users == 0) {
		kfree(peelir);
		kfree(p_buf);
    } else {
		return -EBUSY;
    }

    return 0;
}

static const struct of_device_id ir_spi_peel_of_match[] = {
    {.compatible = "ir-spi-peel"},
	{},
};
MODULE_DEVICE_TABLE(of, ir_spi_peel_of_match);

static struct spi_driver ir_spi_peel_driver = {
	.probe =    ir_spi_peel_probe,    
	.driver = {
		.name       =   IR_SPI_PEEL_DRIVER_NAME,
		.owner      =   THIS_MODULE,
		.of_match_table =   ir_spi_peel_of_match,
	},

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};
module_spi_driver(ir_spi_peel_driver);

/*-------------------------------------------------------------------------*/

// static int __init ir_spi_peel_init(void)
// {
//     int status;
//     printk("%s:npages = %u\n", __func__, npages);
//     bufsiz = npages * PAGE_SIZE;
//     if (bufsiz % PAGE_SIZE) {
// 		printk("%s:buffer size not aligned to page\n", __func__);
// 		return -EINVAL;
//     }
//     p_buf = kzalloc(bufsiz, GFP_KERNEL|GFP_ATOMIC);
//     if (p_buf == NULL)
// 		return -ENOMEM;
//     #ifdef USES_MMAP
// 	kmalloc_ptr = p_buf;
// 	kmalloc_area = (int *)((((unsigned long)kmalloc_ptr) +
// 			PAGE_SIZE - 1) & PAGE_MASK);
//     #endif
//     status = spi_register_driver(&ir_spi_peel_driver);
//     if (status < 0 || p_buf == NULL) {
// 		printk("%s: Error registerign peel driver\n", __func__);
// 		return -ENODEV;
// 	}

// 	return status;
// }
// module_init(ir_spi_peel_init);

MODULE_DESCRIPTION("Peel IR SPI driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Preetam S Reddy <preetam.reddy@peel.com>");

