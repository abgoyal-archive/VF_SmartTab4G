/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>


/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/peelirB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */

#define	PEELIR_MAJOR		176	/* assigned */
#define PEELIR_MINORS		32	/* ... up to 256 */
#define PEELIR_CHARDEV_NAME		"peelspi"
#define PEELIR_SYSCLS_NAME		"peelir"
#define PEELIR_SYSDEV_NAME		"peel_ir"


static DECLARE_BITMAP(minors, PEELIR_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

struct peelir_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;
	u8			*bufferrx;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;

	int reset_gpio;
	int slave_select_gpio;
	int config_codne_gpio;
	int vdd_en_gpio;
	int clk_en_gpio;

	struct clk *infrared_clk;
	struct regulator *infrared_iovcc;
	struct regulator *infrared_vpp;

	bool powered;
};


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 170 * 1024;  /* Default buffer size */
static int prev_tx_status; /* Status of previous transaction */

//static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*
 * This can be used for testing the controller, given the busnum and the
 * cs required to use. If those parameters are used, peelir is
 * dynamically added as device on the busnum, and messages can be sent
 * via this interface.
 */
static int busnum = 0;
module_param(busnum, int, S_IRUGO);
MODULE_PARM_DESC(busnum, "bus num of the controller");

static int chipselect = 0;
module_param(chipselect, int, S_IRUGO);
MODULE_PARM_DESC(chipselect, "chip select of the desired device");

static int maxspeed = 1520000;
module_param(maxspeed, int, S_IRUGO);
MODULE_PARM_DESC(maxspeed, "max_speed of the desired device");

static int spimode = SPI_MODE_3;
module_param(spimode, int, S_IRUGO);
MODULE_PARM_DESC(spimode, "mode of the desired device");


static ssize_t peelir_sync(struct peelir_data *peelir, struct spi_message *message);


/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void peelir_complete(void *arg)
{
	complete(arg);
}

static ssize_t
peelir_sync(struct peelir_data *peelir, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	printk("allan:%s\n",__func__);
	message->complete = peelir_complete;
	message->context = &done;

	spin_lock_irq(&peelir->spi_lock);
	if (peelir->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(peelir->spi, message);
	spin_unlock_irq(&peelir->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}


static int peelir_message(struct peelir_data *peelir,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned n, total;
	u8 *buf, *bufrx;
	int	status = -EFAULT;

	printk("allan 1:%s\n",__func__);
	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = peelir->buffer;
	bufrx = peelir->bufferrx;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}
		printk("allan 2:%s\n",__func__);
		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = bufrx;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		printk("allan 3:%s\n",__func__);
		buf += k_tmp->len;
		bufrx += k_tmp->len;
		u_tmp->speed_hz = 1520000;
		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
		printk("allan :%s, k_tmp->speed_hz = %d\n",__func__, k_tmp->speed_hz);
//#ifdef VERBOSE
		dev_err(&peelir->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : peelir->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : peelir->spi->max_speed_hz);
//#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = peelir_sync(peelir, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = peelir->bufferrx;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;

done:
	printk("allan failed:%s\n",__func__);
	kfree(k_xfers);
	return status;
}

static long
peelir_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int	err = 0;
	int	retval = 0;
	struct peelir_data *peelir;
	struct spi_device *spi;
	u32	tmp;
	unsigned n_ioc;
	struct spi_ioc_transfer	*ioc;

	printk("allan:%s\n",__func__);
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	peelir = filp->private_data;

	spin_lock_irq(&peelir->spi_lock);
	spi = spi_dev_get(peelir->spi);
	spin_unlock_irq(&peelir->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&peelir->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
              printk("[allan] --- SPI_IOC_RD_MODE = %d\n", cmd);
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
              printk("[allan] --- SPI_IOC_RD_LSB_FIRST = %d\n", cmd);
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
              printk("[allan] --- SPI_IOC_RD_BITS_PER_WORD = %d\n", cmd);
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
              printk("[allan] --- SPI_IOC_RD_MAX_SPEED_HZ = %d\n", cmd);
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
              printk("[allan] --- SPI_IOC_WR_MODE = %d\n", cmd);
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			//spi->mode = (u8)tmp;            when testing ,disable mode from APK  by allan
			spi->mode = 1;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
              printk("[allan] --- SPI_IOC_WR_LSB_FIRST = %d\n", cmd);
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
              printk("[allan] --- SPI_IOC_WR_BITS_PER_WORD = %d\n", cmd);
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
              printk("[allan] --- SPI_IOC_WR_MAX_SPEED_HZ = %d\n", cmd);
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			printk("allan:%s  max_speed_hz  = %d \n",__func__, save);
			spi->max_speed_hz = 1520000; //960000;         // when testing . disable 10K set from apk  by allan
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;

	default:
              printk("[allan] --- default = %d\n", cmd);
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		printk("[allan] --- peelir_message \n");
		retval = peelir_message(peelir, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&peelir->buf_lock);
	spi_dev_put(spi);
	return retval;
}


static int peelir_open(struct inode *inode, struct file *filp)
{
	struct peelir_data *peelir;
	int status = -ENXIO;
	//int	ret = 0;

	pr_info("%s : allan add \n", __func__);

	mutex_lock(&device_list_lock);

	list_for_each_entry(peelir, &device_list, device_entry) {
		if (peelir->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (!peelir->buffer) {
			peelir->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!peelir->buffer) {
				printk("kmalloc fail\n");
				dev_dbg(&peelir->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (!peelir->bufferrx) {
			peelir->bufferrx = kmalloc(bufsiz, GFP_KERNEL);
			if (!peelir->bufferrx) {
				dev_dbg(&peelir->spi->dev, "open/ENOMEM\n");
				kfree(peelir->buffer);
				peelir->buffer = NULL;
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			peelir->users++;
			filp->private_data = peelir;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("peelir: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);

//infrared_open_failed:
	return status;
}

static int peelir_release(struct inode *inode, struct file *filp)
{
	struct peelir_data *peelir;
	int	status = 0;

	mutex_lock(&device_list_lock);
	peelir = filp->private_data;
	filp->private_data = NULL;

	printk("allan-------powerDown L5 1.2v-------------\n");

	/* last close? */
	peelir->users--;
	if (!peelir->users) {
		int	dofree;

		kfree(peelir->buffer);
		peelir->buffer = NULL;
		kfree(peelir->bufferrx);
		peelir->bufferrx = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&peelir->spi_lock);
		dofree = (peelir->spi == NULL);
		spin_unlock_irq(&peelir->spi_lock);

		if (dofree)
			kfree(peelir);
	}
	mutex_unlock(&device_list_lock);
	return status;
}

/*
 * sysfs layer
 */

static ssize_t ir_tx_status(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
        return sprintf( buf, "%d\n", prev_tx_status );
}

static DEVICE_ATTR(txstat, S_IRUGO, ir_tx_status, NULL);

static struct attribute *tx_status_attributes = {
        &dev_attr_txstat.attr,
};

static const struct file_operations peelir_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = peelir_ioctl,
	.open =		peelir_open,
	.release =	peelir_release,
};


static struct class *peelir_class;

/*-------------------------------------------------------------------------*/



static int peelir_probe(struct spi_device *spi)
{
	struct peelir_data *peelir;
	int	status;
	unsigned long minor;

	pr_info("%s : allan add \n", __func__);

	peelir = kzalloc(sizeof(*peelir), GFP_KERNEL);
	if (!peelir)
		return -ENOMEM;

	/* Initialize the driver data */
	peelir->spi = spi;

	spin_lock_init(&peelir->spi_lock);
	mutex_init(&peelir->buf_lock);

	INIT_LIST_HEAD(&peelir->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, PEELIR_MINORS);
	if (minor < PEELIR_MINORS) {
		struct device *dev;

		peelir->devt = MKDEV(PEELIR_MAJOR, minor);
		dev = device_create(peelir_class, &spi->dev, peelir->devt,
					peelir, PEELIR_SYSDEV_NAME);

		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;

	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&peelir->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	peelir->spi->max_speed_hz = 1520000;

	if (status == 0)
		spi_set_drvdata(spi, peelir);
	else
		kfree(peelir);

	/* sysfs entry */
	status = sysfs_create_file(&spi->dev.kobj, tx_status_attributes);
	if( status )
		dev_dbg(&spi->dev, " Error creating sysfs entry " );

	return status;
}

static int  peelir_remove(struct spi_device *spi)
{
	struct peelir_data *peelir = spi_get_drvdata(spi);
	pr_info("%s : allan add \n", __func__);
	sysfs_remove_file(&spi->dev.kobj, tx_status_attributes);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&peelir->spi_lock);
	peelir->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&peelir->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&peelir->device_entry);
	device_destroy(peelir_class, peelir->devt);
	clear_bit(MINOR(peelir->devt), minors);
	if (peelir->users == 0)
		kfree(peelir);
	mutex_unlock(&device_list_lock);

	return 0;
}

static int  peelir_suspend(struct spi_device *spi, pm_message_t mesg)
{
	//struct peelir_data *peelir = spi_get_drvdata(spi);

	/* prevent new opens */
	mutex_lock(&device_list_lock);

	mutex_unlock(&device_list_lock);

	return 0;
}

static int  peelir_resume(struct spi_device *spi)
{
	//struct peelir_data *peelir = spi_get_drvdata(spi);

	/* prevent new opens */
	mutex_lock(&device_list_lock);

	mutex_unlock(&device_list_lock);

	return 0;
}


static struct of_device_id peelir_of_match_table[] = {
	{ .compatible = "infrared,ice40-spi-infrared", },
	{},
};

static struct spi_driver peelir_spi_driver = {
	.driver = {
		.name =		"ice40_spi_infrared",
		.owner =	THIS_MODULE,
		.of_match_table = peelir_of_match_table,
	},
	.probe =	peelir_probe,
	.remove =	peelir_remove,
	.suspend =  peelir_suspend,
	.resume =   peelir_resume,
};


/*-------------------------------------------------------------------------*/
static int __init peelir_init(void)
{
	int status;

	printk(KERN_ERR "allan:%s\n",__func__);
	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(PEELIR_MINORS > 256);
	status = register_chrdev(PEELIR_MAJOR, PEELIR_CHARDEV_NAME, &peelir_fops);
	if (status < 0)
		return status;

	peelir_class = class_create(THIS_MODULE, PEELIR_SYSCLS_NAME);
	if (IS_ERR(peelir_class)) {
		status = PTR_ERR(peelir_class);
		goto error_class;
	}

	return 0;

error_class:
	pr_info("%s : class_create failed \n", __func__);
	unregister_chrdev(PEELIR_MAJOR, peelir_spi_driver.driver.name);
	return status;
}
core_initcall(peelir_init);

static void __exit peelir_exit(void)
{
	spi_unregister_driver(&peelir_spi_driver);
	class_destroy(peelir_class);
	unregister_chrdev(PEELIR_MAJOR, peelir_spi_driver.driver.name);
}
module_exit(peelir_exit);

module_spi_driver(peelir_spi_driver);


MODULE_AUTHOR("Allan, <pingao.yang@tcl.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spi_peel_infrared");


