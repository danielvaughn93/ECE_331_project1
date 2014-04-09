// A. Sheaff LCD RPi 2/6/13
// Allow access to LCD on the RPi
// Add a varible and then is discarded after init
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/io.h>
#include <mach/platform.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <asm/gpio.h>
//#include "gpiolcd.h"


#define LCD_MOD_AUTH "Sheaff"
#define LCD_MOD_DESCR "GPIO LCD Driver"
#define LCD_MOD_SDEV "GPIO LCD RPi"

static long lcd_ioctl(struct file * flip, unsigned int cmd, unsigned long arg);
static int lcd_open(struct inode *inode, struct file *filp);
static int lcd_release(struct inode *inode, struct file *filp);
static char *lcd_devnode(struct device *dev, umode_t *mode);
static void lcd_init(void);

static const struct file_operations lcd_fops = {
        .owner=THIS_MODULE,
        .open=lcd_open,
        .release=lcd_release,
        .unlocked_ioctl=lcd_ioctl,
};

struct lcd_data {
        int lcd_mjr;
        struct class *lcd_class;
};

static struct lcd_data lcd = {
        .lcd_mjr=0,
        .lcd_class=NULL,
};

static void lcd_init(void)
{
        printk(KERN_NOTICE "LCD initialized\n");
}

static long lcd_ioctl(struct file * flip, unsigned int cmd, unsigned long arg)
{
        return -EINVAL;
}

static int lcd_open(struct inode *inode, struct file *filp)
{
        printk(KERN_NOTICE "Open Successful\n");
        return 0;

}

static int lcd_release(struct inode *inode, struct file *filp)
{
        return 0;
}

static char *lcd_devnode(struct device *dev, umode_t *mode)
{
        if (mode) *mode = 0666;
        return NULL;
}

// Module init
static int __init rpigpio_lcd_minit(void)
{
        // So we're going to need to register a platform device
        // struct platform_device
        //bcm_register_device(&bcm2708_gpio_lcd_device);
        // request GPIO pins connected to LCD
        // Initialize the LCD
        // If we use busy flag then we need to have a timeout
        // Cannot hold processor indef.
        // Create a character device that is read and writable
        // Read will read the LCD data - not sure where it starts
        // Write will add characters to screen
        // ioctl?
        // open/close
        struct device *dev=NULL;
        int ret=0;

        printk(KERN_INFO "%s\n",LCD_MOD_DESCR);
        printk(KERN_INFO "By: %s\n",LCD_MOD_AUTH);

        lcd.lcd_mjr=register_chrdev(0,"gpio_lcd",&lcd_fops);
        if (lcd.lcd_mjr<0) {
                printk(KERN_NOTICE "Cannot register char device\n");
                return lcd.lcd_mjr;
        }
        lcd.lcd_class=class_create(THIS_MODULE, "lcd_class");
        if (IS_ERR(lcd.lcd_class)) {
                unregister_chrdev(lcd.lcd_mjr,"lcd_gpio");
                return PTR_ERR(lcd.lcd_class);
        }
        lcd.lcd_class->devnode=lcd_devnode;
        dev=device_create(lcd.lcd_class,NULL,MKDEV(lcd.lcd_mjr,0),(void *)&lcd,"lcd");
        if (IS_ERR(dev)) {
                class_destroy(lcd.lcd_class);
                unregister_chrdev(lcd.lcd_mjr,"lcd_gpio");
                return PTR_ERR(dev);
        }
        // Call lcd_init
        lcd_init();
        return ret;
}

// Module removal
static void __exit rpigpio_lcd_mcleanup(void)
{
        //bcm_unregister_device(&bcm2708_gpio_lcd_device);
        // Release LCD GPIO pins
        device_destroy(lcd.lcd_class,MKDEV(lcd.lcd_mjr,0));
        class_destroy(lcd.lcd_class);
        unregister_chrdev(lcd.lcd_mjr,"lcd_gpio");
        printk(KERN_INFO "Goodbye\n");
        return;
}

module_init(rpigpio_lcd_minit);
module_exit(rpigpio_lcd_mcleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(LCD_MOD_AUTH);
MODULE_DESCRIPTION(LCD_MOD_DESCR);