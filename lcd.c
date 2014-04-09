// A. Sheaff and D. Vaughn LCD RPi 4/9/14
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
#include <linux/delay.h>

//#include "gpiolcd.h"


#define LCD_MOD_AUTH "D Vaughn and Scheaf"
#define LCD_MOD_DESCR "GPIO LCD Driver"
#define LCD_MOD_SDEV "GPIO LCD RPi"

//  Pin ID     Rpi GPIO number     
#define RS      25           
#define RW      4            
#define E       24          
#define DB4     23          
#define DB5     17          
#define DB6     27          
#define DB7     22          

// Structures to hold the pins that were successfully allocated.
typedef struct PinSet {
        int pin;                // Linux GPIO pin number
        char *name;             // Name of the pin, supplied to gpio_request()
        int result;             // set to zero on successfully obtaining pin
} tPinSet;


static tPinSet pins[] = {
        {E, "RPI_E", -1},
        {RW, "RPI_RW", -1},
        {RS, "RPI_RS", -1},
        {DB4, "RPI_DB4", -1},
        {DB5, "RPI_DB5", -1},
        {DB6, "RPI_DB6", -1},
        {DB7, "RPI_DB7", -1},
};

static spinlock_t my_lock;

#define GPIO_IOCTL_MAG 'k'

//in: pin to read       //out: value                    //the value read on the pin
#define LCD_READ                        _IOWR(GPIO_IOCTL_MAG, 0x90, int)
#define LCD_WRITE                       _IOW(GPIO_IOCTL_MAG, 0x91, int)
#define NUM_PINS (sizeof(pins)/sizeof(tPinSet))
// Macros for setting control lines
#define RS_LOW  gpio_set_value(RS, 0);
#define RS_HIGH gpio_set_value(RS, 1);
#define E_LOW   gpio_set_value(E, 0);
#define E_HIGH  gpio_set_value(E, 1);


static long lcd_ioctl(struct file * flip, unsigned int cmd, unsigned long arg);
static int lcd_open(struct inode *inode, struct file *filp);
static int lcd_release(struct inode *inode, struct file *filp);
static char *lcd_devnode(struct device *dev, umode_t *mode);
static void lcd_init(void);
static void awaken(unsigned int val);
static void PutData(char c);
static ssize_t lcd_write(struct file *file, const char *buf, size_t count, loff_t * ppos);
static void PinReturn(void);
static void PutCom(char c);


static const struct file_operations lcd_fops = {
        .owner = THIS_MODULE,
        .open = lcd_open,
        .release = lcd_release,
        .write = lcd_write,
        .unlocked_ioctl = lcd_ioctl,
};

struct lcd_data {
        int lcd_mjr;
        struct class *lcd_class;
};

static struct lcd_data lcd = {
        .lcd_mjr=0,
        .lcd_class=NULL,
};

static long lcd_ioctl(struct file * flip, unsigned int cmd, unsigned long arg)
{
        int data;
        int lock=1;

        // Read the request data
        if (copy_from_user(&data, (int *) arg, sizeof(data))) {
                printk(KERN_INFO "copy_from_user error on LCD ioctl.\n");
                return -EFAULT;
        }

        switch (cmd) {
                case LCD_READ:

                case LCD_WRITE:
                        lock=spin_trylock(&my_lock);
                        if(!lock){
                                printk(KERN_INFO "Unable to obtain lock");
                                return -EACCES; //Denys permission
                        }
                        else{
                                printk(KERN_INFO "Got the lock!");
                                //lcd_write(flip, &data, size_t count, loff_t * ppos);
                                return 0;
                        }
        default:
                printk(KERN_INFO "Invalid ioctl on LCD\n");
                return -EINVAL;
        }

        // return the result
        if (copy_to_user((int *) arg, &data, 4)) {
                printk(KERN_INFO "copy_to_user error on LCD ioctl\n");
                return -EFAULT;
        }

        return 0;
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
        PinReturn();
        printk(KERN_INFO "Goodbye\n");
        return;
}

//nibble function
static void awaken(unsigned int val)
{
        gpio_set_value(DB4, (val & 0x1));
        gpio_set_value(DB5, (val & 0x2) >> 1);
        gpio_set_value(DB6, (val & 0x4) >> 2);
        gpio_set_value(DB7, (val & 0x8) >> 3);
        udelay(1);
        E_LOW;
        udelay(1);              // data setup time
        E_HIGH;
        udelay(1);              // data hold time
}
static void PutData(char c)
{
        udelay(1);
        RS_HIGH;
        udelay(1);
        awaken((c >> 4) & 0xf);
        awaken(c & 0xf);
        udelay(50);
}

// Called when writing to the device file.
static ssize_t lcd_write(struct file *file, const char *buf, size_t count, loff_t * ppos)
{
        int err;
        char c;
        const char *ptr = buf;
        int i;
        spin_lock(&my_lock);
        for (i = 0; i < count; i++) {
                err = copy_from_user(&c, ptr++, 1);
                if (err != 0)
                        return -EFAULT;
              //  put char on screen
                PutData(c);
        }
        spin_unlock(&my_lock);
        return count;
}

// Return any acquired pins.
static void PinReturn(void)
{
        int i;
        for (i = 0; i < NUM_PINS; i++) {
                if (pins[i].result == 0) {
                        gpio_free(pins[i].pin);
                        pins[i].result = -1;    //avoid multiple free.
                }
        }
}

// Send command code to the display
static void PutCom(char c)
{
        udelay(1);
        RS_LOW;
        udelay(1);
        awaken((c >> 4) & 0xf);
        awaken(c & 0xf);
        udelay(50);
}

static void lcd_init(void)
{
        int i;
        int got_pins = 1;

        // Request pins
        for (i = 0; i < NUM_PINS; i++) {
                pins[i].result = gpio_request(pins[i].pin, pins[i].name);
                if (pins[i].result != 0)
                        got_pins = 0;
        }

        // On any failures, return any pins we managed to get and quit.
        if (!got_pins) {
                PinReturn();
                return;
        }
        // Set port direction.  We assume we can do this if we got the pins.
        // Set initial values to low (0v).
        for (i = 0; i < NUM_PINS; i++) {
                gpio_direction_output(pins[i].pin, 0);
        }

      // Power on and setup the display
        awaken(0x03);
        msleep(1);
        awaken(0x03);
        msleep(1);
        awaken(0x03);
        msleep(1);
        awaken(0x02);
        msleep(1);

        PutCom(0x28);
        udelay(50);
        PutCom(0x0c);
        udelay(50);
        PutCom(0x01);
        udelay(50);
        PutCom(0x06);
        udelay(50);

}


module_init(rpigpio_lcd_minit);
module_exit(rpigpio_lcd_mcleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(LCD_MOD_AUTH);
MODULE_DESCRIPTION(LCD_MOD_DESCR);