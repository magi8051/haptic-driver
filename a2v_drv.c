#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/bitops.h>
#include <linux/capability.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/in.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/inetdevice.h>
#include <linux/ip.h>
#include <linux/kthread.h>

#include <net/arp.h>
#include <net/ip.h>
#include <net/route.h>
#include <net/ip_fib.h>

#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/ioctl.h>

#include <a2v_drv.h>

//#define A2V_RTP

#ifdef A2V_RTP
#define A2V_OUT_SIZE 512
#else
#define A2V_OUT_SIZE 3
#endif

int dw791x_a2v_debug_mask = 1;
module_param_named(dw791x_a2v_debug_mask, dw791x_a2v_debug_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);

/* ============== global val ============== */
static int a2v_power=1;
static struct kobject *android_tuningA2V_kobj;
/* ======================================== */
#define DW791X_MODULE_NAME "dw791x-a2v"

int a2v_open(struct inode *inode, struct file *filp)
{
    int ret = -ENODEV;

    if(!try_module_get(THIS_MODULE)) {
        goto error;
    }

    ret = 0;

error:
    return ret;
}


int a2v_release(struct inode *inode, struct file *filp)
{
    module_put(THIS_MODULE);
    return 0;
}


ssize_t a2v_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
#ifdef A2V_RTP
    u8 buffer[A2V_OUT_SIZE] = {0,};

    if (copy_from_user(&buffer, buf, A2V_OUT_SIZE)) {
        a2v_err("copy_from_user failed\n");
        return 0;
    }
                
    a2v_debug("count:%d\n", (int)count);

    dw791x_a2v_rtp_write( (u8*)buffer, A2V_OUT_SIZE);
#else
    unsigned int buffer[A2V_OUT_SIZE] = {0,};

    if (copy_from_user(buffer, buf, sizeof(buffer))) {
        a2v_err("copy_from_user failed\n");
        return 0;
    }

    a2v_debug("count:%d buffer[0]=%d, buffer[1]=%d, buffer[2]=%d\n", (int)count,buffer[0],buffer[1],buffer[2]);

	dw791x_a2v_seq_write(buffer[0],buffer[1],buffer[2]);
#endif
    return 0;
}

#if 0
ssize_t a2v_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	a2v_debug("a2v_unlocked_ioctl\n");

	switch (cmd)
	{
		case A2V_CHANGEMODE:
			if(copy_from_user((void *)&a2v_mode, (const void __user *)arg, sizeof(long)) != 0)
				a2v_err("a2v mode change err.\n");
			a2v_debug("a2v mode chage %d\n",a2v_mode);
			break;
		default :
			break;
	}
    return 0;
}
#endif

ssize_t a2v_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    char data;
    
    data = dw791x_byte_read_ex(0x00);
    
    if (copy_to_user(buf, &data, 1)) {
        a2v_err("copy_to_user failed\n");
        return 0;
    }
    
    return count;

}

static ssize_t a2v_power_set (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef A2V_TOUCH
		int ret;
		struct siginfo info;
		struct task_struct *t;
#endif
		if ( buf[0] == '0') {
			a2v_power = 0;
		}
		else if ( buf[0] == '1') {
			a2v_power = 1;
		}
		else if ( buf[0] == '2') {
			a2v_power = 2;
		}
	
		a2v_debug("dw791x-a2v a2v_power_set %02x %02x %d\n", buf[0],buf[1],a2v_power);

    return count;
}

static ssize_t a2v_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, 256, "%i\n", a2v_power);
}

static struct device_attribute dw791x_a2v_attrs[] = {
	__ATTR(a2v_power, 0755, a2v_power_show, a2v_power_set),
};
static int a2v_sysfs_init(void)
{
    int ret = 0;
	int i = 0;
    android_tuningA2V_kobj = kobject_create_and_add("dongwoon_a2v_drv", NULL);
    
    if (android_tuningA2V_kobj == NULL) {
        a2v_debug("subsystem_register_failed\n");
    }

	for(i=0;i<ARRAY_SIZE(dw791x_a2v_attrs);i++) {
		ret = sysfs_create_file(android_tuningA2V_kobj, &dw791x_a2v_attrs[i].attr);
		if(ret < 0) {
			a2v_debug("vibrator sysfs create failed\n");
			return ret;
		}
	}
    return 0;
}
static void a2v_remove(void)
{
	int i = 0;

    for(i=0;i<ARRAY_SIZE(dw791x_a2v_attrs);i++) {
		sysfs_remove_file(android_tuningA2V_kobj, &dw791x_a2v_attrs[i].attr);
	}
    kobject_del(android_tuningA2V_kobj);
}

struct file_operations a2v_fops = {
  .owner    = THIS_MODULE,
  .open     = a2v_open,
  .release  = a2v_release,
  .write = a2v_write,
  .read = a2v_read,
#if 0 
  .unlocked_ioctl = a2v_unlocked_ioctl,
#endif
};

struct miscdevice a2v_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
	.name =	DW791X_MODULE_NAME,
	.fops = &a2v_fops,
};

static int __init a2v_init(void)
{
    int ret = 0;

    pr_err("%s: enter\n", __func__);

    ret = misc_register(&a2v_miscdev);

    if( ret < 0 ) {
        a2v_err("misc driver register error\n");
        return ret;
    }
    else
        a2v_debug("init done\n");

    a2v_sysfs_init();
    return 0;
}

static void __exit a2v_exit(void)
{    
    a2v_debug("remove.\n");
    a2v_remove();
    misc_deregister(&a2v_miscdev);
}

module_init(a2v_init);
module_exit(a2v_exit);

MODULE_DESCRIPTION("Analog to Vibrator Interface Driver");
MODULE_AUTHOR("jks8051@dwanatech.com");
//MODULE_LICENSE("GPL/BSD Dual")
