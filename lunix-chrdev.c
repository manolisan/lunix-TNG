/*
* lunix-chrdev.c
*
* Implementation of character devices
* for Lunix:TNG
*
* Stratis Tsirtsis < stsirtsis@gmail.com >
* Manolis Anastasiou < manolisan28@gmail.com >
*/

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
* Global data
*/
struct cdev lunix_chrdev_cdev;

/*
* Just a quick [unlocked] check to see if the cached
* chrdev state needs to be updated from sensor measurements.
*/
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint32_t timestamp;

	WARN_ON ( !(sensor = state->sensor));

	timestamp = sensor->msr_data[state->type]->last_update;
	if (timestamp != state->buf_timestamp){
		// state needs refresh
		return 1;
	} else {
		// no need for refresh
		return 0;
	}
}

/*
* Updates the cached state of a character device
* based on sensor data. Must be called with the
* character device state lock held.
*/
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint32_t value, timestamp;
	long formated_data;

	sensor = state->sensor;
	WARN_ON(!sensor);

	debug("leaving\n");

	/*
	* Any new data available?
	*/
	printk("DEBUG: Timestamp: %d, Private sate stamp: %d \n", timestamp, state->buf_timestamp);
	if (lunix_chrdev_state_needs_refresh(state) == 0){
		// no change
		return -EAGAIN;
	} else {

		/*
		* Grab the raw data quickly, hold the
		* spinlock for as little as possible.
		*/
		spin_lock(&sensor->lock);
		timestamp = sensor->msr_data[state->type]->last_update;
		value = sensor->msr_data[state->type]->values[0];
		/* Why use spinlocks? See LDD3, p. 119 */
		spin_unlock(&sensor->lock);

		/*
		* Now we can take our time to format them,
		* holding only the private state semaphore
		*/
		printk("UPD_DEBUG: Index value: %d !", value);
		if (state->type == 0){
			formated_data = lookup_voltage[value];
		} else if (state->type == 1){
			formated_data = lookup_temperature[value];
		} else if (state->type == 2){
			formated_data = lookup_light[value];
		} else {
			//error
			return -EFAULT;
		}
		printk("UPD_DEBUG: Formated_data: %d !", formated_data);

		// Convert formated data to a sequence of char bytes
		long digit, restnum = formated_data;
		int old_buf_lim = state->buf_lim;
		int num_of_digits = 0;
		char chdigit;
		do {
			digit = restnum%10;
			restnum = restnum/10;
			chdigit = digit + '0';
			state->buf_data[state->buf_lim] = chdigit;
			state->buf_lim = (state->buf_lim+1) % 20;
			num_of_digits++;
			printk("UPD_DEBUG: Data character: %c\n", chdigit);
		} while(restnum != 0);
		int left = old_buf_lim, right = old_buf_lim + num_of_digits - 1;
		char a;
		while (left <= right){
			a = state->buf_data[left%20];
			state->buf_data[left%20] = state->buf_data[right%20];
			state->buf_data[right%20] = a;
			left++;
			right--;
		}
		right = state->buf_lim+20;
		state->buf_data[right%20] = state->buf_data[(right-1)%20];
		state->buf_data[(right-1)%20] = state->buf_data[(right-2)%20];
		state->buf_data[(right-2)%20] = state->buf_data[(right-3)%20];
		state->buf_data[(right-3)%20] = '.';
		state->buf_lim = (state->buf_lim+1) % 20;
		state->buf_data[state->buf_lim] = '\n';
		state->buf_lim = (state->buf_lim+1) % 20;
		state->buf_timestamp = timestamp;
	}

	debug("leaving\n");
	return 0;
}

/*************************************
* Implementation of file operations
* for the Lunix character device
*************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	struct lunix_chrdev_state_struct* private_ptr;
	unsigned int minor_num;
	/* ? */
	int ret;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
	goto out;

	// find minor node of the device, to be open
	minor_num = iminor(inode);

	// alocate space for lunix sensor state struct
	private_ptr = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if(!private_ptr){
		debug("failed to allocate memory for private data (lunix_chrdev_state_struct)\n");
		goto out;
	}

	// type of the device: Batt, light, temp
	private_ptr->type = minor_num % 8;
	if (private_ptr->type > 2 || private_ptr->type < 0) {
		debug("Not proper device measurement. The number is %d.\n", private_ptr->type);
		goto out;
	}

	// initialize buf lim,timestamp and semaphore
	private_ptr->buf_lim = 0;
	private_ptr->buf_timestamp = 0;
	sema_init(&(private_ptr->lock), 1);

	// connection with the proper device. 1-16
	private_ptr->sensor = &lunix_sensors[minor_num / 8];

	//save lunix sensor state to file private data field
	filp->private_data = private_ptr;

	out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	WARN_ON(!filp->private_data);
	kfree(filp->private_data);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;
	long result;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	printk("DEBUG: We are starting the read call!\n");

	/* Lock */

	if(down_interruptible(&(state->lock))){
		return -ERESTARTSYS;
	}

	printk("DEBUG: Initial f_pos: %d buf_lim: %d\n", *f_pos, state->buf_lim);

	while (lunix_chrdev_state_update(state) == -EAGAIN && state->buf_lim == *f_pos) {
		printk("DEBUG: Let's sleep, no data available\n");
		up(&state->lock);
		if(wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state) && state->buf_lim == *f_pos) ){
			return -ERESTARTSYS;
		}

		printk("DEBUG: I've just wake up!\n");

		if(down_interruptible(&state->lock)){
			return -ERESTARTSYS;
		}
	}

	/* Determine the number of cached bytes to copy to userspace */
	if (state->buf_lim > *f_pos){
		ret = cnt < state->buf_lim - *f_pos ? cnt : state->buf_lim - *f_pos;
	}
	else {
		ret = cnt < 20 - *f_pos ? cnt : 20 - *f_pos;
	}

	printk("DEBUG: Calculated count: %d\n", ret);

	// copy to usr space
	printk("DEBUG: About to copy to user.\n");
	int i;
	for(i=0; i<ret; i++){
		printk("DEBUG: Data byte %d: %c\n", i, state->buf_data[*f_pos + i]);
	}

	if (copy_to_user(usrbuf, &state->buf_data[*f_pos], ret)){
		up(&state->lock);
		return -EFAULT;
	}

	*f_pos += ret;
	printk("DEBUG: Changed f_pos. New value: %d\n", *f_pos);

	/* Auto-rewind on EOF mode? */
	if(*f_pos == 20){
		*f_pos = 0;
	}
	printk("DEBUG: Rewinded f_pos. Final f_pos value: %d Final buf_lim value: %d\n", *f_pos, state->buf_lim);
	out:
	/* Unlock? */
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops =
{
	.owner    = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	* Register the character device with the kernel, asking for
	* a range of minor numbers (number of sensors * 8 measurements / sensor)
	* beginning with LINUX_CHRDEV_MAJOR:0
	*/
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix-TNG");
	/* register_chrdev_region? */
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	/* ? */
	ret = cdev_add(&lunix_chrdev_cdev ,dev_no ,lunix_minor_cnt);
	/* cdev_add? */
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

	out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
