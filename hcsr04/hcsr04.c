/*
References for this driver is taken from the sample examples of zephyr.
For HCSR implementation we referred, /zephyr/drivers/sensors/tmp007 */

#include <misc/byteorder.h>
#include <misc/__assert.h>
#include <asm_inline_gcc.h>
#include <device.h>
#include <zephyr/types.h>
#include <stdlib.h>
#include <kernel.h>
#include <string.h>
#include <board.h>
#include <sensor.h>
#include <gpio.h>
#include <errno.h>
#include "hcsr04.h"

struct device *dev6;
struct sensor_value result;

long long start=0,stop=0,diff=0;
int begin,end,difference;

void handler(struct device *dev6, struct gpio_callback *callb, u32_t pin)
{	
	long long tmp = _tsc_read();
	int value;  
	int ready=0;
	gpio_pin_read(dev6,5,&value);	
	if(value==1)
	{
		start = tmp;
		gpio_pin_configure(dev6,5,(GPIO_DIR_IN|GPIO_INT|GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW));
	}
	else if(value == 0)
	{
		stop = tmp;
		gpio_pin_configure(dev6,5,(GPIO_DIR_IN|GPIO_INT|GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH));
		ready=1;
	}
	if(ready==1)
	{
		diff = stop-start;
		result.val1=(diff/(400*58));	
	}
}


static int hcsr_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	int flag;
	flag=gpio_pin_write(dev6,4,1); 
	if(flag<0)
		printk("Error in gpio_pin_write 3");

	k_busy_wait(75);
	flag=gpio_pin_write(dev6,4,0); 
	if(flag<0)
		printk("Error in gpio_pin_write 3");
	k_busy_wait(500);
	
	return 0;
}

static int hcsr_channel_get(struct device *dev,enum sensor_channel chan,struct sensor_value *val)
{
	end=k_cycle_get_32();
	val->val1 = result.val1;
	val->val2=end;
	return 0;
}

static const struct sensor_driver_api hcsr_driver_api = {
	.sample_fetch = hcsr_sample_fetch,
	.channel_get = hcsr_channel_get,
};

static int hcsr_init(struct device *dev){
	
	printk("In the init....0\n");
	struct hcsr_data *drv_data = dev->driver_data;
		
	dev6 = device_get_binding("GPIO_0");

	drv_data->function = handler;	
	dev->driver_api = &hcsr_driver_api;
	return 0;
}

struct hcsr_data hcsr_driver_0, hcsr_driver_1;

DEVICE_INIT(HCSR0, CONFIG_HCSR_NAME_0, hcsr_init,
		    &hcsr_driver_0, NULL,
		    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);

DEVICE_INIT(HCSR1, CONFIG_HCSR_NAME, hcsr_init,
		    &hcsr_driver_1, NULL,
		    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);