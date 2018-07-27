/* References for this driver is taken from the sample examples of zephyr.
   For EEPROM implementation we referred, /zephyr/samples/drivers/i2c_fujitsu_fram */

#include <misc/__assert.h>
#include <misc/printk.h>
#include <device.h>
#include <kernel.h>
#include <gpio.h>
#include <i2c.h>
#include <misc/util.h>
#include <zephyr.h>
#include <flash.h>
#include <board.h>
#include <zephyr/types.h>
#include <misc/byteorder.h>
#include <pinmux.h>

#define FC256_I2C_ADDR 0x50

struct fc256_data
{
	struct device *i2c;
	struct device *gpio_wp;
}fc256_data;

static int mem_read(struct device *dev, off_t offset, void *data,size_t len)
{
	struct fc256_data *mem_read_data = dev->driver_data;
	
	u8_t rd_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */
	/* FRAM address */
	rd_addr[0] = (offset >> 8) & 0xFF;
	rd_addr[1] = offset & 0xFF;

	/* Setup I2C messages */
	// Send the address to read from 
	msgs[0].buf = rd_addr;
	msgs[0].len = 2;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = len;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(mem_read_data->i2c, &msgs[0], 2, FC256_I2C_ADDR);
	return 0;
}

static int mem_write(struct device *dev, off_t offset, const void *data_h,size_t len)
{
	void *data=(void *)data_h;
	struct fc256_data *mem_write_data = dev->driver_data;
	u8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* FRAM address */
	wr_addr[0] = (offset >> 8) & 0xFF;
	wr_addr[1] = offset & 0xFF;

	/* Setup I2C messages */
	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = len;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(mem_write_data->i2c, &msgs[0], 2, FC256_I2C_ADDR);
	return 0;
}

static int mem_erase(struct device *dev, off_t offset, size_t size)
{
	u32_t data_erase[16];
	int i,ret;

	for (i = 0; i < 16; i++)
	{
		data_erase[i] = 0X00;
	}
    for(i=0;i<512;i++)
    { 
    	ret = mem_write(dev, 0x00+(i*64), data_erase,sizeof(data_erase));
    }
	return 0;
}

static int mem_write_protection(struct device *dev, bool enable)
{
	struct fc256_data *data = dev->driver_data;
	if(enable == true)
		gpio_pin_write(data->gpio_wp,3,1);
	else if(enable == false)
		gpio_pin_write(data->gpio_wp,3,0);
	return 0;
}

static const struct flash_driver_api fc256_api = {
	.read = mem_read,
	.write = mem_write,
	.erase = mem_erase,
	.write_protection = mem_write_protection
};

static int mem_init(struct device *dev)
{
	int ret=1;
	struct fc256_data *data = dev->driver_data;
	printk("In init trying to bind exp1\n");

	struct device *dev_gpio = device_get_binding("EXP1");

	if(dev_gpio==NULL)
		printk("Error in init while binding GPIO\n");

	gpio_pin_configure(dev_gpio,0,GPIO_DIR_OUT);
	ret = gpio_pin_write(dev_gpio,0,0);

	if(ret!=0)
		printk("Error in writing to gpio 32\n");
	
	gpio_pin_configure(dev_gpio,1,GPIO_DIR_OUT);
	ret = gpio_pin_write(dev_gpio,1,0);

	if(ret!=0)
		printk("Error in writing to gpio 33\n");
	
	data->gpio_wp = device_get_binding("GPIO_0");
	gpio_pin_configure(data->gpio_wp,3,GPIO_DIR_OUT);
	
	struct fc256_data *drv_data = dev->driver_data;
	printk("trying to bind fc256 i2c master from driver\n");
	drv_data->i2c = device_get_binding(CONFIG_FC256_I2C_MASTER);

	if (drv_data->i2c == NULL) {
		printk("Failed to get pointer to %s device!",CONFIG_FC256_I2C_MASTER);
		return -EINVAL;
	}
	
	return 0;

}

DEVICE_AND_API_INIT(FC256_NAME,CONFIG_FC256_DRV_NAME,mem_init,&fc256_data, NULL,POST_KERNEL, 90, &fc256_api);