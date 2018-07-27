
/*

References for this program is taken from the sample examples of zephyr.

For implementation of gpio callback function we referred: /zephyr/samples/basic/button
For implementation of shell module we referred: /zephyr/samples/subsys/shell/shell
For implementation for gpio configrations we referred: /zephyr/samples/basic/blinky
For additional support of mutex,threads, pin multiplexing and pwm we referred the online documentation for galileo
For EEPROM implementation we referred, /zephyr/samples/drivers/i2c_fujitsu_fram */

#include <gpio.h>
#include <board.h>
#include <misc/util.h>
#include <misc/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <device.h>
#include <zephyr.h>
#include <shell/shell.h>
#include <pinmux.h>
#include <flash.h>
#include <kernel.h>
#include <sensor.h>

/* size of stack area used by each thread */
#define STACKSIZE 500
#define EDGE (GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)  // Both edge confuguration macros

/* scheduling priority used by each thread */
#define PRIORITY 5   // priority of thread reading values from sensor
#define PRIORITY1 7  // priority of thread writing to EEPROM

#define MUX CONFIG_PINMUX_NAME
#define MY_SHELL_MODULE "my_shell"
struct k_sem my_sem; // declare semaphore

static struct gpio_callback callb;  // callback object 
struct sensor_value temp_value;

//structure used for data passing by sensor
struct hcsr_data 
{
	void (*function)();
};

// pointer to device structure
struct device *temp_dev;
struct device *dev6;
struct device *pinmux;

void my_config(int); 		// function to select sensor
void erase_mydata(); 		// function to erase all EEPROM values
void write_mydata(int); 	// function to write valus into EEPROM
void read_mydata(int, int); // function to read from EEPROM

int p,p1,p2; 
struct device *dev,*fc_dev;
int ret,i,j,num;

// to store data into buffere  to reead and write into EEPROM
u32_t cmp_data[16]; 
u32_t data[16];
u32_t begin;

//defining the stack area for threads along side its STACKSIZE
K_THREAD_STACK_DEFINE(thread_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(thread_stack_area1, STACKSIZE);
struct k_thread thread_data, thread_data1;



/*Fucntion to read distance and time stamp from sensor and store it in a buffer to be written to EEPROM. This thread has higher priority than thread which
will write values into EEPROM. When we enter 'start p' as shell command, this function  will be executed after erasing all values in EEPROM. To synchronize 
threads such that thread 2 ececutes after thread 1 comletes reading all values, we have used semaaphore. */
/* Since our page size is 64 bytes and each distance measurement and timestamp is 32 bits each (4 bytes each), hence for one measurement it takes 
8 bytes so in a page of 64 bytes we can enter 8 measurements. So in this loop in one iteration we store first the distance and then the timestamp.
Hence buffer has 16 values and loop is increment in steps of 2 everytime. 
sensor_sample_Ftech will trigger sensor, and sensor_channel_get will get the distance value from sensor
We are taking timestamp before triggering the sensor and then again after we get the distance. The difference is our final timestamp value */
void thread1(void* a ,void* b,void* c)
{
	int i;
	int r;

	for(i=0;i<p;i++) 
	{
		for(j=0;j<16;j=j+2)
		{	
			begin=k_cycle_get_32();
			r = sensor_sample_fetch(temp_dev);
			if (r) {
				printf("sensor_sample_fetch failed return: %d\n", r);
				break;
		}
		r = sensor_channel_get(temp_dev, SENSOR_CHAN_ALL,&temp_value);
		if (r) {
			printf("sensor_channel_get failed return: %d\n", r);
			break;
		}
		
		cmp_data[j] = temp_value.val1;
		cmp_data[j+1] = temp_value.val2-begin;
		}

		for(j=0;j<16;j=j+2)
		{
			data[j] = cmp_data[j];
			data[j+1]= cmp_data[j+1];
		}	
		  k_sem_give(&my_sem);
	}
	printk("\n EXIT READ FROM SENSOR THREAD");
}	


/*Fucntion to write to EEPROM. This thread has lower priority than thread1. To write any data set write protection as 0 which is at IO0. 
  This will take semaphore posted by thread 1 and hence will be executed after thread 1 only.*/
void thread2(void* a ,void* b,void* c)
{
	int j;

	for(j=0;j<p;j++)
	{
		ret=flash_write_protection_set(fc_dev, false);
		if(ret<0)
		printk("\nError in write protection");
		k_sem_take(&my_sem, K_FOREVER);

		/* write them to the FRAM */
		ret = flash_write(fc_dev, 0x00+(j*64), data,sizeof(data));
		if (ret) 
		{
			printk("Error writing to FC256! error code (%d)\n", ret);
		} 
	}	

	printk("\n EXIT WRITE TO EEPROM THREAD");
}	


// shell module command function to select sensor. This function will be selected after enable shell command
static int shell_cmd_params_0(int argc, char *argv[])            
{                                             
    p=atoi(argv[1]);
    printk("\n\nSENSOR SELECTION: %d\n", p);
    my_config(p);
    return 0;
}

// shell module command function to erase and write data. This function will be selected after start shell command
static int shell_cmd_params_1(int argc, char *argv[])             
{                                             
    p=atoi(argv[1]);
    printk("\n\nNumber : %d\n", p);
    erase_mydata();
    return 0;
}

// shell module command function to read data. This function will be selected after dump shell command
static int shell_cmd_params_2(int argc, char *argv[])           
{                                              
    p1=atoi(argv[1]);
    p2=atoi(argv[2]);
    read_mydata(p1,p2);
    return 0;
}

// registers the shell module commands
static struct shell_cmd commands[] = {  
    { "enable", shell_cmd_params_0, "enable argc" },   
    { "start", shell_cmd_params_1, "start argc" },
    { "dump", shell_cmd_params_2, "dump argc" },
    { NULL, NULL, NULL }
};                                                                             

// init fucntion for shell
static void myshell_init(void)                                                
{
    printk("\n REGISTERED SHELL, ENTER enable (0/1/2), start (page), dump (page1,page2) \n"); 
    SHELL_REGISTER(MY_SHELL_MODULE, commands);
}


/*Function to select sensor and configure trigger and echo pins accordingly
configure for sensor 1. TRIGGER is at IO1 and ECHO is at IO2
configure for sensor 2. TRIGGER is at IO7 and ECHO is at IO5 */

void my_config(int n)
{
	int flag;
	pinmux = device_get_binding(CONFIG_PINMUX_NAME);
	if(n==0)
	{
		printk("\n NO SENSOR SELECTED FOR MEASUREMENT");
	}	

	else if(n==1)
	{
		printk("\n SENSOR 1 selected, TRIGGER is at IO1 and ECHO is at IO2");
		temp_dev = device_get_binding("HCSR0");
		if (!temp_dev) {
			printf("error: no temp device\n");
			return;
		}

		flag = pinmux_pin_set(pinmux,1,PINMUX_FUNC_A);
		if(flag!=0)
			printk("pinmux 1 error\n");

		flag = pinmux_pin_set(pinmux,2,PINMUX_FUNC_B);
		if(flag!=0)
			printk("pinmux 2 error\n");
	
		dev6 = device_get_binding("GPIO_0");
		gpio_pin_configure(dev6,5,(GPIO_DIR_IN|GPIO_INT|EDGE));
		
		struct hcsr_data *data = temp_dev->driver_data;
		gpio_init_callback(&callb,data->function, BIT(5));// intializing callback on the IO0
		gpio_add_callback(dev6, &callb);	//binding call back with device pointer
		gpio_pin_enable_callback(dev6,5);
	
	}

	else if(n==2)
	{
		printk("\nSENSOR 1 selected, TRIGGER is at IO7 and ECHO is at IO5");
		temp_dev = device_get_binding("HCSR1");
		if (!temp_dev) {
			printf("error: no temp device\n");
			return;
		}

		flag = pinmux_pin_set(pinmux,1,PINMUX_FUNC_A);
		if(flag!=0)
			printk("pinmux 1 error\n");

		flag = pinmux_pin_set(pinmux,2,PINMUX_FUNC_B);
		if(flag!=0)
			printk("pinmux 2 error\n");
			
		dev6 = device_get_binding("GPIO_0");
		gpio_pin_configure(dev6,5,(GPIO_DIR_IN|GPIO_INT|EDGE));
		
		struct hcsr_data *data = temp_dev->driver_data;
		gpio_init_callback(&callb,data->function, BIT(5));// intializing callback on the IO0
		gpio_add_callback(dev6, &callb);	//binding call back with device pointer
		gpio_pin_enable_callback(dev6,5);
	}		
	else
	{
		printk("WRONG ENABLE INPUT, TRY AGAIN");
	}	
}

/* To clear all data we are writing 0X00 in all pages as according to data sheet other option was to give VCC more that 5v. 
set write prrotection as 0 to write into EEPROM. Write protection bit is IO0. */

void erase_mydata()
{
	printk("\n\nErase data");

	k_sleep(100);
	
	ret=flash_write_protection_set(fc_dev, false);
	if(ret<0)
	printk("\nError in write protection");
	k_sleep(100);

	ret = flash_erase(fc_dev,  0x00,sizeof(data));

	if (ret) 
	{
		printk("Error erasing, error code (%d)\n", ret);
	} 

	else 
	{
		printk("Erased all pages\n");
	}

	k_tid_t thread_tid = k_thread_create(&thread_data, thread_stack_area,
                                 K_THREAD_STACK_SIZEOF(thread_stack_area),
                                 thread1,
                                 NULL, NULL, NULL,
                                 PRIORITY1, 0, K_NO_WAIT);
	printk("\n thread ID %d",(int)thread_tid); 

	k_tid_t thread_tid1 = k_thread_create(&thread_data1, thread_stack_area1,
                                 K_THREAD_STACK_SIZEOF(thread_stack_area1),
                                 thread2,
                                 NULL, NULL, NULL,
                                 PRIORITY, 0, K_NO_WAIT);
 	printk("\n thread ID %d",(int)thread_tid1); 
}
  

/* To read all data . set write prrotection as 1to read from EEPROM. Write protection bit is IO0. */
void read_mydata(int a1,int a2)
{
 for(j=a1-1;j<a2;j++)
       {
	        ret=flash_write_protection_set(fc_dev, true);
			if(ret<0)
				printk("\nError in write protection");
			k_sleep(500);
	        
			ret = flash_read(fc_dev, 0x00+(j*64),data,sizeof(data));
			if (ret) 
			{
				printk("Error reading, error code (%d)\n", ret);
			} 
			else 
			{
				printk("\n Read 64 bytes from page %d \n",j+1);
			}
			for (i = 0; i < 16; i=i+2) {
				printk("\n %d  %d",data[i],data[i+1]);
		    }
		}
}

int main()
{
	k_sem_init(&my_sem, 0, 1);
	printk("in main\n");
	pinmux = device_get_binding(MUX);
    if(pinmux==NULL)
     	printk("Error in pinmux\n");
	
	pinmux_pin_set(pinmux, 18, PINMUX_FUNC_C);
	pinmux_pin_set(pinmux, 19, PINMUX_FUNC_C);

	printk("exp2 bind\n");
	dev = device_get_binding("EXP2");

	if(dev==NULL)
     	printk("Error in EXP2\n");
	ret=gpio_pin_write(dev,9,1);

    if(ret<0)
        printk("\nError in pin write of EXP2");
    ret=gpio_pin_write(dev,11,1);

    if(ret<0)
        printk("\nError in pin write of EXP2");

    fc_dev = device_get_binding("FC256_NAME");

    if(fc_dev==NULL)
    	printk("fc256 driver not found\n");

    myshell_init();
    return 0;
}