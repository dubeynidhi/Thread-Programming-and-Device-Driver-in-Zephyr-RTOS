#include <device.h>
#include <gpio.h>
#include <misc/util.h>



struct hcsr_data {
	void (*function)();
};

