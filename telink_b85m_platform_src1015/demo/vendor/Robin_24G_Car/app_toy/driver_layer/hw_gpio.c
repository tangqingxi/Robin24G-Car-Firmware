//*******************************************************************
/*
 * hw_gpio.c
 *
 *  Created on: 2026年8月20日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#include "hw_gpio.h"

//*******************************************************************
void gpio_init_out_low(GPIO_PinTypeDef pin)
{
    gpio_set_func(pin,AS_GPIO);
	gpio_set_input_en(pin,0);			//disable input
	gpio_write(pin, 0);              	//output 0
	gpio_set_output_en(pin, 1); 		//enable output
}

//*******************************************************************
void gpio_init_out_High(GPIO_PinTypeDef pin)
{
    gpio_set_func(pin,AS_GPIO);
	gpio_set_input_en(pin,0);			//disable input
	gpio_write(pin, 1);              	//output 1
	gpio_set_output_en(pin, 1); 		//enable output
}

//*******************************************************************
void gpio_output_pullup_high(GPIO_PinTypeDef pin)
{
    gpio_set_func(pin, AS_GPIO);
    //gpio_shutdown(pin);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_setup_up_down_resistor(pin, PM_PIN_PULLUP_10K);
    gpio_write(pin,1);
}

//*******************************************************************
void gpio_disable_float_low(GPIO_PinTypeDef pin)
{
    gpio_set_func(pin, AS_GPIO);
    //gpio_shutdown(pin);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 0);
    gpio_setup_up_down_resistor(pin, PM_PIN_UP_DOWN_FLOAT);
}

//*******************************************************************
void gpio_intput_pullup_10k(GPIO_PinTypeDef pin)
{
    gpio_set_func(pin, AS_GPIO);
    //gpio_shutdown(pin);
    gpio_set_input_en(pin, 1);
    gpio_set_output_en(pin, 0);
    gpio_setup_up_down_resistor(pin, PM_PIN_PULLUP_10K);
}

//*******************************************************************
void gpio_intput_pulldown_100k(GPIO_PinTypeDef pin)
{
    gpio_set_func(pin, AS_GPIO);
    //gpio_shutdown(pin);
    gpio_set_input_en(pin, 1);
    gpio_set_output_en(pin, 0);
    gpio_setup_up_down_resistor(pin, PM_PIN_PULLDOWN_100K);
}

//*******************************************************************
void gpio_intput_float(GPIO_PinTypeDef pin)
{
    gpio_set_func(pin, AS_GPIO);
    //gpio_shutdown(pin);
    gpio_set_input_en(pin, 1);
    gpio_set_output_en(pin, 0);
    gpio_setup_up_down_resistor(pin, PM_PIN_UP_DOWN_FLOAT);
}


//*******************************************************************
//*******************************************************************


