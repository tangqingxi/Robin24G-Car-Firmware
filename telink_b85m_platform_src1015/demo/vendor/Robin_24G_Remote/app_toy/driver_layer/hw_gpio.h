//*******************************************************************
/*
 * hw_gpio.h
 *
 *  Created on: 2026年8月19日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#ifndef VENDOR_ROBIN_24G_REMOTE_APP_TOY_DRIVER_LAYER_HW_GPIO_H_
#define VENDOR_ROBIN_24G_REMOTE_APP_TOY_DRIVER_LAYER_HW_GPIO_H_
//*******************************************************************
//*******************************************************************
#include "app_public_config.h"

//*******************************************************************
void gpio_init_out_low(GPIO_PinTypeDef pin);
void gpio_init_out_High(GPIO_PinTypeDef pin);
void gpio_output_pullup_high(GPIO_PinTypeDef pin);
void gpio_disable_float(GPIO_PinTypeDef pin);
void gpio_disable_PullUp_10k(GPIO_PinTypeDef pin);
void gpio_disable_PullDown(GPIO_PinTypeDef pin);
void gpio_intput_pullup_10k(GPIO_PinTypeDef pin);
void gpio_intput_pulldown_100k(GPIO_PinTypeDef pin);
void gpio_intput_float(GPIO_PinTypeDef pin);


//*******************************************************************
#endif /* VENDOR_ROBIN_24G_REMOTE_APP_TOY_DRIVER_LAYER_HW_GPIO_H_ */
//*******************************************************************

