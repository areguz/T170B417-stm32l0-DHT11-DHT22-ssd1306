/**
 *  @file mk_dht22.h
 *	@brief DHT22 Library
 *  @date Created on: Oct 4, 2019
 *  @author Author: mesut.kilic
 *	@version 1.0.0
 */


#ifndef MK_DHT22_H_
#define MK_DHT22_H_

#include "stm32l0xx.h"
#include "main.h"
#include "gpio.h"

#define OUTPUT 1
#define INPUT 0

/**
 * @brief DHT22 struct
 */
struct _dht22_t{
	GPIO_TypeDef* port;	///GPIO Port ex:GPIOA
	uint16_t pin; ///GPIO pin ex:GPIO_PIN_3
	TIM_HandleTypeDef *htim; /// timer for measure time ex:htim3
	uint8_t temperature; ///Temperature value
	uint8_t humidty; ///humidity value
};
typedef struct _dht22_t dht22_t;


void init_dht22(dht22_t *dht2, TIM_HandleTypeDef *htim, GPIO_TypeDef* port, uint16_t pin);
void set_dht22_gpio_mode(dht22_t *dht2, uint8_t pMode);
uint8_t readDHT22(dht22_t *dht2);

#endif /* MK_DHT22_H_ */
