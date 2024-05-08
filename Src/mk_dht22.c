/**
 *  @file mk_dht22.c
 *	@brief dht22 Library
 *  @date Created on: Oct 4, 2019
 *  @author Author: mesut.kilic
 *	@version 1.0.0
 */

#include "mk_dht22.h"

/**
 * @brief configure dht22 struct with given parameter
 * @param htim TIMER for calculate delays ex:&htim2
 * @param port GPIO port ex:GPIOA
 * @param pin GPIO pin ex:GPIO_PIN_2
 * @param dht2 struct to configure ex:&dht2
 */
void init_dht22(dht22_t *dht2, TIM_HandleTypeDef *htim, GPIO_TypeDef* port, uint16_t pin){
	dht2->htim = htim;
	dht2->port = port;
	dht2->pin = pin;
}

/**
 * @brief set DHT pin direction with given parameter
 * @param dht2 struct for dht2
 * @param pMode GPIO Mode ex:INPUT or OUTPUT
 */
void set_dht22_gpio_mode(dht22_t *dht2, uint8_t pMode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(pMode == OUTPUT)
	{
	  GPIO_InitStruct.Pin = dht2->pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(dht2->port, &GPIO_InitStruct);
	}else if(pMode == INPUT)
	{
	  GPIO_InitStruct.Pin = dht2->pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(dht2->port, &GPIO_InitStruct);
	}
}

/**
 * @brief reads dht22 value
 * @param dht struct for dht22
 * @return 1 if read ok 0 if something wrong in read
 */
uint8_t readdht22(dht22_t *dht2)
{
	uint16_t mTime1 = 0, mTime2 = 0, mBit = 0;
	uint8_t humVal = 0, tempVal = 0, parityVal = 0, genParity = 0;
	uint8_t mData[40];

	//start comm
	set_dht22_gpio_mode(dht2, OUTPUT);			//set pin direction as input
	HAL_GPIO_WritePin(dht2->port, dht2->pin, GPIO_PIN_RESET);
	HAL_Delay(18);					//wait 18 ms in Low state
	__disable_irq();	//disable all interupts to do only read dht otherwise miss timer
	HAL_TIM_Base_Start(dht2->htim); //start timer
	set_dht22_gpio_mode(dht2, INPUT);
	//check dht answer
	__HAL_TIM_SET_COUNTER(dht2->htim, 0);				//set timer counter to zero
	while(HAL_GPIO_ReadPin(dht2->port, dht2->pin) == GPIO_PIN_SET){
		if((uint16_t)__HAL_TIM_GET_COUNTER(dht2->htim) > 500){
			__enable_irq();
			return 0;
		}
	}
	__HAL_TIM_SET_COUNTER(dht2->htim, 0);
	while(HAL_GPIO_ReadPin(dht2->port, dht2->pin) == GPIO_PIN_RESET){
		if((uint16_t)__HAL_TIM_GET_COUNTER(dht2->htim) > 500){
			__enable_irq();
			return 0;
		}
	}
	mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(dht2->htim);
	__HAL_TIM_SET_COUNTER(dht2->htim, 0);
	while(HAL_GPIO_ReadPin(dht2->port, dht2->pin) == GPIO_PIN_SET){
		if((uint16_t)__HAL_TIM_GET_COUNTER(dht2->htim) > 500){
			__enable_irq();
			return 0;
		}
	}
	mTime2 = (uint16_t)__HAL_TIM_GET_COUNTER(dht2->htim);

	//if answer is wrong return
	if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
	{
		__enable_irq();
		return 0;
	}

//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	for(int j = 0; j < 40; j++)
	{
		__HAL_TIM_SET_COUNTER(dht2->htim, 0);
		while(HAL_GPIO_ReadPin(dht2->port, dht2->pin) == GPIO_PIN_RESET){
			if((uint16_t)__HAL_TIM_GET_COUNTER(dht2->htim) > 500){
				__enable_irq();
				return 0;
			}

		}
		__HAL_TIM_SET_COUNTER(dht2->htim, 0);
		while(HAL_GPIO_ReadPin(dht2->port, dht2->pin) == GPIO_PIN_SET){
			if((uint16_t)__HAL_TIM_GET_COUNTER(dht2->htim) > 500){
				__enable_irq();
				return 0;
			}

		}
		mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(dht2->htim);

		//check pass time in high state
		//if pass time 25uS set as LOW
		if(mTime1 > 20 && mTime1 < 30)
		{
			mBit = 0;
		}
		else if(mTime1 > 60 && mTime1 < 80) //if pass time 70 uS set as HIGH
		{
			 mBit = 1;
		}

		//set i th data in data buffer
		mData[j] = mBit;

	}

	HAL_TIM_Base_Stop(dht2->htim); //stop timer
	__enable_irq(); //enable all interrupts

	//get hum value from data buffer
	for(int i = 0; i < 8; i++)
	{
		humVal += mData[i];
		humVal = humVal << 1;
	}

	//get temp value from data buffer
	for(int i = 16; i < 24; i++)
	{
		tempVal += mData[i];
		tempVal = tempVal << 1;
	}

	//get parity value from data buffer
	for(int i = 32; i < 40; i++)
	{
		parityVal += mData[i];
		parityVal = parityVal << 1;
	}

	parityVal = parityVal >> 1;
	humVal = humVal >> 1;
	tempVal = tempVal >> 1;

	genParity = humVal + tempVal;

//	if(genParity == parityVal)

	dht2->temperature = tempVal;
	dht2->humidty = humVal;


	return 1;
}
