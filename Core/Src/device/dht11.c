#include "dht11.h"

#define OUTPUT 1
#define INPUT 0


void DHT11_Init(DHT11Handle *dht, TIM_HandleTypeDef *htim, GPIO_TypeDef *port,
		uint16_t pin) {
	dht->HTimUs = htim;
	dht->Port = port;
	dht->Pin = pin;
}

void _set_dht11_gpio_mode(DHT11Handle *dht, uint8_t pMode,
		GPIO_PinState ouput_level) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	if (pMode == OUTPUT) {
		GPIO_InitStruct.Pin = dht->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(dht->Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(dht->Port, dht->Pin, ouput_level);
	} else if (pMode == INPUT) {
		GPIO_InitStruct.Pin = dht->Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(dht->Port, &GPIO_InitStruct);
	}
}


ErrorStatus DHT11_Read(DHT11Handle *dht) {
	uint16_t mTime1 = 0, mTime2 = 0, mBit = 0;
	uint8_t humVal = 0, tempVal = 0, parityVal = 0, genParity = 0;
	uint8_t mData[40];

	//start comm

	//default: Data Single-bus free status is at high voltage level.
	_set_dht11_gpio_mode(dht, OUTPUT, GPIO_PIN_SET);
	HAL_Delay(20);

	//When the communication between MCU and DHT11 begins,
	//the programme of MCU will set Data Single-bus voltage level from high to low
	//and this process must take at least 18ms to ensure DHT’s detection of MCU's signal
	HAL_GPIO_WritePin(dht->Port, dht->Pin, GPIO_PIN_RESET);
	HAL_Delay(20);

	//then MCU will pull up voltage and wait 20-40us for DHT’s response.
	HAL_GPIO_WritePin(dht->Port, dht->Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(dht->HTimUs, 0);

	//disable all interupts to do only read dht otherwise miss timer
	//__disable_irq();

	//start timer
	HAL_TIM_Base_Start(dht->HTimUs);

	//change to input mode
	_set_dht11_gpio_mode(dht, INPUT, GPIO_PIN_RESET);
	//set timer counter to zero
	__HAL_TIM_SET_COUNTER(dht->HTimUs, 0);
	//wait for DHT’s response.
	while (HAL_GPIO_ReadPin(dht->Port, dht->Pin) == GPIO_PIN_SET) {
		if ((uint16_t) __HAL_TIM_GET_COUNTER(dht->HTimUs) > 500) {
			HAL_TIM_Base_Stop(dht->HTimUs);
			//__enable_irq();
			return ERROR;
		}
	}

	//measure dht response time.
	//dht send out a low-voltage-level response signal, which lasts 80us.
	__HAL_TIM_SET_COUNTER(dht->HTimUs, 0);
	while (HAL_GPIO_ReadPin(dht->Port, dht->Pin) == GPIO_PIN_RESET) {
		if ((uint16_t) __HAL_TIM_GET_COUNTER(dht->HTimUs) > 500) {
			HAL_TIM_Base_Stop(dht->HTimUs);
			//__enable_irq();
			return ERROR;
		}
	}
	mTime1 = (uint16_t) __HAL_TIM_GET_COUNTER(dht->HTimUs);

	//Then the programme of DHT sets Data Single-bus voltage level
	//from low to high and keeps it for 80us for DHT’s preparation for sending data.
	__HAL_TIM_SET_COUNTER(dht->HTimUs, 0);
	while (HAL_GPIO_ReadPin(dht->Port, dht->Pin) == GPIO_PIN_SET) {
		if ((uint16_t) __HAL_TIM_GET_COUNTER(dht->HTimUs) > 500) {
			HAL_TIM_Base_Stop(dht->HTimUs);
			//__enable_irq();
			return ERROR;
		}
	}
	mTime2 = (uint16_t) __HAL_TIM_GET_COUNTER(dht->HTimUs);

	//pre-process abnormal
	if (mTime1 < 60 || mTime1 > 100 || mTime2 < 60 || mTime2 > 100) {
		HAL_TIM_Base_Stop(dht->HTimUs);
		//__enable_irq();
		return ERROR;
	}

	for (int j = 0; j < 40; j++) {
		__HAL_TIM_SET_COUNTER(dht->HTimUs, 0);
		while (HAL_GPIO_ReadPin(dht->Port, dht->Pin) == GPIO_PIN_RESET) {
			if ((uint16_t) __HAL_TIM_GET_COUNTER(dht->HTimUs) > 500) {
				HAL_TIM_Base_Stop(dht->HTimUs);
				//__enable_irq();
				return ERROR;
			}

		}

		__HAL_TIM_SET_COUNTER(dht->HTimUs, 0);
		while (HAL_GPIO_ReadPin(dht->Port, dht->Pin) == GPIO_PIN_SET) {
			if ((uint16_t) __HAL_TIM_GET_COUNTER(dht->HTimUs) > 500) {
				HAL_TIM_Base_Stop(dht->HTimUs);
				//__enable_irq();
				return ERROR;
			}

		}

		mTime1 = (uint16_t) __HAL_TIM_GET_COUNTER(dht->HTimUs);

		//check pass time in high state
		//if pass time 25uS set as LOW
		if (mTime1 > 15 && mTime1 < 40) {
			mBit = 0;
		} else if (mTime1 > 55 && mTime1 < 90) //if pass time 70 uS set as HIGH
				{
			mBit = 1;
		}
		//set i th data in data buffer
		mData[j] = mBit;

	}

	HAL_TIM_Base_Stop(dht->HTimUs);
	_set_dht11_gpio_mode(dht, OUTPUT, GPIO_PIN_SET);
	//enable all interrupts
	//__enable_irq();

	//get hum value from data buffer
	for (int i = 0; i < 8; i++) {
		humVal += mData[i];
		humVal = humVal << 1;
	}

	//get temp value from data buffer
	for (int i = 16; i < 24; i++) {
		tempVal += mData[i];
		tempVal = tempVal << 1;
	}

	//get parity value from data buffer
	for (int i = 32; i < 40; i++) {
		parityVal += mData[i];
		parityVal = parityVal << 1;
	}

	parityVal = parityVal >> 1;
	humVal = humVal >> 1;
	tempVal = tempVal >> 1;

	genParity = humVal + tempVal;

	dht->Temperature = tempVal;
	dht->Humidty = humVal;

	if (tempVal >= TEMP_MIN && tempVal <= TEMP_MAX && humVal >= HUMIDITY_MIN
			&& humVal <= HUMIDITY_MAX) {
		return SUCCESS;
	} else {
		return ERROR;
	}
}
