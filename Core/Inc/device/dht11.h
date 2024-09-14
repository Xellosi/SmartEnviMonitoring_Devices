#ifndef MK_DHT11_H_
#define MK_DHT11_H_

#include "stm32f407xx.h"
#include "main.h"

#define TEMP_MAX 45
#define TEMP_MIN 0
#define HUMIDITY_MAX 100
#define HUMIDITY_MIN 0


struct _dht11_t {
	GPIO_TypeDef *Port;	//GPIOA
	uint16_t Pin; //GPIO_PIN_2
	TIM_HandleTypeDef *HTimUs;
	uint8_t Temperature; //Temperature value
	uint8_t Humidty; //humidity value
};
typedef struct _dht11_t DHT11Handle;

/* setup parameters related to dht11*/
void DHT11_Init(DHT11Handle *dht, TIM_HandleTypeDef *htim, GPIO_TypeDef *port, uint16_t pin);
/* read temperature and humidty  */
ErrorStatus DHT11_Read(DHT11Handle *dht);

#endif /* MK_DHT11_H_ */
