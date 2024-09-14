/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"


#include "dht11.h"
#include "lcd.h"
#include "utils.h"
#include "esp32at.h"
#include "comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//esp
#define RECV_BUFFER_NUM 5
#define RECV_BUFFER_SIZE 300
#define SEND_BUFFER_SIZE 256
#define WAIT_RES_TIMEOUTMS ((uint16_t)5000)

char http_device_url[] = "http://192.168.47.157:80/api/device";
char http_weather_url[] = "http://192.168.47.157:80/api/weather";
char server_ip[] = "192.168.47.157";
char mqtt_port[] = "1883";

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* system */
char device_id[DEVICE_UID_LEN];
DateTime_t query_time;
TimerHandle_t mstim;

/* lcd */
SemaphoreHandle_t lcd_mutex;
LCDContent_t lcd_content;

/* measurement */
DHT11Handle dht;
uint8_t tempc = 0;
uint8_t humidity = 0;
uint32_t meas_intv_sec = 10;

/* esp at */
CommHandle_t hcomm;
uint8_t send_buffer[SEND_BUFFER_SIZE];
uint16_t send_data_len = 0;
uint8_t recv_buffers[RECV_BUFFER_NUM][RECV_BUFFER_SIZE];
uint16_t recv_sizes[RECV_BUFFER_NUM] = {0};
void (*recv_callback)(const char*) = NULL;


char res_waiting[RECV_BUFFER_SIZE];
volatile bool res_receivced = false;
char* target_res = NULL;

volatile int recv_buffer_idx = 0;
volatile int recv_read_idx = 0;

volatile uint32_t meas_send_count = 0;
volatile uint32_t meas_recv_count = 0;
HAL_StatusTypeDef send_status;
HAL_StatusTypeDef receive_status;

/* os */
const char *INIT_TASK_NAME = "Init";
const char *MEAS_TASK_NAME = "Meas";
const char *ESP_RECV_TASK_NAME = "EspRecv";
TaskHandle_t meas_htask;
TaskHandle_t recv_htask;
TaskHandle_t init_htask;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
BaseType_t RTC_SetValue(DateTime_t *datetime);
BaseType_t ESP_Init(UART_HandleTypeDef *huart);
BaseType_t Server_Conn(UART_HandleTypeDef *huart, CommHandle_t *hcomm);
BaseType_t Web_Fetch_Time(UART_HandleTypeDef *huart, CommHandle_t *hcomm, DateTime_t *datetime);
BaseType_t Res_Waiting_Setup(const char* msg);
BaseType_t Wait_Esp_Res(uint16_t timeoutMs, char** res);
void Wait_Res_Compare(const char* msg);
void Handle_SysMsg(const char* msg);
void Build_Weather_Message(char* msg, const char* prefix, uint8_t value);

void Init_Task(void* arg);
//runtime
void Meas_Task(void* arg);
void Esp_Rec_Task(void* arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init(&hi2c1);
	LCD_SetContent_Refresh(&hi2c1, &lcd_content, "Init...", ".");

	BaseType_t status = pdPASS;
	//create the initializing task
	status &= xTaskCreate(Init_Task, INIT_TASK_NAME,
			(configSTACK_DEPTH_TYPE)4096, NULL, 2, &init_htask);
	configASSERT(status == pdPASS);

	//create measuring task
	status &= xTaskCreate(Meas_Task, MEAS_TASK_NAME,
			(configSTACK_DEPTH_TYPE)4096, NULL, 1, &meas_htask);
	configASSERT(status == pdPASS);

	//create the task for receiving response from esp32
	status &= xTaskCreate(Esp_Rec_Task, ESP_RECV_TASK_NAME,
			(configSTACK_DEPTH_TYPE)4096, NULL, 3, &recv_htask);
	configASSERT(status == pdPASS);

	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 40 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 40000 - 1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Detect_SDIO_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = Detect_SDIO_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//----- task handler
void Init_Task(void* arg)
{
	BaseType_t init_res;
	Read_Device_Uid(device_id);
	init_res = ESP_Init(&huart2);
	Build_CommHandle(&hcomm, http_device_url, http_weather_url, server_ip, mqtt_port, device_id, &htim13);

	Server_Conn(&huart2, &hcomm);
	init_res = Web_Fetch_Time(&huart2, &hcomm, &query_time);
	init_res = RTC_SetValue(&query_time);

	if (init_res == pdPASS){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "InitSucc.", ".");
	}
	else{
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "InitFailed", ".");
		configASSERT(init_res == pdPASS);
	}
	vTaskDelete(NULL);
	//xTaskNotify(meas_htask, 0, 0);
	/*
	 * If the task implementation ever exits the above loop, then the task
	 * must be deleted before reaching the end of its implementing function.
	 * When NULL is passed as a parameter to the vTaskDelete() API function,
	 * this indicates that the task to be deleted is the calling (this) task.
	 */
	//xTaskNotifyWait(0, 0x00, NULL, portMAX_DELAY);
	//TickType_t xDelay = pdMS_TO_TICKS(meas_intv_sec * 1000000);
	//vTaskDelay(xDelay);
}

//task for receving
void Esp_Rec_Task(void* arg)
{
	BaseType_t waitNotiResut;
	uint32_t ulNotifiedValue;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, recv_buffers[recv_buffer_idx], RECV_BUFFER_SIZE);
	for (;;) {
		xTaskNotifyWait(0, 0x00, NULL, portMAX_DELAY);
		while (recv_read_idx != recv_buffer_idx){
			Recv_Handler(recv_buffers[recv_read_idx]);
			recv_read_idx = (recv_read_idx + 1) % RECV_BUFFER_NUM;
		}
		//waitNotiResut = xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
	}
	/*
	 * If the task implementation ever exits the above loop, then the task
	 * must be deleted before reaching the end of its implementing function.
	 * When NULL is passed as a parameter to the vTaskDelete() API function,
	 * this indicates that the task to be deleted is the calling (this) task.
	 */

	vTaskDelete(NULL);
}

void Meas_Task(void* arg)
{
	DHT11_Init(&dht, &htim7, DHT11_GPIO_Port, DHT11_Pin);
	char msg1[LCD_CHAR_NUM];
	char msg2[LCD_CHAR_NUM];
	TickType_t xDelay;
	for (;;) {
		ErrorStatus status = DHT11_Read(&dht);
		if (status != ERROR) {
			tempc = dht.Temperature;
			humidity = dht.Humidty;
			status = Build_WeatherReportQuery((char*)send_buffer, &hcomm, tempc, humidity, device_id);

			status &= Res_Waiting_Setup(POST_SUCCESS) == pdPASS;
			HAL_StatusTypeDef send_status = ESP_Http_Post(&huart2, (char*)send_buffer);
			status &= Wait_Esp_Res(WAIT_RES_TIMEOUTMS, NULL) == pdPASS;
			if (send_status == HAL_OK && status == SUCCESS){
				meas_send_count++;
				Build_Weather_Message(msg1, "Temp", tempc);
				Build_Weather_Message(msg2, "Humi", humidity);
				LCD_SetContent_Refresh(&hi2c1, &lcd_content, msg1, msg2);
			}
		}
		else {

		}
		xDelay = pdMS_TO_TICKS(meas_intv_sec * 1000);
		vTaskDelay(xDelay);
	}
	/*
	 * If the task implementation ever exits the above loop, then the task
	 * must be deleted before reaching the end of its implementing function.
	 * When NULL is passed as a parameter to the vTaskDelete() API function,
	 * this indicates that the task to be deleted is the calling (this) task.
	 */
	vTaskDelete(NULL);
}

//----- functions
BaseType_t ESP_Init(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef status = ESP_Close_Echo(huart);
	HAL_Delay(1000);
	return status == HAL_OK;
}

BaseType_t Server_Conn(UART_HandleTypeDef *huart, CommHandle_t *hcomm)
{
	BaseType_t status = pdPASS;

	status = Res_Waiting_Setup(POST_SUCCESS);
	status &= Post_Login(huart, hcomm) == SUCCESS;
	status &= Wait_Esp_Res(WAIT_RES_TIMEOUTMS, NULL);

	if (status == pdFAIL){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "postlogin fail", ".");
		printf("post login failed\n");
		return status;
	}

	HAL_StatusTypeDef halstatus = HAL_OK;

	status = Res_Waiting_Setup(ESP_RESP_OK);
	halstatus = ESP_MQTT_CLEAN(&huart2);
	status &= Wait_Esp_Res(WAIT_RES_TIMEOUTMS, NULL);
	HAL_Delay(10 * 1000);
	if (halstatus != HAL_OK || status == pdFAIL){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "MQTTCleanFail", ".");
		printf("mqtt clean failed\n");
		//return status;
	}

	status = Res_Waiting_Setup(ESP_RESP_OK);
	halstatus = ESP_MQTT_Cfg(&huart2, 1, "123", "123", "123");
	status &= Wait_Esp_Res(WAIT_RES_TIMEOUTMS, NULL);

	if (halstatus != HAL_OK || status == pdFAIL){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "MQTTCfgFail", ".");
		printf("mqtt cfg failed\n");
		return status;
	}

	status = Res_Waiting_Setup(ESP_RESP_OK);
	halstatus = ESP_MQTT_CONN(&huart2, hcomm ->ServerIp, hcomm ->MQTTPort);
	status &= Wait_Esp_Res(WAIT_RES_TIMEOUTMS, NULL);

	if (halstatus != HAL_OK || status == pdFAIL){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "MQTTConnFail", ".");
		printf("mqtt conn failed\n");
		return status;
	}

	size_t len = 0;
	memcpy(send_buffer, hcomm ->DeviceId, DEVICE_UID_LEN);
	len += DEVICE_UID_LEN;
	memcpy(send_buffer + DEVICE_UID_LEN, SUFFIX_REQ, strlen(SUFFIX_REQ));
	len += strlen(SUFFIX_REQ);
	send_buffer[len++] = '\0';

	status = Res_Waiting_Setup(ESP_RESP_OK);
	halstatus = ESP_MQTT_Sub(&huart2, (char*) send_buffer);
	status = Wait_Esp_Res(WAIT_RES_TIMEOUTMS, NULL);

	if (halstatus != HAL_OK || status == pdFAIL){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "MQTTSubFail", ".");
		printf("mqtt sub failed\n");
		return pdFAIL;
	}

	memcpy(send_buffer + DEVICE_UID_LEN, SUFFIX_RES, strlen(SUFFIX_REQ));
	status = Res_Waiting_Setup(ESP_RESP_OK);
	halstatus = ESP_MQTT_Sub(&huart2, (char*) send_buffer);
	status = Wait_Esp_Res(WAIT_RES_TIMEOUTMS, NULL);

	if (halstatus != HAL_OK || status == pdFAIL){
		printf("mqtt sub failed\n");
		return pdFAIL;
	}

	return status;
}

BaseType_t Web_Fetch_Time(UART_HandleTypeDef *huart, CommHandle_t *hcomm, DateTime_t *datetime)
{
	ErrorStatus status = pdPASS;
	status = Res_Waiting_Setup(ESP_HTTPGET_RES_PREFIX);
	status &= Get_CurrentTime(huart, hcomm, datetime);
	char* res = NULL;
	status &= Wait_Esp_Res(WAIT_RES_TIMEOUTMS, &res);

	if (status == ERROR || res == NULL){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "TimeFetchFail", ".");
		printf("time fetch failed\n");
		return pdFAIL;
	}

	char* dt_res = strchr(res, KeyValueSeperater);
	if (dt_res == NULL){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "TimeFetchFail", ".");
		printf("timeFormatErr\n");
		return pdFAIL;
	}
	ErrorStatus errstat = Try_Parse_Time(dt_res + 1, datetime);
	if (errstat == ERROR){
		LCD_SetContent_Refresh(&hi2c1, &lcd_content, "TimeFetchFail", ".");
		printf("timeFormatErr\n");
		return pdFAIL;
	}
	BaseType_t retval = RTC_SetValue(datetime);
	if (retval){
		printf("timeFormatSucc\n");
	}
	else{
		printf("timeFormatErr\n");
	}
	return retval;
}

BaseType_t RTC_SetValue(DateTime_t *datetime)
{
	HAL_StatusTypeDef status;

	RTC_DateTypeDef dateInit;
	dateInit.Year = datetime->Year;
	dateInit.Month = datetime->Month;
	dateInit.Date = datetime->Day;
	//not use weekday but must assign a reasonable value to prevent uncertain error
	dateInit.WeekDay = 1;
	status = HAL_RTC_SetDate(&hrtc, &dateInit, RTC_FORMAT_BIN);

	RTC_TimeTypeDef timeInit;
	timeInit.Hours = datetime->Hours;
	timeInit.Minutes = datetime->Minutes;
	timeInit.Seconds = datetime->Seconds;
	status = HAL_RTC_SetTime(&hrtc, &timeInit, RTC_FORMAT_BIN);

	HAL_RTC_GetDate(&hrtc, &dateInit, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &timeInit, RTC_FORMAT_BIN);
	return status == HAL_OK;
}

//----- IRQ & callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint16_t size = min(Size, RECV_BUFFER_SIZE - 1);
	recv_buffers[recv_buffer_idx][size] = '\0';
	recv_buffer_idx = (recv_buffer_idx + 1) % RECV_BUFFER_NUM;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, recv_buffers[recv_buffer_idx], RECV_BUFFER_SIZE);
	BaseType_t higherPriority = pdFALSE;
	xTaskNotifyFromISR(recv_htask, 0, eNoAction, &higherPriority);
	portYIELD_FROM_ISR(higherPriority);
}

void Recv_Handler(const char* msg)
{
	if (recv_callback != NULL){
		recv_callback(msg);
	}
	Handle_SysMsg(msg);
}

BaseType_t Res_Waiting_Setup(const char* msg)
{
	BaseType_t retval = pdFAIL;
	if (msg == NULL){
		return retval;
	}

	size_t size = strlen(msg);
	memcpy(res_waiting, msg, size);
	res_waiting[size] = '\0';
	recv_callback = &Wait_Res_Compare;
	res_receivced = false;
	target_res = NULL;

	return pdPASS;
}

BaseType_t Wait_Esp_Res(uint16_t timeoutMs, char** res)
{
	BaseType_t retval = pdFAIL;
	htim13.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim13);
	while (htim13.Instance->CNT < timeoutMs){
		if (res_receivced){
			if (res != NULL){
				*res = target_res;
			}
			retval = pdPASS;
			break;
		}
	}

	HAL_TIM_Base_Stop(&htim13);

	if (retval == pdPASS){
		printf("waiting succ. %s\n", res_waiting);
	}
	else{
		printf("waiting failed. %s\n", res_waiting);
	}
	recv_callback = NULL;
	res_waiting[0] = '\0';
	return retval;
}

void Wait_Res_Compare(const char* msg)
{
	char* match_pt = strstr(msg, res_waiting);
	if (match_pt != NULL){
		target_res = msg;
		res_receivced = true;
	}
	else{
		res_receivced = false;
		target_res = NULL;
	}
}

void Handle_SysMsg(const char* msg)
{

}

void Build_Weather_Message(char* msg, const char* prefix, uint8_t value)
{
	int len = strlen(prefix);
	memcpy(msg, prefix, len);
	msg[len++] = '-';
	msg[len++] = value / 10 + '0';
	msg[len++] = value % 10 + '0';
	msg[len++] = '\0';
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName )
{
   // log_error ( "STACK OVERFLOW DETECTED: ", pcTaskName );
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
		HAL_GPIO_TogglePin(GPIOD, 14);
	    HAL_Delay(500);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
