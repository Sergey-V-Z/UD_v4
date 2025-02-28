/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_spi.h"
#include "Delay_us_DWT.h"
#include "LED.h"
#include "stdio.h"
#include "externDriver.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t count_tic = 0; //для замеров времени выполнения кода
extern TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef no;
settings_t settings = {dir::CW, 1, 1};

extern_driver *pMotor;
extern_driver ext_drive(&settings, &htim4, &htim1, TIM_CHANNEL_2, &htim6, &htim3);
led LED_IPadr;
led LED_error;
led LED_OSstart;

flash mem_spi;
pins_spi_t ChipSelect = {SPI3_CS_GPIO_Port, SPI3_CS_Pin};
pins_spi_t WriteProtect = {WP_GPIO_Port, WP_Pin};
pins_spi_t Hold = {HOLD_GPIO_Port, HOLD_Pin};
bool resetSettings = false;

// Глобальный экземпляр логгера
UartLogger_t logger;

// Локальный буфер для DMA передачи
char txBuffer[MAX_MESSAGE_SIZE];
// Пул сообщений и буфер для него
LogMessage_t messagePool[QUEUE_SIZE];
uint8_t messagePoolUsed[QUEUE_SIZE];
osMutexId poolMutexHandle;

uint8_t message_rx[message_RX_LENGTH];
uint8_t UART2_rx[UART2_RX_LENGTH];
uint16_t indx_message_rx = 0;
uint16_t indx_UART2_rx = 0;
uint16_t Size_message = 0;
uint16_t Start_index = 0;
extern osMessageQId rxDataUART2Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ReadStraps();
void finishedBlink();
void timoutBlink();

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE extern "C" int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


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
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	//DWT_Init();
	uint8_t endMAC = 0, IP = 100;

	HAL_GPIO_WritePin(eth_RST_GPIO_Port, eth_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(eth_RST_GPIO_Port, eth_RST_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_SET);

	pMotor = &ext_drive;

	mem_spi.Init(&hspi3, 0, ChipSelect, WriteProtect, Hold, false);
	//HAL_Delay(100);
	mem_spi.Read(&settings);

	// если установлен джампер set
	// заходим в режим настройки
	bool StartSettings = false;
	for (int var = 0; var < 5; ++var) {

		if(HAL_GPIO_ReadPin(MAC_IP_Pin_GPIO_Port, MAC_IP_Pin_Pin)){
			StartSettings = true;
		}else{
			StartSettings = false;
			break;
		}

		HAL_Delay(30);
	}

	//режим настройки
	if(StartSettings){
		StartSettings = false;
		HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET); // PC14 VD3

		endMAC = ReadStraps();
		HAL_Delay(300);
		HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET); // PC13 VD2

		// ждем снятия джампера или таймаута
		int time;
		bool Settings;
		for (time = 0; time < 600; ++time) {

			if(HAL_GPIO_ReadPin(MAC_IP_Pin_GPIO_Port, MAC_IP_Pin_Pin)){
				Settings = true;
			}else{
				Settings = false;
				break;
			}

			HAL_Delay(100);
		}

		if(!Settings){ // if pin settings is 0
			IP = ReadStraps();
			HAL_Delay(300);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); // PC15 VD4

			if((settings.version != 0) || (settings.version != 0xFF)){ // если считанные настройки не пусты то перезаписываем только изменения

				if(endMAC != 0xFF){
					settings.MAC[5] = endMAC;
				}

				if(IP != 0xFF){
					settings.saveIP.ip[3] = IP;
				}

				mem_spi.W25qxx_EraseSector(0);
				mem_spi.Write(settings);
				mem_spi.Read(&settings);

				finishedBlink();

				// and reset system
				HAL_Delay(500);
				NVIC_SystemReset();
			}else{
				resetSettings = true; // else reset all settings
			}
		}else{

			timoutBlink();
			// and reset system
			HAL_Delay(1000);
			NVIC_SystemReset();
		}


	}
	if((settings.version == 0) || (settings.version == 0xFF) || resetSettings || settings.version != CURENT_VERSION)
	{
		//STM_LOG("Start reset settings");
		resetSettings = false;

		settings.Direct = dir::CW;
		settings.mod_rotation = mode_rotation_t::step_inf;
		settings.res2 = motor_t::stepper_motor;
		settings.Speed = 100;
		settings.StartSpeed = 100;
		settings.Accel = 500;
		settings.Slowdown = 200; // чем больше число тем медленне ускорение
		//settings.SlowdownDistancePer = 10.0; //10%
		settings.res = 0;
		settings.TimeOut = 60000;
		settings.DHCPset = true;

		settings.saveIP.ip[0] = 192;
		settings.saveIP.ip[1] = 168;
		settings.saveIP.ip[2] = 1;
		settings.saveIP.ip[3] = IP;

		settings.saveIP.mask[0] = 255;
		settings.saveIP.mask[1] = 255;
		settings.saveIP.mask[2] = 255;
		settings.saveIP.mask[3] = 0;

		settings.saveIP.gateway[0] = 192;
		settings.saveIP.gateway[1] = 168;
		settings.saveIP.gateway[2] = 1;
		settings.saveIP.gateway[3] = 1;

		settings.MAC[0] = 0x44;
		settings.MAC[1] = 0x84;
		settings.MAC[2] = 0x23;
		settings.MAC[3] = 0x84;
		settings.MAC[4] = 0x44;
		settings.MAC[5] = endMAC;

		settings.sensors_map.CW_sensor = 0;
		settings.sensors_map.CCW_sensor = 0;
		settings.sensors_map.detected = false;

		settings.version = CURENT_VERSION;

		mem_spi.W25qxx_EraseSector(0);
		mem_spi.Write(settings);
		mem_spi.Read(&settings);
	}
	//HAL_Delay(500);
	mem_spi.SetUsedInOS(true); // switch to use in OS
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

uint8_t ReadStraps(){
	uint8_t tempStraps;

	//Bit0
	if (HAL_GPIO_ReadPin(MAC_b0_GPIO_Port, MAC_b0_Pin)) SET_BIT(tempStraps,1<<0);
	else CLEAR_BIT(tempStraps,1<<0);
	//Bit1
	if (HAL_GPIO_ReadPin(MAC_b1_GPIO_Port, MAC_b1_Pin)) SET_BIT(tempStraps,1<<1);
	else CLEAR_BIT(tempStraps,1<<1);
	//Bit2
	if (HAL_GPIO_ReadPin(MAC_b2_GPIO_Port, MAC_b2_Pin)) SET_BIT(tempStraps,1<<2);
	else CLEAR_BIT(tempStraps,1<<2);
	//Bit3
	if (HAL_GPIO_ReadPin(MAC_b3_GPIO_Port, MAC_b3_Pin)) SET_BIT(tempStraps,1<<3);
	else CLEAR_BIT(tempStraps,1<<3);
	//Bit4
	if (HAL_GPIO_ReadPin(MAC_b4_GPIO_Port, MAC_b4_Pin)) SET_BIT(tempStraps,1<<4);
	else CLEAR_BIT(tempStraps,1<<4);
	//Bit5
	if (HAL_GPIO_ReadPin(MAC_b5_GPIO_Port, MAC_b5_Pin)) SET_BIT(tempStraps,1<<5);
	else CLEAR_BIT(tempStraps,1<<5);
	//Bit6
	if (HAL_GPIO_ReadPin(MAC_b6_GPIO_Port, MAC_b6_Pin)) SET_BIT(tempStraps,1<<6);
	else CLEAR_BIT(tempStraps,1<<6);
	//Bit7
	if (HAL_GPIO_ReadPin(MAC_b7_GPIO_Port, MAC_b7_Pin)) SET_BIT(tempStraps,1<<7);
	else CLEAR_BIT(tempStraps,1<<7);

	return tempStraps;
}

void finishedBlink(){
#define  timeBetween 300

	// finished blink
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); // PC15 VD4
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET); // PC13 VD2
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET); // PC14 VD3


	for (int var = 0; var < 5; ++var) {
		HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET); // PC14 VD3
		HAL_Delay(timeBetween);
		HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET); // PC14 VD3
		HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET); // PC13 VD2
		HAL_Delay(timeBetween);
		HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET); // PC13 VD2
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); // PC15 VD4
		HAL_Delay(timeBetween);
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); // PC15 VD4

	}

	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); // PC15 VD4
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET); // PC13 VD2
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET); // PC14 VD3
}

void timoutBlink(){
	// timOut plink  all
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); // PC15 VD4
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET); // PC13 VD2
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET); // PC14 VD3
	for (int var = 0; var < 5; ++var) {
		HAL_GPIO_TogglePin(B_GPIO_Port, B_Pin); // PC15 VD4
		HAL_GPIO_TogglePin(R_GPIO_Port, R_Pin); // PC13 VD2
		HAL_GPIO_TogglePin(G_GPIO_Port, G_Pin); // PC14 VD3
		HAL_Delay(800);
	}
}

// Проверка вызова из прерывания
static uint8_t isInInterrupt(void) {
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

// Получить свободный слот из пула
static int getFreeMessageSlot(void) {
    for(int i = 0; i < QUEUE_SIZE; i++) {
        if(messagePoolUsed[i] == 0) {
            messagePoolUsed[i] = 1;
            return i;
        }
    }
    return -1;
}

// �?нициализация логгера
void Logger_Init(UART_HandleTypeDef* huart) {
    // Сохраняем указатель на UART
    logger.huart = huart;
    logger.isTransmitting = 0;
    logger.txBuffer = txBuffer;
    logger.started = 1;

    // Очищаем пул сообщений
    memset(messagePoolUsed, 0, sizeof(messagePoolUsed));

    // Создаем мьютекс для пула
    osMutexDef(poolMutex);
    poolMutexHandle = osMutexCreate(osMutex(poolMutex));

    // Создаем очередь сообщений (теперь храним только индексы)
    osMessageQDef(logQueue, QUEUE_SIZE, uint32_t);
    logger.messageQueue = osMessageCreate(osMessageQ(logQueue), NULL);
}

// Обработка сообщений и отправка через UART
void Logger_Process(void) {
    if (!logger.isTransmitting) {
        osEvent event = osMessageGet(logger.messageQueue, 0); // Неблокирующее получение

        if (event.status == osEventMessage) {
            uint32_t msgIndex = event.value.v;
            if(msgIndex < QUEUE_SIZE) {
                LogMessage_t* msg = &messagePool[msgIndex];

                // Проверка валидности сообщения
                if(msg->length > 0 && msg->length < MAX_MESSAGE_SIZE) {
                    // Копируем сообщение в буфер отправки
                    memcpy(logger.txBuffer, msg->data, msg->length);

                    // Освобождаем слот в пуле
                    uint32_t primask_bit;
                    primask_bit = __get_PRIMASK();
                    __disable_irq();
                    messagePoolUsed[msgIndex] = 0;
                    if (!primask_bit) {
                        __enable_irq();
                    }

                    // Начинаем передачу
                    logger.isTransmitting = 1;

                    // Проверяем состояние DMA перед отправкой
                    if(huart2.hdmatx->State == HAL_DMA_STATE_READY) {
                        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(logger.huart, (uint8_t*)logger.txBuffer, msg->length);
                        if(status != HAL_OK) {
                            // Если DMA не готов, используем обычную передачу
                            HAL_UART_Transmit(logger.huart, (uint8_t*)logger.txBuffer, msg->length, 100);
                            logger.isTransmitting = 0;
                        }
                    } else {
                        // DMA занят, используем обычную передачу
                        HAL_UART_Transmit(logger.huart, (uint8_t*)logger.txBuffer, msg->length, 100);
                        logger.isTransmitting = 0;
                    }
                } else {
                    // Некорректный размер сообщения - просто освобождаем слот
                    uint32_t primask_bit;
                    primask_bit = __get_PRIMASK();
                    __disable_irq();
                    messagePoolUsed[msgIndex] = 0;
                    if (!primask_bit) {
                        __enable_irq();
                    }
                }
            }
        }
    }
}

// Callback завершения передачи
void Logger_TxCpltCallback(void) {
    logger.isTransmitting = 0;
}

// Функция логирования (может вызываться из прерывания или потока)
void Logger_Log(const char* format, ...) {
    if(!logger.started) return;
    if (!format) return;

    int slot = -1;

    // Безопасное получение слота с учетом контекста (прерывание или нет)
    if(isInInterrupt()) {
        // В прерывании - атомарно захватываем слот
        // Используем атомарный подход без мьютекса (мьютексы недопустимы в прерываниях)
        for(int i = 0; i < QUEUE_SIZE; i++) {
            if(messagePoolUsed[i] == 0) {
                // Попытка атомарно захватить слот
                uint32_t primask_bit;
                primask_bit = __get_PRIMASK();
                __disable_irq();
                if(messagePoolUsed[i] == 0) {
                    messagePoolUsed[i] = 1;
                    slot = i;
                }
                if (!primask_bit) {
                    __enable_irq();
                }
                if(slot >= 0) break;
            }
        }
    } else {
        // В обычном коде используем мьютекс
        osMutexWait(poolMutexHandle, osWaitForever);
        slot = getFreeMessageSlot();
        osMutexRelease(poolMutexHandle);
    }

    if(slot < 0) return; // Нет свободных слотов

    LogMessage_t* msg = &messagePool[slot];
    va_list args;

    va_start(args, format);
    int length = vsnprintf(msg->data, MAX_MESSAGE_SIZE - 2, format, args);
    va_end(args);

    if (length <= 0) {
        // Атомарное освобождение слота
        uint32_t primask_bit;
        primask_bit = __get_PRIMASK();
        __disable_irq();
        messagePoolUsed[slot] = 0;
        if (!primask_bit) {
            __enable_irq();
        }
        return;
    }

    // Добавляем '\r' и нулевой символ
    msg->data[length] = '\r';
    msg->data[length + 1] = 0;
    msg->length = length + 1;

    // Помещаем индекс сообщения в очередь
    osStatus status = osMessagePut(logger.messageQueue, slot, 0);
    if(status != osOK) {
        // Атомарное освобождение слота при ошибке
        uint32_t primask_bit;
        primask_bit = __get_PRIMASK();
        __disable_irq();
        messagePoolUsed[slot] = 0;
        if (!primask_bit) {
            __enable_irq();
        }
    }
}

// Обработчик прерывания DMA UART
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == logger.huart) {
        Logger_TxCpltCallback();
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM3)
    {
    	switch (settings.mod_rotation) {
    		case step_by_meter_enc_intermediate:
    		//case step_by_meter_enc_limit:
    		case calibration_enc:
    		{
				switch(htim->Channel)
						{
							case HAL_TIM_ACTIVE_CHANNEL_1:
								// канал торможения
								//pMotor->StepsAllHandler(__HAL_TIM_GET_COUNTER(htim));
								break;

							case HAL_TIM_ACTIVE_CHANNEL_2:
								//канал остановки

								// Прерывание от канала 2
								break;

							case HAL_TIM_ACTIVE_CHANNEL_3:
								// Прерывание от канала 3
								pMotor->slowdown();
								break;

							case HAL_TIM_ACTIVE_CHANNEL_4:
								// Прерывание от канала 4
								pMotor->stop(statusTarget_t::finished);
								break;

							case HAL_TIM_ACTIVE_CHANNEL_CLEARED:
								// Нет активного канала
								break;

							default:
								// Неизвестный канал
								break;
						}
    			break;
    		}
    		case step_by_meter_timer_intermediate:
    		//case step_by_meter_timer_limit:
    		case calibration_timer:
    		case bldc_limit:
    		case step_inf:
    		case bldc_inf:
    		{
    			return;
    			break;
    		}
    		default:
    		{
    			break;
    		}
    	}

    }

    if(htim->Instance == TIM4)
    {
    	switch (settings.mod_rotation) {
    		case step_by_meter_timer_intermediate:
    		//case step_by_meter_timer_limit:
    		case calibration_timer:
    		{
    	        switch(htim->Channel)
    	                {
    	                    case HAL_TIM_ACTIVE_CHANNEL_1:
    	                    	// канал торможения
    	                    	//pMotor->StepsAllHandler(__HAL_TIM_GET_COUNTER(htim));
    	                    	pMotor->slowdown();
    	                        break;

    	                    case HAL_TIM_ACTIVE_CHANNEL_2:
    	                    	//канал остановки
    	                    	pMotor->stop(statusTarget_t::finished);
    	                        // Прерывание от канала 2
    	                        break;

    	                    case HAL_TIM_ACTIVE_CHANNEL_3:
    	                        // Прерывание от канала 3
    	                        break;

    	                    case HAL_TIM_ACTIVE_CHANNEL_4:
    	                        // Прерывание от канала 4
    	                        break;

    	                    case HAL_TIM_ACTIVE_CHANNEL_CLEARED:
    	                        // Нет активного канала
    	                        break;

    	                    default:
    	                        // Неизвестный канал
    	                        break;
    	                }
    			break;
    		}
    		case step_by_meter_enc_intermediate:
    		//case step_by_meter_enc_limit:
    		case calibration_enc:
    		case bldc_limit:
    		case step_inf:
    		case bldc_inf:
    		{
    			return;
    			break;
    		}
    		default:
    		{
    			break;
    		}
    	}
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if ((GPIO_Pin == D0_Pin) || (GPIO_Pin == D1_Pin)) {
		//pMotor->SensHandler(GPIO_Pin);
		pMotor->StartDebounceTimer(GPIO_Pin);
	}

	if (GPIO_Pin == enc_Z_in_Pin) {
		//pMotor->SensHandler();
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        // Проверяем, что DMA_TC флаг установлен
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) != SET) {
            // Ожидаем завершения передачи
        };

        uint16_t Size_Data = Size - Start_index;

        HAL_UART_RxEventTypeTypeDef rxEventType;
        rxEventType = HAL_UARTEx_GetRxEventType(huart);
        switch (rxEventType) {
        case HAL_UART_RXEVENT_IDLE:
            // Копируем данные
            memcpy(&message_rx[indx_message_rx], &UART2_rx[Start_index], Size_Data);

            if ((message_rx[indx_message_rx + Size_Data - 1] == '\r') ||
                (message_rx[indx_message_rx + Size_Data - 1] == 0)) {
                message_rx[indx_message_rx + Size_Data] = 0;

                // Отправляем сообщение в очередь с таймаутом 0
                osStatus status = osMessagePut(rxDataUART2Handle, (uint32_t)indx_message_rx, 0);
                if (status != osOK) {
                    // Если очередь заполнена, очищаем ее
                    osEvent evt;
                    do {
                        evt = osMessageGet(rxDataUART2Handle, 0);
                    } while(evt.status == osEventMessage);

                    // Пытаемся отправить снова
                    status = osMessagePut(rxDataUART2Handle, (uint32_t)indx_message_rx, 0);
                }

                Size_message = 0;
                indx_message_rx = 0;
            } else {
                indx_message_rx += Size_Data;
            }

            Start_index = Size;
            break;

        case HAL_UART_RXEVENT_TC:
            // Копируем в начало буфера
            memcpy(&message_rx[indx_message_rx], &UART2_rx[Start_index], Size_Data);
            indx_message_rx += Size_Data;
            Start_index = 0;
            break;

        default:
            STM_LOG("Неизвестный тип события UART: %d", rxEventType);
            break;
        }

        // Перезапускаем DMA для приема
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_rx, UART2_RX_LENGTH);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if (htim->Instance == TIM4) {
    //HAL_IncTick();
	//pMotor->StepsAllHandler(__HAL_TIM_GET_COUNTER(htim));
	// что то пошло не так останов и сброс счетчиков
  }

  if (htim->Instance == TIM6)
  {
      pMotor->HandleDebounceTimeout();
  }
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
	STM_LOG("Error handler");
	__disable_irq();
	while (1)
	{
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
