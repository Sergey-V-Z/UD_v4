/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_spi.h"
#include "LED.h"
#include "lwip.h"
using namespace std;
#include <string>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include "api.h"
#include <iostream>
#include <vector>
#include "device_API.h"
#include <iostream>
#include <iomanip>
#include "externDriver.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//for ethernetif.c
//extern settings_t settings;
//MACAddr[5] = settings.MAC_end;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern settings_t settings;
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim3;
//extern DAC_HandleTypeDef hdac;
//extern DMA_HandleTypeDef hdma_adc1;
//extern ADC_HandleTypeDef hadc1;

extern extern_driver *pMotor;
extern led LED_IPadr;
extern led LED_error;
extern led LED_OSstart;

void actoin_motor_set(cJSON *obj, bool save);
void actoin_ip(cJSON *obj, bool save);
void actoin_resp_all_set(void);
void actoin_resp_status(void);

uint16_t ADC_Data[40];
//переменные для дебага
int Start = false;
bool DR = false;
uint32_t steps = 2000;
//dir dir1 = dir::CW;

extern settings_t settings;

// for SPI Flash
extern SPI_HandleTypeDef hspi3;

extern flash mem_spi;

//структуры для netcon
extern struct netif gnetif;

//TCP_IP
string strIP;
string in_str;

//переменные для обшей работы
uint32_t var_sys[100];

extern uint8_t message_rx[message_RX_LENGTH];
extern uint8_t UART2_rx[UART2_RX_LENGTH];
extern uint16_t indx_message_rx;
extern uint16_t indx_UART2_rx;
extern uint16_t Size_message;
extern uint16_t Start_index;
/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId Motor_poolHandle;
osThreadId ledTaskHandle;
osThreadId callTaskHandle;
osThreadId uart_taskHandle;
osThreadId loggerTaskHandle;
osMessageQId rxDataUART2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// extern "C"
/* USER CODE END FunctionPrototypes */

void MainTask(void const * argument);
void motor_pool(void const * argument);
void LedTask(void const * argument);
void CallTask(void const * argument);
void uart_Task(void const * argument);
void LoggerTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
extern "C" void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of rxDataUART2 */
  osMessageQDef(rxDataUART2, 16, uint8_t);
  rxDataUART2Handle = osMessageCreate(osMessageQ(rxDataUART2), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, MainTask, osPriorityNormal, 0, 512);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of Motor_pool */
  osThreadDef(Motor_pool, motor_pool, osPriorityNormal, 0, 512);
  Motor_poolHandle = osThreadCreate(osThread(Motor_pool), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, LedTask, osPriorityNormal, 0, 512);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of callTask */
  osThreadDef(callTask, CallTask, osPriorityNormal, 0, 128);
  callTaskHandle = osThreadCreate(osThread(callTask), NULL);

  /* definition and creation of uart_task */
  osThreadDef(uart_task, uart_Task, osPriorityNormal, 0, 1024);
  uart_taskHandle = osThreadCreate(osThread(uart_task), NULL);

  /* definition and creation of loggerTask */
  osThreadDef(loggerTask, LoggerTask, osPriorityNormal, 0, 128);
  loggerTaskHandle = osThreadCreate(osThread(loggerTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_MainTask */
/**
 * @brief  Function implementing the mainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_MainTask */
void MainTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN MainTask */
  //нициализируем логгер
  Logger_Init(&huart2);

  STM_LOG("Start step enc. %f", 0.01);

	LED_IPadr.setParameters(mode::ON_OFF);
	while (gnetif.ip_addr.addr == 0) {
		osDelay(1);
	}	//ждем получение адреса
	LED_IPadr.LEDon();
	osDelay(1000);
	LED_IPadr.LEDoff();
	strIP = ip4addr_ntoa(&gnetif.ip_addr);
	STM_LOG("IP: %s", strIP.c_str());

	//структуры для netcon
	struct netconn *conn = NULL;
	struct netconn *newconn;
	struct netbuf *netbuf;
	volatile err_t err, accept_err;
	//ip_addr_t local_ip;
	//ip_addr_t remote_ip;
	void *in_data = NULL;
	uint16_t data_size = 0;

	//Флаги для разбора сообщения
	string f_cmd("C");
	string f_addr("A");
	string f_datd("D");
	string delim("x");

	/* Infinite loop */
	for (;;) {

		conn = netconn_new(NETCONN_TCP);
		if (conn != NULL) {
			err = netconn_bind(conn, NULL, 81);	//assign port number to connection
			if (err == ERR_OK) {
				netconn_listen(conn);	//set port to listening mode
				while (1) {
					accept_err = netconn_accept(conn, &newconn);//suspend until new connection
					if (accept_err == ERR_OK) {
						LED_IPadr.LEDon();
						STM_LOG("Connect open");
						while ((accept_err = netconn_recv(newconn, &netbuf))
								== ERR_OK)//работаем до тех пор пока клиент не разорвет соеденение
						{

							do {
								netbuf_data(netbuf, &in_data, &data_size);//get pointer and data size of the buffer
								in_str.assign((char*) in_data, data_size);//copy in string
								/*-----------------------------------------------------------------------------------------------------------------------------*/
								STM_LOG("Get CMD %s", in_str.c_str());

								if(!in_str.empty())
								{
									string resp = Command_execution(in_str);
									netconn_write(newconn, resp.c_str(), resp.size(), NETCONN_COPY);
								}

							} while (netbuf_next(netbuf) >= 0);
							netbuf_delete(netbuf);

						}
						netconn_close(newconn);
						netconn_delete(newconn);
						STM_LOG("Connect close");
						LED_IPadr.LEDoff();
					} else
						netconn_delete(newconn);
					osDelay(20);
				}
			}
		}
		osDelay(1);
	}
  /* USER CODE END MainTask */
}

/* USER CODE BEGIN Header_motor_pool */
/**
 * @brief Function implementing the Motor_pool thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_motor_pool */
void motor_pool(void const * argument)
{
  /* USER CODE BEGIN motor_pool */
	pMotor->Init();
	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	/* Infinite loop */
	for (;;) {
		//osDelay(1);
		pMotor->AccelHandler();

		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровно через 1 милисекунду
		osDelay(1);
	}
  /* USER CODE END motor_pool */
}

/* USER CODE BEGIN Header_LedTask */
/**
 * @brief Function implementing the ledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LedTask */
void LedTask(void const * argument)
{
  /* USER CODE BEGIN LedTask */

	LED_IPadr.Init(G_GPIO_Port, G_Pin);
	LED_error.Init(R_GPIO_Port, R_Pin);
	LED_OSstart.Init(B_GPIO_Port, B_Pin);
	LED_OSstart.setParameters(mode::BLINK, 1000, 100);
	LED_OSstart.LEDon();

	//uint32_t tickcount = osKernelSysTick();// переменная для точной задержки
	/* Infinite loop */
	for (;;) {
		LED_IPadr.poll();
		LED_error.poll();
		LED_OSstart.poll();

		osDelay(1);
		//taskYIELD();
		//osDelayUntil(&tickcount, 1); // задача будет вызываься ровро через 1 милисекунду
	}
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_CallTask */
/**
 * @brief Function implementing the callTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CallTask */
void CallTask(void const * argument)
{
  /* USER CODE BEGIN CallTask */

	/* Infinite loop */
	for (;;) {
		if(pMotor->Calibration_pool())
		{

            // Сохраняем настройки в память
            //mem_spi.W25qxx_EraseSector(0);
            //osDelay(5);
            //mem_spi.Write(settings);

            STM_LOG("Calibration completed successfully");
		}
		//pMotor->findHome();
		osDelay(1);
	}
  /* USER CODE END CallTask */
}

/* USER CODE BEGIN Header_uart_Task */
/**
 * @brief Function implementing the uart_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_uart_Task */
void uart_Task(void const * argument)
{
  /* USER CODE BEGIN uart_Task */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_rx, UART2_RX_LENGTH);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    for(;;) {
        // Меньший тайм-аут для лучшей отзывчивости системы
        osEvent evt = osMessageGet(rxDataUART2Handle, 5000); // 5 секунд вместо 60

        if (evt.status == osEventMessage) {
            // Сообщение успешно получено - обрабатываем

            // парсим json
            cJSON *json = cJSON_Parse((char*) message_rx);
            if (json != NULL) {
                cJSON *id = cJSON_GetObjectItemCaseSensitive(json, "id");
                cJSON *name_device = cJSON_GetObjectItemCaseSensitive(json, "name_device");
                cJSON *type_data = cJSON_GetObjectItemCaseSensitive(json, "type_data");
                cJSON *save_settings = cJSON_GetObjectItemCaseSensitive(json, "save_settings");
                cJSON *obj = cJSON_GetObjectItemCaseSensitive(json, "obj");

                if (cJSON_IsNumber(id) && cJSON_GetNumberValue(id) == ID_CTRL) {
                    bool save_set = false;
                    if (cJSON_IsTrue(save_settings)) {
                        save_set = true;
                    } else {
                        save_set = false;
                    }

                    if (cJSON_IsNumber(type_data)) {
                        switch (type_data->valueint) {
                        case 1: // ip settings
                            actoin_ip(obj, save_set);
                            break;
                        case 2: // motor settings
                            actoin_motor_set(obj, save_set);
                            break;
                        case 3:
                            actoin_resp_all_set();
                            break;
                        case 4:
                            actoin_resp_status();
                            break;
                        default:
                            STM_LOG("data type not registered");
                            break;
                        }
                    }
                } else {
                    STM_LOG("id not valid");
                }

                cJSON_Delete(json);
            } else {
                STM_LOG("Invalid JSON");
            }
        } else if (evt.status == osEventTimeout) {
            // В случае тайм-аута, просто выводим информационное сообщение и продолжаем
            //STM_LOG("No message received, waiting...");
        }

        osDelay(10); // Добавляем небольшую задержку для экономии CPU
    }
  /* USER CODE END uart_Task */
}

/* USER CODE BEGIN Header_LoggerTask */
/**
* @brief Function implementing the loggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoggerTask */
void LoggerTask(void const * argument)
{
  /* USER CODE BEGIN LoggerTask */
  /* Infinite loop */
  for(;;)
  {
	Logger_Process();
    osDelay(1);
  }
  /* USER CODE END LoggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void actoin_motor_set(cJSON *obj, bool save) {
	cJSON *j_speed = cJSON_GetObjectItemCaseSensitive(obj, "speed");
	cJSON *j_set_speed = cJSON_GetObjectItemCaseSensitive(obj, "set_speed");
	cJSON *j_start_speed = cJSON_GetObjectItemCaseSensitive(obj, "start_speed");
	cJSON *j_set_start_speed = cJSON_GetObjectItemCaseSensitive(obj, "set_start_speed");
	cJSON *j_accel = cJSON_GetObjectItemCaseSensitive(obj, "accel");
	cJSON *j_set_accel = cJSON_GetObjectItemCaseSensitive(obj, "set_accel");
	cJSON *j_slow = cJSON_GetObjectItemCaseSensitive(obj, "slow");
	cJSON *j_set_slow = cJSON_GetObjectItemCaseSensitive(obj, "set_slow");
	cJSON *j_target = cJSON_GetObjectItemCaseSensitive(obj, "target");
	cJSON *j_set_target = cJSON_GetObjectItemCaseSensitive(obj, "set_target");
	cJSON *j_time_out = cJSON_GetObjectItemCaseSensitive(obj, "time_out");
	cJSON *j_set_time_out = cJSON_GetObjectItemCaseSensitive(obj, "set_time_out");
	cJSON *j_cw_ccw = cJSON_GetObjectItemCaseSensitive(obj, "cw_ccw");
	cJSON *j_set_cw_ccw = cJSON_GetObjectItemCaseSensitive(obj, "set_cw_ccw");
	cJSON *j_rotation = cJSON_GetObjectItemCaseSensitive(obj, "rotation");
	cJSON *j_set_rotation = cJSON_GetObjectItemCaseSensitive(obj,"set_rotation");

	// Временные переменные для хранения значений до проверки их корректности
	int Speed = 0;
	int StartSpeed = 0;
	int Acceleration = 0;
	int Slowdown = 0;
	int SlowdownDistance = 0;
	int Target = 0;
	int TimeOut = 0;

	// Флаг для отслеживания ошибок в параметрах
	bool paramError = false;

	// Проверка параметра speed
	if ((j_set_speed != NULL) && (j_speed != NULL) && cJSON_IsString(j_speed)) {
		if (cJSON_IsTrue(j_set_speed)) {
			// Безопасное преобразование строки в число
			char* endptr;
			Speed = strtol(j_speed->valuestring, &endptr, 10);
			if (*endptr != '\0') {
				paramError = true;
				STM_LOG("Error converting speed parameter");
			}
		}
	} else if (cJSON_IsTrue(j_set_speed)) {
		paramError = true;
		STM_LOG("Invalid speed parameter");
	}

	// Проверка параметра start_speed
	if ((j_set_start_speed != NULL) && (j_start_speed != NULL) && cJSON_IsString(j_start_speed)) {
		if (cJSON_IsTrue(j_set_start_speed)) {
			char* endptr;
			StartSpeed = strtol(j_start_speed->valuestring, &endptr, 10);
			if (*endptr != '\0') {
				paramError = true;
				STM_LOG("Error converting start_speed parameter");
			}
		}
	} else if (cJSON_IsTrue(j_set_start_speed)) {
		paramError = true;
		STM_LOG("Invalid start_speed parameter");
	}

	// Проверка параметра accel
	if ((j_set_accel != NULL) && (j_accel != NULL) && cJSON_IsString(j_accel)) {
		if (cJSON_IsTrue(j_set_accel)) {
			char* endptr;
			Acceleration = strtol(j_accel->valuestring, &endptr, 10);
			if (*endptr != '\0') {
				paramError = true;
				STM_LOG("Error converting accel parameter");
			}
		}
	} else if (cJSON_IsTrue(j_set_accel)) {
		paramError = true;
		STM_LOG("Invalid accel parameter");
	}

	// Проверка параметра slow
	if ((j_set_slow != NULL) && (j_slow != NULL) && cJSON_IsString(j_slow)) {
		if (cJSON_IsTrue(j_set_slow)) {
			char* endptr;
			Slowdown = strtol(j_slow->valuestring, &endptr, 10);
			if (*endptr != '\0') {
				paramError = true;
				STM_LOG("Error converting slow parameter");
			}
		}
	} else if (cJSON_IsTrue(j_set_slow)) {
		paramError = true;
		STM_LOG("Invalid slow parameter");
	}

	// Проверка параметра target
	if ((j_set_target != NULL) && (j_target != NULL) && cJSON_IsString(j_target)) {
		if (cJSON_IsTrue(j_set_target)) {
			char* endptr;
			Target = strtol(j_target->valuestring, &endptr, 10);
			if (*endptr != '\0') {
				paramError = true;
				STM_LOG("Error converting target parameter");
			}
		}
	} else if (cJSON_IsTrue(j_set_target)) {
		paramError = true;
		STM_LOG("Invalid target parameter");
	}

	// Проверка параметра time_out
	if ((j_set_time_out != NULL) && (j_time_out != NULL) && cJSON_IsString(j_time_out)) {
		if (cJSON_IsTrue(j_set_time_out)) {
			char* endptr;
			TimeOut = strtol(j_time_out->valuestring, &endptr, 10);
			if (*endptr != '\0') {
				paramError = true;
				STM_LOG("Error converting time_out parameter");
			}
		}
	} else if (cJSON_IsTrue(j_set_time_out)) {
		paramError = true;
		STM_LOG("Invalid time_out parameter");
	}

	// Если были ошибки в параметрах, завершаем функцию
	if (paramError) {
		STM_LOG("Error in motor parameters");
		return;
	}

	// Запись настроек в мотор (только если не было ошибок в параметрах)
	if (cJSON_IsTrue(j_set_speed))
		pMotor->SetSpeed(Speed);

	if (cJSON_IsTrue(j_set_start_speed))
		pMotor->SetStartSpeed(StartSpeed);

	if (cJSON_IsTrue(j_set_accel))
		pMotor->SetAcceleration(Acceleration);

	if (cJSON_IsTrue(j_set_slow))
		pMotor->SetSlowdown(Slowdown);

	if (cJSON_IsTrue(j_set_time_out))
		pMotor->setTimeOut(TimeOut);

	// Обработка параметра cw_ccw
	if ((j_set_cw_ccw != NULL) && (j_cw_ccw != NULL) && cJSON_IsNumber(j_cw_ccw)) {
		if (cJSON_IsTrue(j_set_cw_ccw)) {
			// Проверка на диапазон
			if (j_cw_ccw->valuedouble >= dir::CW && j_cw_ccw->valuedouble <= dir::CCW)
				pMotor->SetDirection((dir)j_cw_ccw->valuedouble);
			else
				pMotor->SetDirection(dir::CW);
		}
	}

	// Обработка параметра rotation
	if ((j_set_rotation != NULL) && (j_rotation != NULL) && cJSON_IsNumber(j_rotation)) {
		if (cJSON_IsTrue(j_set_rotation)) {
			pMotor->SetMode((mode_rotation_t)j_rotation->valuedouble);
		}
	}

	STM_LOG("Settings motor set successful");

	// Сохранение настроек при необходимости
	if (save) {
		STM_LOG("Save motor settings");
		mem_spi.W25qxx_EraseSector(0);
		osDelay(5);
		mem_spi.Write(settings);
	}
}

void actoin_ip(cJSON *obj, bool save) {
	cJSON *j_IP = cJSON_GetObjectItemCaseSensitive(obj, "IP");
	cJSON *j_setIP = cJSON_GetObjectItemCaseSensitive(obj, "setIP");
	cJSON *j_MAC = cJSON_GetObjectItemCaseSensitive(obj, "MAC");
	cJSON *j_setMAC = cJSON_GetObjectItemCaseSensitive(obj, "setMAC");
	cJSON *j_GATEWAY = cJSON_GetObjectItemCaseSensitive(obj, "GATEWAY");
	cJSON *j_setGATEWAY = cJSON_GetObjectItemCaseSensitive(obj, "setGATEWAY");
	cJSON *j_MASK = cJSON_GetObjectItemCaseSensitive(obj, "MASK");
	cJSON *j_setMASK = cJSON_GetObjectItemCaseSensitive(obj, "setMASK");
	cJSON *j_DNS = cJSON_GetObjectItemCaseSensitive(obj, "DNS");
	cJSON *j_setDNS = cJSON_GetObjectItemCaseSensitive(obj, "setDNS");
	cJSON *j_DHCP = cJSON_GetObjectItemCaseSensitive(obj, "DHCP");
	cJSON *j_setDHCP = cJSON_GetObjectItemCaseSensitive(obj, "setDHCP");

	// Флаг для отслеживания ошибок в параметрах
	bool paramError = false;

	// Функция для парсинга IP-адреса формата "xxx.xxx.xxx.xxx"
	auto parseIPAddress = [&paramError](const char* ipString, uint8_t* destination) -> bool {
		if (ipString == NULL || destination == NULL) {
			STM_LOG("Invalid IP string or destination");
			return false;
		}

		char* ipCopy = strdup(ipString);
		if (ipCopy == NULL) {
			STM_LOG("Memory allocation failed");
			return false;
		}

		char* token = strtok(ipCopy, ".");
		int i = 0;

		while (token != NULL && i < 4) {
			char* endptr;
			long value = strtol(token, &endptr, 10);

			// Проверка на корректность преобразования и диапазон значений
			if (*endptr != '\0' || value < 0 || value > 255) {
				STM_LOG("Invalid IP address format");
				paramError = true;
				free(ipCopy);
				return false;
			}

			destination[i++] = (uint8_t)value;
			token = strtok(NULL, ".");
		}

		// Проверка, что у нас ровно 4 октета
		if (i != 4 || strtok(NULL, ".") != NULL) {
			STM_LOG("IP address must have exactly 4 octets");
			paramError = true;
			free(ipCopy);
			return false;
		}

		free(ipCopy);
		return true;
	};

	// Функция для парсинга MAC-адреса формата "xx:xx:xx:xx:xx:xx"
	auto parseMACAddress = [&paramError](const char* macString, uint8_t* destination) -> bool {
		if (macString == NULL || destination == NULL) {
			STM_LOG("Invalid MAC string or destination");
			return false;
		}

		char* macCopy = strdup(macString);
		if (macCopy == NULL) {
			STM_LOG("Memory allocation failed");
			return false;
		}

		char* token = strtok(macCopy, ":");
		int i = 0;

		while (token != NULL && i < 6) {
			char* endptr;
			long value = strtol(token, &endptr, 16);

			// Проверка на корректность преобразования и диапазон значений
			if (*endptr != '\0' || value < 0 || value > 255) {
				STM_LOG("Invalid MAC address format");
				paramError = true;
				free(macCopy);
				return false;
			}

			destination[i++] = (uint8_t)value;
			token = strtok(NULL, ":");
		}

		// Проверка, что у нас ровно 6 октетов
		if (i != 6 || strtok(NULL, ":") != NULL) {
			STM_LOG("MAC address must have exactly 6 octets");
			paramError = true;
			free(macCopy);
			return false;
		}

		free(macCopy);
		return true;
	};

	// Обработка IP-адреса
	if ((j_setIP != NULL) && cJSON_IsTrue(j_setIP)) {
		if (j_IP == NULL || !cJSON_IsString(j_IP) || j_IP->valuestring == NULL) {
			STM_LOG("Invalid IP parameter");
			paramError = true;
		} else {
			if (!parseIPAddress(j_IP->valuestring, settings.saveIP.ip)) {
				STM_LOG("Failed to parse IP address");
			}
		}
	}

	// Обработка MAC-адреса
	if ((j_setMAC != NULL) && cJSON_IsTrue(j_setMAC)) {
		if (j_MAC == NULL || !cJSON_IsString(j_MAC) || j_MAC->valuestring == NULL) {
			STM_LOG("Invalid MAC parameter");
			paramError = true;
		} else {
			if (!parseMACAddress(j_MAC->valuestring, settings.MAC)) {
				STM_LOG("Failed to parse MAC address");
			}
		}
	}

	// Обработка Gateway-адреса
	if ((j_setGATEWAY != NULL) && cJSON_IsTrue(j_setGATEWAY)) {
		if (j_GATEWAY == NULL || !cJSON_IsString(j_GATEWAY) || j_GATEWAY->valuestring == NULL) {
			STM_LOG("Invalid GATEWAY parameter");
			paramError = true;
		} else {
			if (!parseIPAddress(j_GATEWAY->valuestring, settings.saveIP.gateway)) {
				STM_LOG("Failed to parse Gateway address");
			}
		}
	}

	// Обработка Mask-адреса
	if ((j_setMASK != NULL) && cJSON_IsTrue(j_setMASK)) {
		if (j_MASK == NULL || !cJSON_IsString(j_MASK) || j_MASK->valuestring == NULL) {
			STM_LOG("Invalid MASK parameter");
			paramError = true;
		} else {
			if (!parseIPAddress(j_MASK->valuestring, settings.saveIP.mask)) {
				STM_LOG("Failed to parse Mask address");
			}
		}
	}

	// Обработка DNS-адреса (закомментировано в оригинальном коде)
	if ((j_setDNS != NULL) && cJSON_IsTrue(j_setDNS)) {
		if (j_DNS == NULL || !cJSON_IsString(j_DNS) || j_DNS->valuestring == NULL) {
			STM_LOG("Invalid DNS parameter");
			paramError = true;
		} else {
			// В оригинальном коде эта часть закомментирована
			// Просто логируем, что получили DNS, но не обрабатываем его
			STM_LOG("DNS parameter received but not processed");
			/*
			uint8_t dns[4];
			if (parseIPAddress(j_DNS->valuestring, dns)) {
				// settings.dns[0] = dns[0];
				// settings.dns[1] = dns[1];
				// settings.dns[2] = dns[2];
				// settings.dns[3] = dns[3];
			}
			*/
		}
	}

	// Обработка DHCP-флага
	if ((j_setDHCP != NULL) && cJSON_IsTrue(j_setDHCP)) {
		if (j_DHCP == NULL) {
			STM_LOG("Invalid DHCP parameter");
			paramError = true;
		} else {
			settings.DHCPset = cJSON_IsTrue(j_DHCP) ? 1 : 0;
		}
	}

	// Если были ошибки в параметрах, логируем это
	if (paramError) {
		STM_LOG("Errors in network parameters");
	} else {
		STM_LOG("Settings set successful");

		// Сохранение настроек при необходимости
		if (save) {
			STM_LOG("Save settings");
			mem_spi.W25qxx_EraseSector(0);
			osDelay(5);
			mem_spi.Write(settings);
		}
	}
}

void actoin_resp_all_set() {

	cJSON *j_all_settings_obj = cJSON_CreateObject();
	if (!j_all_settings_obj) return;

	cJSON *j_to_host = cJSON_CreateObject();
	if (!j_to_host) return;

	cJSON_AddNumberToObject(j_to_host, "id", ID_CTRL);
	cJSON_AddStringToObject(j_to_host, "name_device", NAME);
	cJSON_AddNumberToObject(j_to_host, "type_data", 3);

	// настройки в строку
	string srtIP_to_host = std::to_string(settings.saveIP.ip[0])+"."+
							std::to_string(settings.saveIP.ip[1])+"."+
							std::to_string(settings.saveIP.ip[2])+ "."+
							std::to_string(settings.saveIP.ip[3]);
	cJSON_AddStringToObject(j_all_settings_obj, "IP", srtIP_to_host.c_str());

	char srtMAC_to_host[100];
	snprintf(srtMAC_to_host, sizeof(srtMAC_to_host),
            "%02x:%02x:%02x:%02x:%02x:%02x",
			settings.MAC[0],settings.MAC[1],settings.MAC[2],
			settings.MAC[3],settings.MAC[4],settings.MAC[5]);

	cJSON_AddStringToObject(j_all_settings_obj, "MAC", srtMAC_to_host);

	string srtGATEWAY_to_host = std::to_string(settings.saveIP.gateway[0])+"."+
							std::to_string(settings.saveIP.gateway[1])+"."+
							std::to_string(settings.saveIP.gateway[2])+"."+
							std::to_string(settings.saveIP.gateway[3]);
	cJSON_AddStringToObject(j_all_settings_obj, "GATEWAY", srtGATEWAY_to_host.c_str());

	string srtMASK_to_host = std::to_string(settings.saveIP.mask[0])+"."+
							std::to_string(settings.saveIP.mask[1])+"."+
							std::to_string(settings.saveIP.mask[2])+"."+
							std::to_string(settings.saveIP.mask[3]);
	cJSON_AddStringToObject(j_all_settings_obj, "MASK", srtMASK_to_host.c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "DNS", "0.0.0.0");

	if(settings.DHCPset)
	{
		cJSON_AddTrueToObject(j_all_settings_obj, "DHCP");
	}
	else
	{
		cJSON_AddFalseToObject(j_all_settings_obj, "DHCP");
	}

	// настройки мотора
	cJSON_AddStringToObject(j_all_settings_obj, "speed", std::to_string(pMotor->getSpeed()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "start_speed", std::to_string(pMotor->getStartSpeed()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "accel", std::to_string(pMotor->getAcceleration()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "slow", std::to_string(pMotor->getSlowdown()).c_str());

	cJSON_AddStringToObject(j_all_settings_obj, "time_out", std::to_string(pMotor->getTimeOut()).c_str());

	cJSON_AddNumberToObject(j_all_settings_obj, "cw_ccw", pMotor->getStatusDirect());

	cJSON_AddNumberToObject(j_all_settings_obj, "rotation", pMotor->getMode());

	cJSON_AddStringToObject(j_all_settings_obj, "version", std::to_string(settings.version).c_str());


	cJSON_AddItemToObject(j_to_host, "obj", j_all_settings_obj);

	char *str_to_host = cJSON_Print(j_to_host);

	STM_LOG("%s", str_to_host);

	cJSON_free(str_to_host);
	cJSON_Delete(j_to_host);
	//cJSON_Delete(j_all_settings_obj);
}

void actoin_resp_status() {

	cJSON *j_all_settings_obj = cJSON_CreateObject();
	if (!j_all_settings_obj) return;

	cJSON *j_to_host = cJSON_CreateObject();
	if (!j_to_host) return;

	cJSON_AddNumberToObject(j_to_host, "id", ID_CTRL);
	cJSON_AddStringToObject(j_to_host, "name_device", NAME);
	cJSON_AddNumberToObject(j_to_host, "type_data", 4);

	string srt_l_dist = std::to_string(pMotor->getCurrentSteps());
	cJSON_AddStringToObject(j_all_settings_obj, "l_dist", srt_l_dist.c_str());

	switch (pMotor->getStatusRotation()) {
		case statusMotor::ACCEL:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "ACCEL");
			break;
		case statusMotor::BRAKING:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "BRAKING");
			break;
		case statusMotor::MOTION:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "MOTION");
			break;
		case statusMotor::STOPPED:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "STOPPED");
			break;
		default:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_rotation", "");
			break;
	}

	switch (pMotor->getStatusTarget()) {
		case statusTarget_t::inProgress:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "inProgress");
			break;
		case statusTarget_t::errMotion:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "errMotion");
			break;
		case statusTarget_t::errDirection:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "errDirection");
			break;
		case statusTarget_t::finished:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "finished");
			break;
		default:
			cJSON_AddStringToObject(j_all_settings_obj, "stat_target", "");
			break;
	}

	switch (pMotor->get_pos()) {
		case pos_t::D0:
			cJSON_AddStringToObject(j_all_settings_obj, "sensor_pos", "D0");
			break;
		case pos_t::D1:
			cJSON_AddStringToObject(j_all_settings_obj, "sensor_pos", "D1");
			break;
		case pos_t::D_0_1:
			cJSON_AddStringToObject(j_all_settings_obj, "sensor_pos", "D_0_1");
			break;
		default:
			cJSON_AddStringToObject(j_all_settings_obj, "sensor_pos", "");
			break;
	}


	cJSON_AddItemToObject(j_to_host, "obj", j_all_settings_obj);

	char *str_to_host = cJSON_Print(j_to_host);
	//string out = str_to_host;
	//out += '\r';

	//HAL_UART_Transmit(&huart2, (uint8_t*)out.c_str(), out.size(), HAL_MAX_DELAY);
	STM_LOG("%s", str_to_host);

	cJSON_free(str_to_host);
	cJSON_Delete(j_to_host);
}

/******************************************************************************************************
 Handlers
 ******************************************************************************************************/

/* USER CODE END Application */
