/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cJSON.h"
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

#define DBG_PORT huart2
#define LOG_TX_BUF_SIZE 2048

#define CURENT_VERSION 47
#define ID_CTRL 2
#define NAME "motor controller"

#define LWIP_DHCP 1

// Команды API для работы с точками (добавить в enum)
#define CMD_SAVE_POINT     20  // Сохранить текущую позицию как точку
#define CMD_GET_POSITION   21  // Получить текущую позицию в шагах
#define CMD_GOTO_SW0   		22  // Установить текущую позицию
#define CMD_GOTO_SW1   		23  // Установить текущую позицию
#define CMD_GOTO_POINT     24  // Перейти на точку
#define CMD_GET_POINT     25  // Получить позицию в шагах из точки
#define CMD_GOTO_POSITION   27  // Перейти на позицию в шагах
#define CMD_GET_MAX_POSITION 28 // Получить максимальную позицию
#define CMD_GET_MIN_POSITION 29 // Получить минимальную позицию
#define CMD_SET_POINT 30 // Получить минимальную позицию

#define MAX_POINTS			10

// Структура ответа на запрос позиции
struct position_response_t {
    uint32_t current_steps;    // Текущая позиция в шагах
    uint32_t current_point;    // Номер текущей точки
    uint8_t is_calibrated;        // Статус калибровки
};

// Структура для работы с точками
struct points_response_t {
    uint32_t points[10];       // Массив точек
    uint32_t count;            // Количество точек
};

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void STM_LOG(const char* format, ...);

void ledBlink();
//double map(double x, double in_min, double in_max, double out_min, double out_max);
typedef enum dir
{
    CW = 0,
    CCW = 1,
    END_OF_LIST = 3
}dir;

typedef enum motor_t
{
	stepper_motor = 0,
    bldc = 1
}motor_t;

// Режимы работы
typedef enum mode_rotation_t {
    bldc_inf = 0,         // Бесконечное вращение BLDC
	bldc_limit = 1,           // Бесконечное вращение без энкодера

    step_inf = 2,       // По счетчику с таймера

	//step_by_meter_timer_limit = 3,  // По концевикам с таймером
	//step_by_meter_enc_limit = 4,    // По концевикам с энкодером

	step_by_meter_timer_intermediate = 3,   // С промежуточными остановками по таймеру
	step_by_meter_enc_intermediate = 4,      // С промежуточными остановками по энкодеру

	calibration_timer = 5,							//режим калибровки
	calibration_enc = 6							//режим калибровки
} mode_rotation_t;

typedef struct
{
	uint8_t 	ip[4];// = {192, 168, 0, 2};
	uint8_t		mask[4];//  = {255, 255, 255, 0};
	uint8_t 	gateway[4];// = {192, 168, 0, 1};
}setIP_t;

typedef struct inMessageParam_t
{
	uint16_t 	Start_data;
	uint16_t	Size;
}inMessageParam_t;

// Карта датчиков
typedef struct sensors_map_t {
    uint16_t CW_sensor;     // GPIO_Pin датчика по часовой
    uint16_t CCW_sensor;    // GPIO_Pin датчика против часовой
    uint8_t detected;       // флаг успешного определения датчиков
    uint8_t error_state;    // состояние ошибки датчиков
} sensors_map_t;

// Основные настройки
typedef struct {
    // Сетевые параметры
    uint8_t MAC[6];
    setIP_t saveIP;
    uint8_t DHCPset;

    // Параметры движения
    dir Direct;                  // направление вращения
    mode_rotation_t mod_rotation;// режим вращения
    motor_t res2;              // тип мотора
    uint32_t Speed;             // скорость
    uint32_t StartSpeed;        // начальная скорость
    uint32_t Accel;             // ускорение шагов в милисекунду
    uint32_t Slowdown;          // торможение шагов
    uint32_t res1;
    uint32_t res;

    // Параметры энкодера
    uint32_t stepsENC;          // шаги энкодера между датчиками
    uint32_t stepsENCtoOneStepMotor; // шаги энкодера на шаг мотора

    // Временные параметры
    uint32_t TimeOut;           // таймаут при отсутствии движения

    // Конфигурация датчиков
    sensors_map_t sensors_map;

    // Промежуточные позиции
    uint32_t intermediate_positions[3];  // позиции промежуточных остановок
    uint8_t use_intermediate;            // флаг использования промежуточных остановок

    struct points_map {
        uint32_t points[10];     // массив точек
        uint32_t count;          // количество точек
        uint32_t target_point;   // целевая точка
        uint32_t current_point;  // текущая точка
        uint8_t is_calibrated;      // флаг калибровки
    } points;

    // Системные параметры
    uint8_t version;            // версия конфигурации

} settings_t;


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAC_IP_Pin_Pin GPIO_PIN_2
#define MAC_IP_Pin_GPIO_Port GPIOE
#define R_Pin GPIO_PIN_13
#define R_GPIO_Port GPIOC
#define G_Pin GPIO_PIN_14
#define G_GPIO_Port GPIOC
#define B_Pin GPIO_PIN_15
#define B_GPIO_Port GPIOC
#define eth_RST_Pin GPIO_PIN_0
#define eth_RST_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_8
#define EN_GPIO_Port GPIOE
#define CW_CCW_Pin GPIO_PIN_10
#define CW_CCW_GPIO_Port GPIOE
#define D0_Pin GPIO_PIN_13
#define D0_GPIO_Port GPIOD
#define D0_EXTI_IRQn EXTI15_10_IRQn
#define D1_Pin GPIO_PIN_14
#define D1_GPIO_Port GPIOD
#define D1_EXTI_IRQn EXTI15_10_IRQn
#define DRIVER_ERR_Pin GPIO_PIN_15
#define DRIVER_ERR_GPIO_Port GPIOD
#define DRIVER_ERR_EXTI_IRQn EXTI15_10_IRQn
#define MAC_b7_Pin GPIO_PIN_6
#define MAC_b7_GPIO_Port GPIOC
#define MAC_b6_Pin GPIO_PIN_7
#define MAC_b6_GPIO_Port GPIOC
#define MAC_b5_Pin GPIO_PIN_8
#define MAC_b5_GPIO_Port GPIOC
#define MAC_b4_Pin GPIO_PIN_9
#define MAC_b4_GPIO_Port GPIOC
#define MAC_b3_Pin GPIO_PIN_8
#define MAC_b3_GPIO_Port GPIOA
#define MAC_b2_Pin GPIO_PIN_9
#define MAC_b2_GPIO_Port GPIOA
#define MAC_b1_Pin GPIO_PIN_10
#define MAC_b1_GPIO_Port GPIOA
#define MAC_b0_Pin GPIO_PIN_11
#define MAC_b0_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define WP_Pin GPIO_PIN_0
#define WP_GPIO_Port GPIOD
#define HOLD_Pin GPIO_PIN_1
#define HOLD_GPIO_Port GPIOD
#define enc_G_Pin GPIO_PIN_2
#define enc_G_GPIO_Port GPIOD
#define enc_notG_Pin GPIO_PIN_3
#define enc_notG_GPIO_Port GPIOD
#define enc_Z_in_Pin GPIO_PIN_3
#define enc_Z_in_GPIO_Port GPIOB
#define enc_Z_in_EXTI_IRQn EXTI3_IRQn

/* USER CODE BEGIN Private defines */
#define UART2_RX_LENGTH 512
#define message_RX_LENGTH 512

// Максимальный размер сообщения
#define MAX_MESSAGE_SIZE 512
#define QUEUE_SIZE 12

// Структура сообщения
typedef struct {
    char data[MAX_MESSAGE_SIZE];
    uint16_t length;
} LogMessage_t;

// Структура логгера
typedef struct {
    UART_HandleTypeDef* huart;
    osMessageQId messageQueue;
    char* txBuffer;
    volatile uint8_t isTransmitting;
    uint8_t started;
} UartLogger_t;

// Функции инициализации и работы с логгером
void Logger_Init(UART_HandleTypeDef* huart);
void Logger_Process(void);
void Logger_TxCpltCallback(void);
void Logger_Log(const char* format, ...);

// Запуск задачи логгера
void Logger_StartTask(void);

// Макрос для логирования
#define STM_LOG(...) Logger_Log(__VA_ARGS__)

// Глобальный экземпляр логгера
extern UartLogger_t logger;

// Локальный буфер для DMA передачи
extern char txBuffer[MAX_MESSAGE_SIZE];
// Пул сообщений и буфер для него
extern LogMessage_t messagePool[QUEUE_SIZE];
extern uint8_t messagePoolUsed[QUEUE_SIZE];
extern osMutexId poolMutexHandle;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
