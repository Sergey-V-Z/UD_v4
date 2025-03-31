#pragma once
#include "main.h"
#include "cmsis_os.h"
#include "Delay_us_DWT.h"
#include "stdlib.h"

#define DIRECT_CCW HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_RESET);
#define DIRECT_CW HAL_GPIO_WritePin(CW_CCW_GPIO_Port, CW_CCW_Pin, GPIO_PIN_SET);

#define SET_TIM_ARR_AND_PULSE(htim, channel, arr_value) do { \
    __HAL_TIM_SET_AUTORELOAD((htim), (arr_value)); \
    __HAL_TIM_SET_COMPARE((htim), (channel), (arr_value) / 2); \
} while(0)

// Новый enum для состояния драйвера
typedef enum {
    DRIVER_OK = 0,
    DRIVER_ERROR,
    DRIVER_STATUS_UNKNOWN
} driver_status_t;

typedef enum {
    HALF,
    FULL
} step;

typedef enum {
    inProgress = 0,
    finished,
    errMotion,
    errDirection,
    errDriver,  // Добавляем новое состояние ошибки для драйвера
	errLimitSwitch
} statusTarget_t;

typedef enum {
    MOTION,
    STOPPED,
    ACCEL,
    BRAKING,
    ERROR_M   // Добавляем состояние ошибки
} statusMotor;

typedef enum {
    ENCODER,
    HALLSENSOR,
    NON
} fb;

typedef enum {
    OK,
    No_Connect,
    No_Signal
} sensorsERROR;

enum class pos_t {
    D0,
    D_0_1,
    D1
};

class extern_driver {
public:
    extern_driver(settings_t *set, TIM_HandleTypeDef *timCount, TIM_HandleTypeDef *timFreq,
                 uint32_t channelFreq, TIM_HandleTypeDef *timDebounce, TIM_HandleTypeDef *timENC);
    ~extern_driver();

    // Методы для работы с состоянием драйвера
    driver_status_t getDriverStatus();
    uint32_t getLastDriverCheck();
    void checkDriverStatus();
    bool isDriverStatusValid();

    // Методы установки параметров
    void SetDirection(dir direction);
    void SetSpeed(uint32_t percent);
    void SetStartSpeed(uint32_t percent);
    void SetAcceleration(uint32_t StepsINmS);
    void SetSlowdown(uint32_t steps);
    void setTimeOut(uint32_t time);
    //void SetZeroPoint(void);
    void SetMode(mode_rotation_t mod);
    void Parameter_update(void);
    void updateCurrentPosition(uint32_t pos);
    void setPoint(uint32_t point, uint32_t abs_steps);

    // Методы получения параметров
    pos_t get_pos();
    uint32_t getAcceleration();
    uint32_t getSlowdown();
    uint32_t getSpeed();
    uint32_t getStartSpeed();
    uint32_t getTimeOut();
    dir getStatusDirect();
    statusMotor getStatusRotation();
    uint16_t getRPM();
    mode_rotation_t getMode();
    statusTarget_t getStatusTarget();
    //uint32_t getLastDistance();

    // Методы управления движением
    bool start(uint32_t steps, dir d = dir::END_OF_LIST);
    bool startForCall(dir d);

    void stop(statusTarget_t status);
    void slowdown();
    void removeBreak(bool status);
    //void goTo(int steps, dir direct);
    void Init();
    bool Calibration_pool();
    bool limit_switch_pool();
    //void findHome();
    void CallStart();
    //void findHomeStart();

    bool saveCurrentPositionAsPoint(uint32_t point_number);
    uint32_t getCurrentSteps();
    //bool setCurrentPosition(uint32_t position);
    bool gotoPoint(uint32_t point_number);
    bool gotoLSwitch(uint8_t sw_x);
    uint32_t getCurrentPoint() { return settings->points.current_point; }
    uint32_t getTargetPoint() { return settings->points.target_point; }
    bool gotoPosition(uint32_t position);
    bool gotoInfinity();
    uint32_t getMaxPosition() const;
    uint32_t getMinPosition() const;
    bool isCalibrated();

    // Обработчики
    //void StepsHandler(uint32_t steps);
    //void StepsAllHandler(uint32_t steps);
    void SensHandler(uint16_t GPIO_Pin);
    void AccelHandler();
    void StartDebounceTimer(uint16_t GPIO_Pin);
    void HandleDebounceTimeout();
    void handleTimerInterrupt();

    void setupEncoderMovement(uint32_t totalSteps, dir direction);
    void handleEncoderCompare(uint32_t channel);
    void updateGlobalPosition();
    void updateEncoderGlobalPosition();// Периодическое обновление глобальной позиции во время движения

private:
    // Для отслеживания абсолютной позиции
    int32_t globalPosition = 0;
    uint32_t targetAbsolutePosition = 0; // Для хранения целевой абсолютной позиции
    const uint16_t ENCODER_MID_VALUE = 0x7FFF; // 32767 (середина 16-битного диапазона)
    const uint16_t ENCODER_MAX_PART = 0x6000;  // Максимальное движение в одной части (~24,000)
    bool isLastEncoderPart = false;            // Флаг последней части
    uint32_t totalRemainingSteps = 0;          // Оставшиеся шаги для движения


    //void updateCurrentSteps(int32_t steps);
    bool validatePointNumber(uint32_t point_number);
    //void updateMotionCounter();
    bool validatePosition(uint32_t position);
    //void calculateTargetDistance(uint32_t position);

    void InitTim();
    double map(double x, double in_min, double in_max, double out_min, double out_max);
    bool waitForStop(uint32_t timeout_ms);
    void ChangeTimerMode(TIM_HandleTypeDef *htim, uint32_t Mode);

    uint32_t calculateBrakingDistance(uint32_t currentSpeed);
    double calculateAccelStep(double progress);

    void calculateTimerFrequency();// Метод для расчета частоты тактирования таймера

    uint16_t map_PercentFromARR(uint16_t arr_value);
    uint16_t map_ARRFromPercent(uint16_t percent_value);
    /**
     * Проверка правильности направления движения по энкодеру
     * Останавливает двигатель при обнаружении неверного направления
     */
    void checkEncoderDirection();

    /**
     * Проверка наличия движения по энкодеру
     * Запускает таймер при отсутствии движения
     */
    void checkEncoderMotion();

    // Методы для работы с таблицами разгона и торможения
    void calculateAccelTable(uint32_t accelSteps);   // Расчет таблицы разгона
    void calculateDecelTable(uint32_t brakingSteps);   // Расчет таблицы торможения
    uint32_t calculateBrakingSteps();  // Расчет количества шагов для торможения
    uint32_t calculateAccelSteps();

    // Параметры драйвера
    driver_status_t currentDriverStatus;
    uint32_t lastDriverCheckTime;
    const uint32_t DRIVER_STATUS_VALIDITY_TIME = 1000; // мс
    GPIO_TypeDef* driverErrorPort;
    uint16_t driverErrorPin;

    // Основные параметры
    settings_t *settings;
    TIM_HandleTypeDef *TimCountAllSteps;
    TIM_HandleTypeDef *TimFrequencies;
    uint32_t ChannelClock;
    TIM_HandleTypeDef* debounceTimer;
    TIM_HandleTypeDef *TimEncoder;

    const uint32_t DEBOUNCE_TIMEOUT = 50;
    bool ignore_sensors = false;
    bool is_start_ignore_timer = false;
    volatile bool d0_debounce_active = false;
    volatile bool d1_debounce_active = false;

    // Параметры движения
    uint32_t MaxSpeed = 1;
    uint32_t MinSpeed = 20000;
    uint32_t Time = 0;
    uint8_t TimerIsStart = false;
    uint32_t PrevCounterENC = 0;
    uint8_t countErrDir = 3;
    uint32_t CallSteps = 0;
    //uint32_t LastDistance = 0;
    uint32_t motionSteps = 0;
    uint32_t Speed_Call = 0;
    uint32_t Speed_temp = 0;

    // Состояния
    statusMotor Status = statusMotor::STOPPED;
    statusTarget_t StatusTarget = statusTarget_t::finished;
    fb FeedbackType = fb::NON;

    // Параметры позиционирования
    const uint32_t START_VIBRATION_TIMEOUT = 30;
    uint32_t vibration_start_time = 0;
    pos_t position = pos_t::D_0_1;
    pos_t target = pos_t::D_0_1;
    uint32_t watchdog = 10000;
    bool permission_calibrate = false;
    bool permission_findHome = false;
    bool change_pos = false;
    uint32_t time = 0;
    uint8_t bos_bit = 0;
    uint32_t timerTickFreq;  // Частота тактирования таймера с учетом предделителя

    // Максимальное количество шагов в таблице разгона/торможения
    static const uint16_t MAX_RAMP_STEPS = 1000;

    // Структура для таблиц разгона и торможения
    struct {
        uint32_t accelTable[MAX_RAMP_STEPS];  // Таблица значений ARR для разгона
        uint32_t decelTable[MAX_RAMP_STEPS];  // Таблица значений ARR для торможения
        uint16_t accelSteps;                  // Количество шагов в таблице разгона
        uint16_t decelSteps;                  // Количество шагов в таблице торможения
        uint16_t currentStep;                 // Текущий шаг в таблице (для разгона или торможения)
    } rampTables;

};
