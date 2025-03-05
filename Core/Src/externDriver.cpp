#include <externDriver.hpp>
#include "stdio.h"
#include <cmath>
#include <algorithm>

// Constants for hyperbolic acceleration
const double ACCEL_SCALE = 15.0;   // Увеличен масштабный коэффициент для более быстрого ускорения
const double ACCEL_OFFSET = 0.3;   // Уменьшено смещение для более крутой кривой
const int ACCEL_STEPS = 70;        // Уменьшено количество шагов для более короткого времени разгона

// Calculate acceleration step for given progress (0-1)
double extern_driver::calculateAccelStep(double progress) {
    // Гиперболическая кривая от MinSpeed до settings->Speed
    // Модифицированная для быстрого начального ускорения с сохранением гиперболического характера

    // Используем кубический корень для более быстрого начального прогресса
    double modifiedProgress = pow(progress, 0.33);

    // Инвертируем прогресс для гиперболы
    double invertedProgress = 1.0 - modifiedProgress;

    // Более крутая гиперболическая функция с большим коэффициентом
    // Добавляем более значительный линейный компонент для гарантированного ускорения
    double factor = (ACCEL_SCALE / (invertedProgress + ACCEL_OFFSET)) + (progress * 2.0);

    // Рассчитываем граничные значения для нормализации
    double maxFactor = (ACCEL_SCALE / (0.0 + ACCEL_OFFSET)) + 0.0;
    double minFactor = (ACCEL_SCALE / (1.0 + ACCEL_OFFSET)) + 2.0; // Учитываем измененный линейный компонент
    double normalizedFactor = (factor - minFactor) / (maxFactor - minFactor);

    // Ограничиваем фактор диапазоном [0,1] с дополнительным ускорением
    normalizedFactor = (normalizedFactor < 0) ? 0 : ((normalizedFactor > 1) ? 1 : normalizedFactor);

    // Дополнительно ускоряем начальную фазу
    if (progress < 0.3) {
        normalizedFactor = normalizedFactor * 2.0;
        if (normalizedFactor > 1.0) normalizedFactor = 1.0;
    }

    // Вычисляем период для текущего шага
    double speed_range = MinSpeed - settings->Speed;

    // Возвращаем период, учитывая направление (от MinSpeed к settings->Speed)
    return static_cast<uint32_t>(MinSpeed - speed_range * normalizedFactor);
}

// Calculate required braking distance based on current speed
uint32_t extern_driver::calculateBrakingDistance(uint32_t currentPeriod) {
    // When working with periods, lower period means higher frequency/speed
    // Convert periods to frequencies for calculation
    double f_initial = 1000000.0 / currentPeriod;  // Hz, assuming period in μs
    double f_final = 1000000.0 / MinSpeed;         // Hz, target slow speed

    // Calculate distance based on deceleration rate
    // s = (v_initial^2 - v_final^2) / (2 * deceleration)
    double v_initial = f_initial / 1000.0;  // Normalize for calculation
    double v_final = f_final / 1000.0;      // Normalize for calculation
    double deceleration = settings->Slowdown / 10000.0;

    // Calculate braking distance with safety margin (20%)
    uint32_t distance = static_cast<uint32_t>(
        (pow(v_initial, 2) - pow(v_final, 2)) / (2 * deceleration) * 1.2
    );

    // Ensure minimum safe distance
    return std::max(distance, (uint32_t)100U);
}

/***************************************************************************
 * Класс для шагового двухфазного мотора
 *
 * В этом классе реализован цикл управления и контроля шагового двигателя
 ****************************************************************************/

void extern_driver::Init() {
    currentDriverStatus = DRIVER_STATUS_UNKNOWN;
    lastDriverCheckTime = 0;
    driverErrorPort = DRIVER_ERR_GPIO_Port;
    driverErrorPin = DRIVER_ERR_Pin;

 	switch (settings->mod_rotation) {
		case step_by_meter_enc_intermediate:
		case step_by_meter_timer_intermediate:
		case step_inf:
		{
            TimFrequencies->Instance->PSC = 399;
            MaxSpeed = 50;
            MinSpeed = 13000;
			break;
		}
		case bldc_limit:
    	case bldc_inf:
		{
            TimFrequencies->Instance->PSC = 200-1;
            MaxSpeed = 100;
            MinSpeed = 2666;
			break;
		}
		default:
		{
            TimFrequencies->Instance->PSC = 399;
            MaxSpeed = 50;
            MinSpeed = 13000;
			break;
		}
 	}


    // Рассчитываем частоту тактирования таймера
    calculateTimerFrequency();

    TimFrequencies->Instance->ARR = MinSpeed;
    //Speed_Call = (uint16_t) map(950, 1, 1000, MinSpeed, MaxSpeed);
    Speed_Call = (uint16_t) map_ARRFromPercent(950);

    // Инициализация переменных для работы с таблицами
    rampTables.accelSteps = 0;
    rampTables.decelSteps = 0;
    rampTables.currentStep = 0;

    // Рассчитываем количество шагов, необходимых для торможения
    uint32_t brakingSteps = calculateBrakingSteps();
    uint32_t accelSteps = calculateAccelSteps();
    // Пересчитываем таблицы разгона и торможения
    calculateAccelTable(accelSteps);
    calculateDecelTable(brakingSteps);

    Status = statusMotor::STOPPED;
    FeedbackType = fb::ENCODER;
    StatusTarget = statusTarget_t::finished;

    if (settings->Direct == dir::CW) {
        DIRECT_CW
    } else if (settings->Direct == dir::CCW) {
        DIRECT_CCW
    }

    // Инициализация таймера энкодера
    __HAL_TIM_SET_COUNTER(TimEncoder, 0);
    HAL_TIM_Encoder_Start(TimEncoder, TIM_CHANNEL_ALL);
    HAL_TIM_OC_Start_IT(TimEncoder, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(TimEncoder, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(TimEncoder);

    // Инициализация таймера шагов
    __HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
    //HAL_TIM_Base_Start_IT(TimCountAllSteps);
    //HAL_TIM_Encoder_Start(TimEncoder, TIM_CHANNEL_ALL);
    HAL_TIM_OC_Start_IT(TimCountAllSteps, TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(TimCountAllSteps, TIM_CHANNEL_2);

    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

    checkDriverStatus();
    Parameter_update();
}

driver_status_t extern_driver::getDriverStatus() {
    if (!isDriverStatusValid()) {
        checkDriverStatus();
    }
    return currentDriverStatus;
}

uint32_t extern_driver::getLastDriverCheck() {
    return lastDriverCheckTime;
}

void extern_driver::checkDriverStatus() {
    GPIO_PinState errorState = HAL_GPIO_ReadPin(driverErrorPort, driverErrorPin);
    lastDriverCheckTime = HAL_GetTick();

    if (errorState == GPIO_PIN_SET) {
        currentDriverStatus = DRIVER_ERROR;
        if (Status != statusMotor::ERROR_M) {
            stop(statusTarget_t::errDriver);
            Status = statusMotor::ERROR_M;
        }
    } else {
        currentDriverStatus = DRIVER_OK;
    }
}

bool extern_driver::isDriverStatusValid() {
    return (HAL_GetTick() - lastDriverCheckTime) < DRIVER_STATUS_VALIDITY_TIME;
}

//methods for aktion*********************************************

//
bool extern_driver::start(uint32_t steps, dir d) {

	dir temp_diretion = settings->Direct;
    if (Status == statusMotor::STOPPED) {
        // Проверяем состояние драйвера перед стартом
        checkDriverStatus();
        if (currentDriverStatus != DRIVER_OK) {
            STM_LOG("Cannot start: driver error detected");
            return false;
        }

     	// Направление
     	if(d ==  END_OF_LIST)
     	{
     		temp_diretion = settings->Direct;
     	} else {
     		temp_diretion = d;
     	}
        // проверка по концевикам выставление направления
     	switch (settings->mod_rotation) {
    		case step_by_meter_enc_intermediate:
    		case step_by_meter_timer_intermediate:
    		case calibration_timer:
    		case calibration_enc:
    		{

    			if (temp_diretion == dir::CW && HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin)) {
    				STM_LOG("Cannot move CW: at CW limit switch");
    				return false;
    			}
    			else if (temp_diretion == dir::CCW && HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)) {
    				STM_LOG("Cannot move CCW: at CCW limit switch");
    				return false;
    			}

    			if (temp_diretion == dir::CCW) {
    				DIRECT_CCW
    				ChangeTimerMode(TimCountAllSteps, TIM_COUNTERMODE_UP); //от  0 датчика к 1 CCW
    			} else {
    				DIRECT_CW
    				ChangeTimerMode(TimCountAllSteps, TIM_COUNTERMODE_DOWN); //от  1 датчика к 0 CW
    			}
    			break;
    		}
    		case step_inf:
    		case bldc_inf:
    		{
    			// добавить отключение таймеров обратной связи

    			if (temp_diretion == dir::CCW) {
    				DIRECT_CCW
    			} else {
    				DIRECT_CW
    			}
    			break;
    		}
    		case bldc_limit:
    		{
    			// добавить отключение таймеров обратной связи

    			if (temp_diretion == dir::CW && HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin)) {
    				STM_LOG("Cannot move CW: at CW limit switch");
    				return false;
    			}
    			else if (temp_diretion == dir::CCW && HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)) {
    				STM_LOG("Cannot move CCW: at CCW limit switch");
    				return false;
    			}
    			if (temp_diretion == dir::CCW) {
    				DIRECT_CCW
    			} else {
    				DIRECT_CW
    			}
    			break;
    		}
    		default:
    		{
    			break;
    		}
        }

        ignore_sensors = true;
        vibration_start_time = HAL_GetTick();

        PrevCounterENC = TimEncoder->Instance->CNT;
        countErrDir = 3;
        StatusTarget = statusTarget_t::inProgress;

        //uint32_t SlowdownDistance = 50; //steps/10; //1%

        // Рассчитываем количество шагов, необходимых для торможения
        uint32_t brakingSteps = calculateBrakingSteps();
        uint32_t accelSteps = calculateAccelSteps();
        // Пересчитываем таблицы разгона и торможения
        calculateAccelTable(accelSteps);
        calculateDecelTable(brakingSteps);

        // Сбрасываем счетчик шагов таблицы
        rampTables.currentStep = 0;
        //rampTables.isAccelerating = true;
        //rampTables.isDecelerating = false;

        switch (settings->mod_rotation) {
        	case calibration_enc:
        	case step_by_meter_enc_intermediate:
        	//case step_by_meter_enc_limit:
        	{
        		//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, rampTables.accelTable[0]);
        		//__HAL_TIM_SET_COMPARE(TimFrequencies, ChannelClock, rampTables.accelTable[0] / 2);
        		SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, rampTables.accelTable[0]);

        		// обработать направление. если двигаться в CCW 0->1 то все ок но если двигаться CW 1->0 то шаги для торможения нужно прибавить
				__HAL_TIM_SET_AUTORELOAD(TimEncoder, 0xffff);
        		if(settings->Direct == dir::CCW){
        			__HAL_TIM_SET_COMPARE(TimEncoder, TIM_CHANNEL_3, steps - brakingSteps);
        		} else {
        			__HAL_TIM_SET_COMPARE(TimEncoder, TIM_CHANNEL_3, steps + brakingSteps);
        		}
				__HAL_TIM_SET_COMPARE(TimEncoder, TIM_CHANNEL_4, steps);
        		Status = statusMotor::ACCEL;
        		break;
        	}
        	case step_inf:
        	case calibration_timer:
        	case step_by_meter_timer_intermediate:
        	//case step_by_meter_timer_limit:
        	{
        		//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, rampTables.accelTable[0]);
        		SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, rampTables.accelTable[0]);

				//__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
				__HAL_TIM_SET_AUTORELOAD(TimCountAllSteps, 0xffff);
        		if(settings->Direct == dir::CCW){
        			__HAL_TIM_SET_COMPARE(TimCountAllSteps, TIM_CHANNEL_1, steps - brakingSteps);
        		} else {
        			__HAL_TIM_SET_COMPARE(TimCountAllSteps, TIM_CHANNEL_1, steps + brakingSteps);
        		}
				__HAL_TIM_SET_COMPARE(TimCountAllSteps, TIM_CHANNEL_2, steps);
        		Status = statusMotor::ACCEL;
        		break;
        	}

        	case bldc_limit:
        	case bldc_inf:
        	{
        		//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, settings->Speed);
        		SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, settings->Speed);
        		Status = statusMotor::MOTION;
        		break;
        	}
        	default:
        	{
        		break;
        	}
        }

        //STM_LOG("Speed start: %d", (int)(TimFrequencies->Instance->ARR));
        STM_LOG("Start motor.");

        HAL_TIM_OC_Start_IT(TimFrequencies, ChannelClock);

        return true;
    } else {
        STM_LOG("Fail started motor.");
        return false;
    }
}

bool extern_driver::startForCall(dir d) {
	STM_LOG("Start for call. status: %d, dir: %s", (int)Status, d == dir::CW ? "CW" : "CCW");

    if ((Status == statusMotor::STOPPED) ||
    		((settings->mod_rotation == mode_rotation_t::calibration_enc) && (settings->mod_rotation == mode_rotation_t::calibration_timer)))
    {

        // Сбрасываем счетчик шагов таблицы
        rampTables.currentStep = 0;

    	// настроим и запустим двигатель
    	settings->Direct = d;

        // Проверяем состояние драйвера перед стартом
        checkDriverStatus();
        if (currentDriverStatus != DRIVER_OK) {
            STM_LOG("Cannot start: driver error detected");
            return false;
        }

		if (settings->Direct == dir::CW && HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin)) {
			STM_LOG("Cannot move CW: at CW limit switch");
			return false;
		}
		else if (settings->Direct == dir::CCW && HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)) {
			STM_LOG("Cannot move CCW: at CCW limit switch");
			return false;
		} else {

		}

        ignore_sensors = true;
        vibration_start_time = HAL_GetTick();
        StatusTarget = statusTarget_t::inProgress;


		if (settings->Direct == dir::CCW) {
			DIRECT_CCW
		} else {
			DIRECT_CW
		}

        // Рассчитываем количество шагов, необходимых для торможения
        uint32_t brakingSteps = calculateBrakingSteps();
        uint32_t accelSteps = calculateAccelSteps();
        // Пересчитываем таблицы разгона и торможения
        calculateAccelTable(accelSteps);
        calculateDecelTable(brakingSteps);

		//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, rampTables.accelTable[0]);
		SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, rampTables.accelTable[0]);

		if(settings->mod_rotation == mode_rotation_t::calibration_timer)
		{
			// настройка лимитов
			__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
			__HAL_TIM_SET_AUTORELOAD(TimCountAllSteps, 0xffff);
			__HAL_TIM_SET_COMPARE(TimCountAllSteps, TIM_CHANNEL_1, 0xffff);
			__HAL_TIM_SET_COMPARE(TimCountAllSteps, TIM_CHANNEL_2, 0xffff);
		} else {
			__HAL_TIM_SET_COUNTER(TimEncoder, 0);
			__HAL_TIM_SET_AUTORELOAD(TimEncoder, 0xffff);
			__HAL_TIM_SET_COMPARE(TimEncoder, TIM_CHANNEL_3, 0xffff);
			__HAL_TIM_SET_COMPARE(TimEncoder, TIM_CHANNEL_4, 0xffff);
		}

        Status = statusMotor::ACCEL;
        HAL_TIM_OC_Start_IT(TimFrequencies, ChannelClock);

        return true;
    } else {
        STM_LOG("Fail started motor. motion or mode");
        return false;
    }
}

void extern_driver::stop(statusTarget_t status) {
    ignore_sensors = false;

    HAL_TIM_OC_Stop_IT(TimFrequencies, ChannelClock);
    //permission_calibrate = false;
    //permission_findHome = false;

    switch (status) {
        case statusTarget_t::finished:
            break;
        case statusTarget_t::errMotion:
            break;
        case statusTarget_t::errDirection:
            break;
        case statusTarget_t::errDriver:
            STM_LOG("Motor stopped: driver error detected");
            break;
        default:
            break;
    }

    StatusTarget = status;
    TimerIsStart = false;
    Time = 0;
/*
    switch (settings->mod_rotation) {
    	case by_meter_timer_intermediate:
		case by_meter_timer_limit_switch:
		case by_meter_timer:
		{

		    if (settings->Direct == dir::CCW) {
		            LastDistance = motionSteps + TimCountAllSteps->Instance->CNT;
		            motionSteps = 0;
		    } else {
		            LastDistance = motionSteps + TimCountAllSteps->Instance->CNT;
		            motionSteps = 0;
		    }
			break;
		}
		case infinity_enc:
        case by_meter_enc_intermediate:
        case by_meter_enc_limit_switch:
		case by_meter_enc:
		{

		    if (settings->Direct == dir::CCW) {
		            LastDistance = TimEncoder->Instance->CNT;
		    } else {
		            LastDistance = 0xffff - TimEncoder->Instance->CNT;
		    }
			break;
		}
		default:
		{
			break;
		}
    }
    */
    Status = statusMotor::STOPPED;
}

/**
 * Функция для начала торможения двигателя.
 * Инициирует процесс торможения в зависимости от режима работы.
 */
// Модификация метода slowdown() для использования таблицы торможения
void extern_driver::slowdown() {
    // Если двигатель уже тормозит или остановлен, ничего не делаем
    if (Status == statusMotor::BRAKING || Status == statusMotor::STOPPED) return;

    // Сбрасываем счетчик шагов таблицы
    rampTables.currentStep = 0;
    //rampTables.isAccelerating = false;
    //rampTables.isDecelerating = true;

    // Устанавливаем статус торможения
    Status = statusMotor::BRAKING;

    STM_LOG("Motor slowdown with deceleration table (%u steps)", rampTables.decelSteps);
}

void extern_driver::removeBreak(bool status) {
	if (status) {
		//      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_SET);
	} else {
		//      HAL_GPIO_WritePin(RELE1_GPIO_Port, RELE1_Pin, GPIO_PIN_RESET);
	}
}

//handlers*******************************************************

void extern_driver::SensHandler(uint16_t GPIO_Pin) {

    // Если включено игнорирование датчиков, проверяем не истек ли таймаут
    if (ignore_sensors) {
        if ((HAL_GetTick() - vibration_start_time) < START_VIBRATION_TIMEOUT) {
            // Игнорируем прерывание
            return;
        } else {
            // Таймаут истек, выключаем игнорирование
            ignore_sensors = false;
            position = pos_t::D_0_1;
        }
    }

    stop(statusTarget_t::finished);

	if (GPIO_Pin == D0_Pin) {
		position = pos_t::D0;

        switch (settings->mod_rotation) {

        	case calibration_timer:
        	case calibration_enc:
        	{
        		break;
        	}
        	case step_by_meter_enc_intermediate:
        	case step_by_meter_timer_intermediate:
        	case bldc_limit:
        	case step_inf:
        	case bldc_inf:
        	{
        		if(settings->points.is_calibrated){
					updateCurrentPosition(0); //
					settings->points.current_point = 0;
        		}

        		break;
        	}
        	default:
        	{
        		break;
        	}
        }
	}
	else if (GPIO_Pin == D1_Pin){
		position = pos_t::D1;

        switch (settings->mod_rotation) {

        	case calibration_timer:
        	case calibration_enc:
        	{
        		break;
        	}
        	case step_by_meter_enc_intermediate:
        	case step_by_meter_timer_intermediate:
        	case bldc_limit:
        	case step_inf:
        	case bldc_inf:
        	{
        		if(settings->points.is_calibrated){
					//updateCurrentPosition(CallSteps); //
					settings->points.current_point = 9;
        		}

        		break;
        	}
        	default:
        	{
        		break;
        	}
        }
	}
	else {
		position = pos_t::D_0_1;
	}

}

/**
 * Обработчик разгона и торможения двигателя.
 * Управляет ускорением, замедлением и проверяет корректность движения в зависимости от режима работы.
 */
/*void extern_driver::AccelHandler() {
    static uint32_t accel_step = 0;
    static uint32_t lastSpeedUpdate = 0;
    uint32_t currentTime = HAL_GetTick();

    switch (Status) {
        case statusMotor::ACCEL: {
            if (currentTime - lastSpeedUpdate >= 10) {  // Update every 10ms
                lastSpeedUpdate = currentTime;

                if (permission_calibrate) {
                    // Use exponential acceleration to Speed_Call
                    double progress = static_cast<double>(accel_step) / ACCEL_STEPS;
                    uint32_t newSpeed = static_cast<uint32_t>(calculateAccelStep(progress));

                    if (newSpeed >= settings->Speed) {
                        TimFrequencies->Instance->ARR = newSpeed;
                        accel_step++;
                    } else {
                        TimFrequencies->Instance->ARR = settings->Speed;
                        Status = statusMotor::MOTION;
                        accel_step = 0;
                    }
                } else {
                    // Normal operation - accelerate to settings->Speed
                    double progress = static_cast<double>(accel_step) / ACCEL_STEPS;
                    uint32_t newSpeed = static_cast<uint32_t>(calculateAccelStep(progress));

                    if (newSpeed >= settings->Speed) {
                        TimFrequencies->Instance->ARR = newSpeed;
                        accel_step++;
                    } else {
                        TimFrequencies->Instance->ARR = settings->Speed;
                        Status = statusMotor::MOTION;
                        accel_step = 0;

                        // Calculate and update braking distance
                        uint32_t requiredBrakingDistance = calculateBrakingDistance(settings->Speed);
                        settings->SlowdownDistance = requiredBrakingDistance;
                    }
                }
            }
            break;
        }

        case statusMotor::BRAKING: {
            // Линейное торможение с автоматическим расчётом параметров
            // Для периодов: значение ARR должно увеличиваться для замедления
            uint32_t current_period = TimFrequencies->Instance->ARR;
            uint32_t next_period = current_period + settings->Slowdown;

            if (next_period <= MinSpeed) {
                // Постепенно увеличиваем период (замедляем)
                TimFrequencies->Instance->ARR = next_period;
            } else {
                // Достигли минимальной скорости
                TimFrequencies->Instance->ARR = MinSpeed;

                // Проверяем режим работы для дополнительных действий
                switch (settings->mod_rotation) {
                    case mode_rotation_t::step_inf:
                    case mode_rotation_t::bldc_inf:
                        // Для бесконечных режимов полная остановка
                        stop(statusTarget_t::finished);
                        break;

                    default:
                        // Для режимов с позиционированием продолжаем на минимальной скорости
                        // до достижения целевой позиции
                        TimFrequencies->Instance->ARR = MinSpeed;
                        break;
                }
            }
            break;
        }

        default:
            break;
    }

    // Motion checks based on mode
    switch (settings->mod_rotation) {
        case mode_rotation_t::step_by_meter_enc_intermediate:
        case mode_rotation_t::step_by_meter_enc_limit:
            // Check encoder feedback
            if ((Status == statusMotor::MOTION) || (Status == statusMotor::BRAKING)) {
                checkEncoderMotion();
            }
            // Fallthrough intentional

        case mode_rotation_t::step_by_meter_timer_intermediate:
        case mode_rotation_t::step_by_meter_timer_limit:
            // Handle timeout
            if (TimerIsStart) {
                Time++;
                if (Time >= settings->TimeOut) {
                    TimerIsStart = false;
                    Time = 0;
                    stop(statusTarget_t::errMotion);
                }
            }
            break;

        default:
            break;
    }
}*/

/**
 * @brief Обработчик ускорения с улучшенным профилем для быстрого старта
 */
//#define MAX_STEPS settings->Accel
//#define MAX_STEPS_SLOW settings->

void extern_driver::AccelHandler() {
    static uint16_t step_counter = 0;
    //const uint16_t MAX_STEPS = 500; // Уменьшаем общее число шагов для более быстрого разгона

    switch (Status) {
        case statusMotor::ACCEL: {
            uint16_t start_arr = MinSpeed; // Начальное значение ARR
            uint16_t target_arr = settings->Speed;

            if (step_counter < settings->Accel) {
                // Процент прохождения пути (от 0.0 до 1.0)
                float progress = (float)step_counter / settings->Accel;

                // Используем кубический корень для более быстрого начального ускорения
                // и более плавного завершения
                float factor = 1.0f - powf(progress, 0.33f);

                // Вычисляем новое значение ARR
                uint16_t new_arr = target_arr + (uint16_t)((start_arr - target_arr) * factor);

                // Защита от выхода за границы диапазона
                if (new_arr < target_arr) new_arr = target_arr;
                if (new_arr > start_arr) new_arr = start_arr;

                // Устанавливаем новое значение
                TimFrequencies->Instance->ARR = new_arr;

                // Увеличиваем счетчик шагов
                step_counter++;
            } else {
                // Достигли целевой скорости
                TimFrequencies->Instance->ARR = target_arr;
                Status = statusMotor::MOTION;
                step_counter = 0; // Сбрасываем счетчик для следующего использования
            }
            break;
        }

        case statusMotor::BRAKING: {
            uint16_t start_arr = TimFrequencies->Instance->ARR; // Текущее значение ARR
            uint16_t target_arr = MinSpeed; // Целевое значение ARR (минимальная скорость)

            if (step_counter < settings->Slowdown) {
                // Процент прохождения пути (от 0.0 до 1.0)
                float progress = (float)step_counter / settings->Slowdown;

                // Используем кубическую функцию для более быстрого начального замедления
                float factor = powf(progress, 3.0f);

                // Вычисляем новое значение ARR
                uint16_t new_arr = start_arr + (uint16_t)((target_arr - start_arr) * factor);

                // Защита от выхода за границы диапазона
                if (new_arr < start_arr) new_arr = start_arr;
                if (new_arr > target_arr) new_arr = target_arr;

                // Устанавливаем новое значение
                TimFrequencies->Instance->ARR = new_arr;

                // Увеличиваем счетчик шагов
                step_counter++;
            } else {
                // Достигли минимальной скорости
                TimFrequencies->Instance->ARR = target_arr;
                step_counter = 0; // Сбрасываем счетчик
            }
            break;
        }

        default:
            break;
    }
}

/*void extern_driver::AccelHandler() {
    switch (Status) {
        case statusMotor::ACCEL: {
            if (permission_calibrate) {
                // Режим калибровки - разгон до специальной скорости Speed_Call
                if (((TimFrequencies->Instance->ARR) - settings->Accel) >= settings->Speed) {
                    (TimFrequencies->Instance->ARR) -= settings->Accel;  // Плавное увеличение скорости
                } else {
                    (TimFrequencies->Instance->ARR) = settings->Speed;   // Достигнута целевая скорость
                    Status = statusMotor::MOTION;                        // Переход в режим движения
                }
            } else {
                // Обычный режим работы - разгон до настроенной скорости settings->Speed
                if (((TimFrequencies->Instance->ARR) - settings->Accel) >= settings->Speed) {
                    (TimFrequencies->Instance->ARR) -= settings->Accel;  // Плавное увеличение скорости
                } else {
                    (TimFrequencies->Instance->ARR) = settings->Speed;   // Достигнута целевая скорость
                    Status = statusMotor::MOTION;                        // Переход в режим движения
                }
            }
            break;
        }

        case statusMotor::BRAKING: {
            // Обработка торможения - плавное уменьшение скорости
            if ((TimFrequencies->Instance->ARR) + settings->Slowdown <= MinSpeed) {
                (TimFrequencies->Instance->ARR) += settings->Slowdown;   // Плавное уменьшение скорости
            } else {
            	(TimFrequencies->Instance->ARR) = MinSpeed;
            }
            break;
        }

        default:
            break;
    }

}*/

/**
 * Проверка правильности направления движения по энкодеру
 * Останавливает двигатель при обнаружении неверного направления
 */
void extern_driver::checkEncoderDirection() {
    if (PrevCounterENC != TimEncoder->Instance->CNT) {
        if (settings->Direct == dir::CCW) {
            // Проверка направления для против часовой стрелки
            if ((PrevCounterENC) > TimEncoder->Instance->CNT) {
                if (countErrDir == 0) {
                    stop(statusTarget_t::errDirection);  // Остановка при ошибке направления
                } else {
                    countErrDir--;                      // Уменьшение счетчика ошибок
                }
            }
        } else {
            // Проверка направления для по часовой стрелке
            if ((PrevCounterENC) < TimEncoder->Instance->CNT) {
                if (countErrDir == 0) {
                    stop(statusTarget_t::errDirection);  // Остановка при ошибке направления
                } else {
                    countErrDir--;                      // Уменьшение счетчика ошибок
                }
            }
        }

        PrevCounterENC = TimEncoder->Instance->CNT;    // Обновление предыдущего значения энкодера
        TimerIsStart = false;                          // Сброс таймера
        Time = 0;
    } else {
        TimerIsStart = true;                           // Запуск таймера если нет движения
    }
}

/**
 * Проверка наличия движения по энкодеру
 * Запускает таймер при отсутствии движения
 */
void extern_driver::checkEncoderMotion() {
    // Проверка изменения значения энкодера в допустимых пределах (+/- 100)
    if (((PrevCounterENC + 100) >= TimEncoder->Instance->CNT)
        && (TimEncoder->Instance->CNT >= (PrevCounterENC - 100))) {
        TimerIsStart = true;                           // Запуск таймера при отсутствии движения
    } else {
        PrevCounterENC = TimEncoder->Instance->CNT;    // Обновление предыдущего значения
        TimerIsStart = false;                          // Сброс таймера
        Time = 0;
    }
}

// Калибровка
bool extern_driver::Calibration_pool() {

	bool ret = false;
	mode_rotation_t temp_mode = settings->mod_rotation;

	// доработать калибровку. если один из датчиков вышел из строя и мотор сделает
	// круг и снова поподет на тот же концевик от куда начал то он долже остановится

    if (permission_calibrate && ((settings->mod_rotation != step_inf) && (settings->mod_rotation != bldc_inf))) {
        // Проверяем текущее состояние датчиков
        bool on_D0 = HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET;
        bool on_D1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) == GPIO_PIN_SET;

        settings->points.is_calibrated = false;

        switch (settings->mod_rotation) {

        	case step_by_meter_timer_intermediate:
        	//case step_by_meter_timer_limit:
        	case bldc_limit:
        	case calibration_timer:
        	{
        		settings->mod_rotation =  mode_rotation_t::calibration_timer;
        		break;
        	}
        	case calibration_enc:
        	case step_by_meter_enc_intermediate:
        	//case step_by_meter_enc_limit:
        	{
        		settings->mod_rotation =  mode_rotation_t::calibration_enc;
        		break;
        	}
        	case step_inf:
        	case bldc_inf:
        	default:
        	{
        		break;
        	}
        }
        //STM_LOG("Starting calibration. D0: %d, D1: %d", on_D0, on_D1);

        if(on_D0) {
            // Если мы на D0, движемся к D1
            STM_LOG("On D0 sensor, moving to D1");
            startForCall(dir::CCW);

            for(;;) {
                if(Status == statusMotor::STOPPED) {
                    if(StatusTarget == statusTarget_t::finished) {
                        if(HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) == GPIO_PIN_SET) {
                            STM_LOG("Successfully reached D1");
                            // Теперь двигаемся обратно к D0 для измерения расстояния
                            if(settings->mod_rotation ==  mode_rotation_t::calibration_timer){
                            	__HAL_TIM_SET_COUNTER(TimCountAllSteps,0);
                            }
                            else
                            {
                            	__HAL_TIM_SET_COUNTER(TimEncoder,0);
                            }
                            startForCall(dir::CW);

                            for(;;) {
                                if(Status == statusMotor::STOPPED) {
                                    if(StatusTarget == statusTarget_t::finished) {
                                        if(HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET) {
                                        	CallSteps = getCurrentSteps();
                                            if(settings->mod_rotation ==  mode_rotation_t::calibration_timer){
                                            	__HAL_TIM_SET_COUNTER(TimCountAllSteps,0);
                                            }
                                            else
                                            {
                                            	__HAL_TIM_SET_COUNTER(TimEncoder,0);
                                            }
                                            settings->sensors_map.detected = true;
                                            settings->points.is_calibrated = true;
                                            permission_calibrate = false;
                                            STM_LOG("Calibration completed. Steps: %d", CallSteps);
                                            ret = true;
                                        } else {
                                        	permission_calibrate = false;
                                        	ret = false;
                                        }
                                    } else {
                                    	permission_calibrate = false;
                                    	ret = false;
                                    }
                                    break;
                                }
                                osDelay(1);
                            }
                        } else {
                        	permission_calibrate = false;
                        	ret = false;
                        }
                    } else {
                    	permission_calibrate = false;
                    	ret = false;
                    }
                    break;
                }
                osDelay(1);
            }
        } else if(on_D1) {
            // Если мы на D1, движемся к D0 для завершения
            STM_LOG("On D1 sensor, moving to D0");
            if(settings->mod_rotation ==  mode_rotation_t::calibration_timer){
            	__HAL_TIM_SET_COUNTER(TimCountAllSteps,0);
            }
            else
            {
            	__HAL_TIM_SET_COUNTER(TimEncoder,0);
            }
            startForCall(dir::CW);

            for(;;) {
                if(Status == statusMotor::STOPPED) {
                    if(StatusTarget == statusTarget_t::finished) {
                        if(HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET) {
                        	CallSteps = getCurrentSteps();
                            if(settings->mod_rotation ==  mode_rotation_t::calibration_timer){
                            	__HAL_TIM_SET_COUNTER(TimCountAllSteps,0);
                            }
                            else
                            {
                            	__HAL_TIM_SET_COUNTER(TimEncoder,0);
                            }
                            settings->sensors_map.detected = true;
                            settings->points.is_calibrated = true;
                            permission_calibrate = false;
                            STM_LOG("Calibration completed. Steps: %d", CallSteps);
                            ret = true;
                        } else {
                        	permission_calibrate = false;
                        	ret = false;
                        }
                    }else {
                    	permission_calibrate = false;
						ret = false;
					}
                    break;
                }
                osDelay(1);
            }
        } else if (Status == statusMotor::STOPPED){
            // Если мы между датчиками, сначала движемся к D1
            STM_LOG("Between sensors, moving to D1");
            startForCall(dir::CCW);
        } else {
        	// ошибка стоит прервать калибровку
        }
    }
    settings->mod_rotation = temp_mode;
    return ret;
}

void extern_driver::CallStart() {
	permission_calibrate = true;
}

//methods for set************************************************
void extern_driver::SetSpeed(uint32_t percent) {
	if (percent > 1000) {
		percent = 1000;
	}
	else if (percent < 1) {
		percent = 1;
	}

	if (Status == statusMotor::STOPPED) {
		settings->Speed = (uint32_t) map_ARRFromPercent(percent);
	}
 	switch (settings->mod_rotation) {
		case step_by_meter_timer_intermediate:
		//case step_by_meter_timer_limit:
		case calibration_timer:
		case calibration_enc:
		case step_by_meter_enc_intermediate:
		//case step_by_meter_enc_limit:
		{
			break;
		}
		case bldc_limit:
    	case step_inf:
    	case bldc_inf:
		{
			//__HAL_TIM_SET_AUTORELOAD(TimFrequencies,(uint32_t) map_ARRFromPercent(percent));
			SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, (uint32_t) map_ARRFromPercent(percent));
			break;
		}
		default:
		{
			break;
		}
    }
	Parameter_update();
}


void extern_driver::SetStartSpeed(uint32_t percent) {
	if (percent > 1000) {
		percent = 1000;
	}
	if (percent < 1) {
		percent = 1;
	}
	//settings->StartSpeed = (uint32_t) map(percent, 1, 1000, MinSpeed, MaxSpeed);
	settings->StartSpeed = (uint32_t) map_ARRFromPercent(percent);

}

void extern_driver::SetAcceleration(uint32_t StepsINmS) {
	settings->Accel = StepsINmS;
	Parameter_update();
}

void extern_driver::SetSlowdown(uint32_t steps) {
	settings->Slowdown = steps;
	Parameter_update();

}

void extern_driver::setTimeOut(uint32_t time) {
	settings->TimeOut = time;
}

// Модифицированный метод SetMode() с пересчетом частоты
void extern_driver::SetMode(mode_rotation_t mod) {
    if (!settings) {
        STM_LOG("Error: settings is null");
        return;
    }

    // Запоминаем текущий режим для сравнения
    mode_rotation_t oldMode = settings->mod_rotation;

    // Устанавливаем новый режим
    settings->mod_rotation = mod;

    // Если режим изменился, обновляем параметры таймера и пересчитываем частоты
    if (oldMode != mod) {
        // Обновляем предделитель таймера и диапазон скоростей
        switch (mod) {
            case step_by_meter_enc_intermediate:
            case step_by_meter_timer_intermediate:
            case step_inf:
            {
                TimFrequencies->Instance->PSC = 399;
                MaxSpeed = 50;
                MinSpeed = 13000;
                break;
            }
            case bldc_limit:
            case bldc_inf:
            {
                TimFrequencies->Instance->PSC = 200-1;
                MaxSpeed = 100;
                MinSpeed = 2666;
                break;
            }
            default:
            {
                TimFrequencies->Instance->PSC = 399;
                MaxSpeed = 50;
                MinSpeed = 13000;
                break;
            }
        }

        // Пересчитываем частоту тактирования таймера
        calculateTimerFrequency();

        // Рассчитываем количество шагов, необходимых для торможения
        uint32_t brakingSteps = calculateBrakingSteps();
        uint32_t accelSteps = calculateAccelSteps();
        // Пересчитываем таблицы разгона и торможения
        calculateAccelTable(accelSteps);
        calculateDecelTable(brakingSteps);


        STM_LOG("Mode changed to %d, timer parameters updated", (int)mod);
    }
}


// расчитывает и сохраняет все параметры разгона и торможения
void extern_driver::Parameter_update(void) {

	//FeedbackBraking_P1 = target - ((uint16_t) map(Deaccel, 1, 1000, 1, target));
	//FeedbackBraking_P0 = CircleCounts - ((uint16_t) map(Deaccel, 1, 1000, 1, CircleCounts - target)); // Начало торможения перед нулевой точкой в отсчетах это 1000

	//расчет шага ускорения/торможения
	//Accel = Speed / (target - FeedbackBraking_P1);
	//TimCountAllSteps->Instance->ARR = target;
}

void extern_driver::SetDirection(dir direction) {
	settings->Direct = direction;
}

//methods for get************************************************

uint32_t extern_driver::getAcceleration() {
	return (uint32_t) settings->Accel;
}

uint32_t extern_driver::getSlowdown() {
	return (uint32_t) settings->Slowdown;
}

uint32_t extern_driver::getSpeed() {

	//return (uint32_t) map(settings->Speed, MinSpeed, MaxSpeed, 1, 1000);
	return (uint32_t) map_PercentFromARR(settings->Speed);
}

uint32_t extern_driver::getStartSpeed() {
 //return StartSpeed;
 //return (uint32_t) map(settings->StartSpeed, MinSpeed, MaxSpeed, 1, 1000);
	return (uint32_t) map_PercentFromARR(settings->StartSpeed);
}

uint32_t extern_driver::getTimeOut() {
	return settings->TimeOut;
}

pos_t extern_driver::get_pos() {

	pos_t tmp_pos = pos_t::D_0_1;

	GPIO_PinState D0 = HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin);
	GPIO_PinState D1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin);

	if ((D0 == GPIO_PIN_SET) && (D1 == GPIO_PIN_RESET)) {
		tmp_pos = pos_t::D0;
	}
	else if ((D0 == GPIO_PIN_RESET) && (D1 == GPIO_PIN_SET)){
		tmp_pos = pos_t::D1;
	}
	else {
		tmp_pos = pos_t::D_0_1;
	}

	return tmp_pos;
}

dir extern_driver::getStatusDirect() {
	return settings->Direct;
}

statusMotor extern_driver::getStatusRotation() {
	return Status;
}

uint16_t extern_driver::getRPM() {
	return 0;
}

mode_rotation_t extern_driver::getMode() {
	return settings->mod_rotation;
}

statusTarget_t extern_driver::getStatusTarget() {
	return StatusTarget;
}

void extern_driver::StartDebounceTimer(uint16_t GPIO_Pin) {
	// если мы находимся между концевиками то нужно остановить иначе запустить антидребезг

	//if((position == pos_t::D_0_1) && (ignore_sensors == false))
	if((position == pos_t::D_0_1) && (ignore_sensors == false))
	{
		SensHandler(GPIO_Pin);
	}
	else
	{
	    if(GPIO_Pin == D0_Pin && !d0_debounce_active) {
			d0_debounce_active = true;
			// Настраиваем и запускаем таймер
			__HAL_TIM_SET_COUNTER(debounceTimer, 0);
			HAL_TIM_Base_Start_IT(debounceTimer);
		}
		else if(GPIO_Pin == D1_Pin && !d1_debounce_active) {
			d1_debounce_active = true;
			// Настраиваем и запускаем таймер
			__HAL_TIM_SET_COUNTER(debounceTimer, 0);
			HAL_TIM_Base_Start_IT(debounceTimer);
		}
	}

}

void extern_driver::HandleDebounceTimeout() {
    // Останавливаем таймер
    HAL_TIM_Base_Stop_IT(debounceTimer);

    // Проверяем D0
    if(d0_debounce_active) {
        if(HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin) == GPIO_PIN_SET) {
            // Сигнал все еще активен - вызываем обработчик
            SensHandler(D0_Pin);
        }
        d0_debounce_active = false;
    }

    // Проверяем D1
    if(d1_debounce_active) {
        if(HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin) == GPIO_PIN_SET) {
            // Сигнал все еще активен - вызываем обработчик
            SensHandler(D1_Pin);
        }
        d1_debounce_active = false;
    }
}

bool extern_driver::validatePointNumber(uint32_t point_number) {
    return (point_number < MAX_POINTS);
}

void extern_driver::updateCurrentPosition(uint32_t pos) {
	__HAL_TIM_SET_COUNTER(TimCountAllSteps, pos);
	__HAL_TIM_SET_COUNTER(TimEncoder, pos);
}

bool extern_driver::saveCurrentPositionAsPoint(uint32_t point_number) {
    if(!validatePointNumber(point_number)) return false;
    if(!settings->points.is_calibrated) return false;

    settings->points.points[point_number] = getCurrentSteps();
    if(point_number >= settings->points.count) {
        settings->points.count = point_number + 1;
    }

    return true;
}

uint32_t extern_driver::getCurrentSteps() {
	uint32_t ret = 0;

 	switch (settings->mod_rotation) {
		case step_by_meter_timer_intermediate:
		//case step_by_meter_timer_limit:
		case calibration_timer:
		{
			ret = __HAL_TIM_GET_COUNTER(TimCountAllSteps);
			break;
		}
		case step_by_meter_enc_intermediate:
		//case step_by_meter_enc_limit:
		case calibration_enc:
		{
			ret = __HAL_TIM_GET_COUNTER(TimEncoder);
			break;
		}
		case bldc_limit:
		case step_inf:
    	case bldc_inf:
		default:
		{
			break;
		}
    }

 	return ret;
}

bool extern_driver::gotoPoint(uint32_t point_number) {
    if(!validatePointNumber(point_number)) return false;
    if(!settings->points.is_calibrated) return false;
    if(point_number >= settings->points.count) return false;

    settings->points.target_point = point_number;

    // Определяем направление движения
    if(settings->points.points[point_number] > getCurrentSteps()) {
        settings->Direct = dir::CCW;
    } else {
        settings->Direct = dir::CW;
    }

    return start(settings->points.points[point_number]);
}

bool extern_driver::gotoLSwitch(uint8_t sw_x) {
	if (settings->points.is_calibrated) {

		dir temp_diretion = dir::CW;

		if (sw_x == 0) {
			//start(0, dir::CW);
			temp_diretion = dir::CW;
		} else {
			//start(0xFFFFFFFF, dir::CCW);
			temp_diretion = dir::CCW;
		}

        // Проверяем состояние драйвера перед стартом
        checkDriverStatus();
        if (currentDriverStatus != DRIVER_OK) {
            STM_LOG("Cannot start: driver error detected");
            return false;
        }

		if (temp_diretion == dir::CW && HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin)) {
			STM_LOG("Cannot move CW: at CW limit switch");
			return false;
		}
		else if (temp_diretion == dir::CCW && HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)) {
			STM_LOG("Cannot move CCW: at CCW limit switch");
			return false;
		} else {

		}

        // Сбрасываем счетчик шагов таблицы
        rampTables.currentStep = 0;

        ignore_sensors = true;
        vibration_start_time = HAL_GetTick();
        StatusTarget = statusTarget_t::inProgress;


		if (temp_diretion == dir::CCW) {
			DIRECT_CCW
		} else {
			DIRECT_CW
		}

        // Рассчитываем количество шагов, необходимых для торможения
        uint32_t brakingSteps = calculateBrakingSteps();
        uint32_t accelSteps = calculateAccelSteps();
        // Пересчитываем таблицы разгона и торможения
        calculateAccelTable(accelSteps);
        calculateDecelTable(brakingSteps);

        // Сбрасываем счетчик шагов таблицы
        rampTables.currentStep = 0;

        switch (settings->mod_rotation) {
        	case calibration_enc:
        	case step_by_meter_enc_intermediate:
        	//case step_by_meter_enc_limit:
        	{
        		//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, settings->StartSpeed);
        		SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, rampTables.accelTable[0]);

    			//__HAL_TIM_SET_COUNTER(TimEncoder, 0);
    			__HAL_TIM_SET_AUTORELOAD(TimEncoder, 0xffff);
    			__HAL_TIM_SET_COMPARE(TimEncoder, TIM_CHANNEL_3, 0xffff);
    			__HAL_TIM_SET_COMPARE(TimEncoder, TIM_CHANNEL_4, 0xffff);
        		Status = statusMotor::ACCEL;
        		break;
        	}
        	case step_inf:
        	case calibration_timer:
        	case step_by_meter_timer_intermediate:
        	//case step_by_meter_timer_limit:
        	{
        		//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, settings->StartSpeed);
        		SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, rampTables.accelTable[0]);

    			//__HAL_TIM_SET_COUNTER(TimCountAllSteps, 0);
    			__HAL_TIM_SET_AUTORELOAD(TimCountAllSteps, 0xffff);
    			__HAL_TIM_SET_COMPARE(TimCountAllSteps, TIM_CHANNEL_1, 0xffff);
    			__HAL_TIM_SET_COMPARE(TimCountAllSteps, TIM_CHANNEL_2, 0xffff);
        		Status = statusMotor::ACCEL;
        		break;
        	}

        	case bldc_limit:
        	case bldc_inf:
        	{
        		//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, settings->Speed);
        		SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, settings->StartSpeed);
        		Status = statusMotor::MOTION;
        		break;
        	}
        	default:
        	{
        		break;
        	}
        }

        HAL_TIM_OC_Start_IT(TimFrequencies, ChannelClock);

        return true;
	}
	return false;
}

bool extern_driver::validatePosition(uint32_t position) {
    if(!settings->points.is_calibrated) return false;

    // Проверяем что позиция в допустимых пределах
    uint32_t max_pos = getMaxPosition();
    uint32_t min_pos = getMinPosition();

    return (position >= min_pos && position <= max_pos);
}

uint32_t extern_driver::getMaxPosition() const {
    // Максимальная позиция - это позиция второго концевика
    return CallSteps;
}

uint32_t extern_driver::getMinPosition() const {
    return 0;
}

void extern_driver::setPoint(uint32_t point, uint32_t abs_steps){
	settings->points.points[point] = abs_steps;
}

bool extern_driver::gotoPosition(uint32_t position) {
    if(!validatePosition(position)) {
        STM_LOG("Invalid position requested: %lu", position);
        return false;
    }

    // Проверяем состояние драйвера
    if(getDriverStatus() != DRIVER_OK) {
        STM_LOG("Driver error, cannot move");
        return false;
    }

    // Определяем направление движения
    if(position > getCurrentSteps()) {
        settings->Direct = dir::CCW;
    } else if(position < getCurrentSteps()) {
        settings->Direct = dir::CW;
    } else {
        // Уже в заданной позиции
        return true;
    }

    return start(position);
}


bool extern_driver::gotoInfinity() {
    switch (settings->mod_rotation) {
    	case calibration_enc:
    	case step_by_meter_enc_intermediate:
    	case calibration_timer:
    	case step_by_meter_timer_intermediate:
    	{
    		return false;
    		break;
    	}

    	case bldc_inf:
    	case step_inf:
    	case bldc_limit:
    	{
    		return start(-1);
    		break;
    	}
    	default:
    	{
    		return false;
    		break;
    	}
    }
}

bool extern_driver::isCalibrated() {
    return settings->points.is_calibrated;
}

//*******************************************************
void extern_driver::InitTim() {

}

double extern_driver::map(double x, double in_min, double in_max,
		double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Преобразует процентное значение в значение ARR с линейным изменением частоты
 * @param percent_value Значение в процентах (от 1 до 1000)
 * @return Значение регистра ARR (от MinSpeed до MaxSpeed)
 */
uint16_t extern_driver::map_ARRFromPercent(uint16_t percent_value)
{
    // Проверка входных значений
    if (percent_value < 1) {
        percent_value = 1;
    } else if (percent_value > 1000) {
        percent_value = 1000;
    }

    // Вычисляем частоту вместо ARR для линейности изменения скорости
    // Чем меньше ARR, тем выше частота (и скорость)
    float min_freq = 1.0f / MinSpeed;  // Минимальная частота
    float max_freq = 1.0f / MaxSpeed;  // Максимальная частота

    // Нормализованный процент (от 0.0 до 1.0)
    float normalized_percent = (float)(percent_value - 1) / (1000.0f - 1.0f);

    // Линейная интерполяция частоты
    float target_freq = min_freq + normalized_percent * (max_freq - min_freq);

    // Преобразуем частоту обратно в ARR (округляем до ближайшего целого)
    uint16_t arr_value = (uint16_t)(1.0f / target_freq + 0.5f);

    return arr_value;
}

/**
 * @brief Преобразует значение ARR в процентное значение с учетом линейности частоты
 * @param arr_value Значение регистра ARR
 * @return Значение в процентах (от 1 до 1000)
 */
uint16_t extern_driver::map_PercentFromARR(uint16_t arr_value)
{
    // Проверка входных значений
    if (arr_value < MaxSpeed) {
        arr_value = MaxSpeed;
    } else if (arr_value > MinSpeed) {
        arr_value = MinSpeed;
    }

    // Преобразуем ARR в частоту
    float current_freq = 1.0f / arr_value;
    float min_freq = 1.0f / MinSpeed;
    float max_freq = 1.0f / MaxSpeed;

    // Вычисляем нормализованный процент на основе линейной шкалы частоты
    float normalized_percent = (current_freq - min_freq) / (max_freq - min_freq);

    // Преобразуем в процентную шкалу от 1 до 1000
    float temp_percent = 1.0f + normalized_percent * (1000.0f - 1.0f);

    // Округляем до ближайшего целого
    uint16_t percent_value = (uint16_t)(temp_percent + 0.5f);

    // Проверка на граничные значения
    if (percent_value < 1) {
        percent_value = 1;
    } else if (percent_value > 1000) {
        percent_value = 1000;
    }

    return percent_value;
}

bool extern_driver::waitForStop(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while(Status != statusMotor::STOPPED) {
        if(HAL_GetTick() - start > timeout_ms) {
            STM_LOG("Stop timeout occurred");
            return false;
        }
        osDelay(1);
    }
    return true;
}

void  extern_driver::ChangeTimerMode(TIM_HandleTypeDef *htim, uint32_t Mode)
{
    // Остановить таймер
    HAL_TIM_Base_Stop(htim);

    // Изменить режим
    htim->Instance->CR1 &= ~TIM_CR1_CMS;  // Сбросить биты режима
    htim->Instance->CR1 &= ~TIM_CR1_DIR;  // Сбросить бит направления
    htim->Instance->CR1 |= Mode;          // Установить новый режим

    // Перезапустить таймер
    HAL_TIM_Base_Start(htim);
}

// Реализация метода расчета частоты тактирования таймера
void extern_driver::calculateTimerFrequency() {
    uint32_t timerClockFreq;

    // Определяем, к какой шине подключен таймер (APB1 или APB2)
    // TIM1 и TIM8-TIM11 обычно на APB2, остальные на APB1
    if (TimFrequencies->Instance == TIM1 ||
        TimFrequencies->Instance == TIM8 ||
        TimFrequencies->Instance == TIM9 ||
        TimFrequencies->Instance == TIM10 ||
        TimFrequencies->Instance == TIM11) {
        // Получаем частоту шины APB2
        timerClockFreq = HAL_RCC_GetPCLK2Freq();

        // Если делитель APB2 не равен 1, частота тактирования таймера удваивается
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0) {
            timerClockFreq *= 2;
        }
    } else {
        // Получаем частоту шины APB1
        timerClockFreq = HAL_RCC_GetPCLK1Freq();

        // Если делитель APB1 не равен 1, частота тактирования таймера удваивается
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) {
            timerClockFreq *= 2;
        }
    }

    // Учитываем предделитель таймера
    uint32_t prescaler = TimFrequencies->Instance->PSC + 1;
    timerTickFreq = timerClockFreq / prescaler;

    STM_LOG("Timer frequency calculated: %lu Hz (Clock: %lu Hz, Prescaler: %lu)",
            timerTickFreq, timerClockFreq, prescaler);
}

// Расчет количества шагов для ускорения
uint32_t extern_driver::calculateAccelSteps() {
    // Получаем начальную и целевую скорости
    uint32_t startARR = settings->StartSpeed;
    uint32_t targetARR = settings->Speed;

    // Проверка соответствия значений диапазонам
    if (startARR < MaxSpeed) startARR = MaxSpeed;
    if (startARR > MinSpeed) startARR = MinSpeed;
    if (targetARR < MaxSpeed) targetARR = MaxSpeed;
    if (targetARR > MinSpeed) targetARR = MinSpeed;

    // Преобразуем ARR в частоту (Гц) с использованием предварительно рассчитанной частоты
    float initialFreq = static_cast<float>(timerTickFreq) / (startARR + 1);   // Начальная скорость
    float finalFreq = static_cast<float>(timerTickFreq) / (targetARR + 1);    // Целевая скорость

    // Рассчитываем количество шагов для разгона на основе разницы скоростей
    // Используем формулу: s = (v_final^2 - v_initial^2) / (2 * acceleration)
    float v_initial = initialFreq / 1000.0f;  // Нормализуем для расчета
    float v_final = finalFreq / 1000.0f;      // Нормализуем для расчета
    float acceleration = settings->Accel / 10000.0f;  // Коэффициент ускорения

    // Рассчитываем количество шагов (округляем вверх)
    uint32_t accelSteps = static_cast<uint32_t>(
        (powf(v_final, 2) - powf(v_initial, 2)) / (2 * acceleration) + 0.5f
    );

    // Минимальное безопасное значение
    if (accelSteps < 20U) {
        accelSteps = 20U;
    }

    // Ограничиваем количество шагов размером буфера таблицы
    if (accelSteps > MAX_RAMP_STEPS) {
        accelSteps = MAX_RAMP_STEPS;
        STM_LOG("Warning: Acceleration steps limited to %d (buffer size)", MAX_RAMP_STEPS);
    }

    STM_LOG("Calculated acceleration steps: %lu (initial freq: %f Hz, final freq: %f Hz)",
            accelSteps, initialFreq, finalFreq);

    return accelSteps;
}

// Обновленный метод расчета количества шагов для торможения с использованием целевой скорости
uint32_t extern_driver::calculateBrakingSteps() {
    // Получаем целевую скорость из настроек, а не текущее значение ARR
    uint32_t targetARR = settings->Speed;

    // Проверка соответствия значения диапазону
    if (targetARR < MaxSpeed) targetARR = MaxSpeed;
    if (targetARR > MinSpeed) targetARR = MinSpeed;

    // Преобразуем ARR в частоту (Гц) с использованием предварительно рассчитанной частоты
    float initialFreq = static_cast<float>(timerTickFreq) / (targetARR + 1);  // Целевая скорость (начальная для торможения)
    float finalFreq = static_cast<float>(timerTickFreq) / (MinSpeed + 1);     // Минимальная скорость (конечная для торможения)

    // Рассчитываем дистанцию торможения на основе разницы скоростей
    // Используем формулу: s = (v_initial^2 - v_final^2) / (2 * deceleration)
    float v_initial = initialFreq / 1000.0f;  // Нормализуем для расчета
    float v_final = finalFreq / 1000.0f;      // Нормализуем для расчета
    float deceleration = settings->Slowdown / 10000.0f;  // Коэффициент замедления

    // Добавляем запас 20%
    uint32_t brakingSteps = static_cast<uint32_t>(
        (powf(v_initial, 2) - powf(v_final, 2)) / (2 * deceleration) * 1.2f
    );

    // Минимальное безопасное значение
    if (brakingSteps < 50U) {
        brakingSteps = 50U;
    }

    // Ограничиваем количество шагов размером буфера таблицы
    if (brakingSteps > MAX_RAMP_STEPS) {
        brakingSteps = MAX_RAMP_STEPS;
        STM_LOG("Warning: Braking steps limited to %d (buffer size)", MAX_RAMP_STEPS);
    }

    // Используем %f вместо %.2f для улучшения совместимости с форматированием
    STM_LOG("Calculated braking steps: %lu (initial freq: %f Hz, final freq: %f Hz)",
            brakingSteps, initialFreq, finalFreq);

    return brakingSteps;
}

// Реализация метода расчета таблицы разгона
void extern_driver::calculateAccelTable(uint32_t accelSteps) {
    // Получаем параметры разгона
    uint32_t startARR = settings->StartSpeed;  // Большое значение ARR = низкая скорость
    uint32_t targetARR = settings->Speed;      // Малое значение ARR = высокая скорость

    // Проверка соответствия значений диапазонам
    if (startARR < MaxSpeed) startARR = MaxSpeed;
    if (startARR > MinSpeed) startARR = MinSpeed;
    if (targetARR < MaxSpeed) targetARR = MaxSpeed;
    if (targetARR > MinSpeed) targetARR = MinSpeed;

    // Ограничиваем количество шагов таблицей и имеющимся диапазоном
    uint16_t steps = static_cast<uint16_t>(accelSteps);
    if (steps > MAX_RAMP_STEPS) steps = MAX_RAMP_STEPS;

    // Для линейного изменения частоты, нам нужно нелинейно менять ARR
    // Преобразуем ARR в частоту
    double startFreq = 1.0 / startARR;
    double targetFreq = 1.0 / targetARR;
    double freqDiff = targetFreq - startFreq;

    // Заполняем таблицу ускорения с линейным изменением частоты
    for (uint16_t i = 0; i < steps; i++) {
        // Линейно увеличиваем частоту
        double progress = static_cast<double>(i) / steps;
        double currentFreq = startFreq + (freqDiff * progress);

        // Преобразуем частоту обратно в ARR
        uint32_t arr = static_cast<uint32_t>(1.0 / currentFreq);

        // Проверка границ значения ARR
        if (arr < MaxSpeed) arr = MaxSpeed;
        if (arr > MinSpeed) arr = MinSpeed;

        // Сохраняем значение в таблице
        rampTables.accelTable[i] = arr;
    }

    // Убедимся, что последнее значение точно соответствует целевому
    if (steps > 0) {
        rampTables.accelTable[steps - 1] = targetARR;
    }

    // Сохраняем количество шагов в таблице
    rampTables.accelSteps = steps;

    // Выводим диагностику
    STM_LOG("Linear acceleration table: steps=%d, start=%lu, end=%lu",
           steps, rampTables.accelTable[0], rampTables.accelTable[steps-1]);
}

// Реализация метода расчета таблицы торможения
void extern_driver::calculateDecelTable(uint32_t brakingSteps) {
    // Получаем параметры торможения
    uint32_t startARR = settings->Speed;   // Начинаем с основной скорости
    uint32_t targetARR = MinSpeed;        // Заканчиваем на минимальной скорости

    // Проверка соответствия значений диапазонам
    if (startARR < MaxSpeed) startARR = MaxSpeed;
    if (startARR > MinSpeed) startARR = MinSpeed;

    // Ограничиваем количество шагов таблицей
    uint16_t steps = static_cast<uint16_t>(brakingSteps);
    if (steps > MAX_RAMP_STEPS) steps = MAX_RAMP_STEPS;

    // Для линейного изменения частоты, нам нужно нелинейно менять ARR
    // Преобразуем ARR в частоту
    double startFreq = 1.0 / startARR;
    double targetFreq = 1.0 / targetARR;
    double freqDiff = targetFreq - startFreq; // Будет отрицательное значение для торможения

    // Заполняем таблицу торможения с линейным изменением частоты
    for (uint16_t i = 0; i < steps; i++) {
        // Линейно уменьшаем частоту
        double progress = static_cast<double>(i) / steps;
        double currentFreq = startFreq + (freqDiff * progress);

        // Преобразуем частоту обратно в ARR
        uint32_t arr = static_cast<uint32_t>(1.0 / currentFreq);

        // Проверка границ значения ARR
        if (arr < MaxSpeed) arr = MaxSpeed;
        if (arr > MinSpeed) arr = MinSpeed;

        // Сохраняем значение в таблице
        rampTables.decelTable[i] = arr;
    }

    // Убедимся, что последнее значение точно соответствует целевому
    if (steps > 0) {
        rampTables.decelTable[steps - 1] = targetARR;
    }

    // Сохраняем количество шагов в таблице
    rampTables.decelSteps = steps;

    // Выводим диагностику
    STM_LOG("Linear deceleration table: steps=%d, start=%lu, end=%lu",
           steps, rampTables.decelTable[0], rampTables.decelTable[steps-1]);
}

// Обработчик прерывания таймера
void extern_driver::handleTimerInterrupt() {

	// Обработка разгона
	if (Status == statusMotor::ACCEL) {
		// Увеличиваем счетчик шагов в таблице
		rampTables.currentStep++;

		// Проверяем, достигли ли конца таблицы разгона
		if (rampTables.currentStep >= rampTables.accelSteps) {
			// Достигнута целевая скорость
			//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, settings->Speed);
			SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, settings->Speed);
			Status = statusMotor::MOTION;
			STM_LOG("Acceleration complete, reached target speed");
		} else {
			// Устанавливаем следующее значение ARR из таблицы
			//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, rampTables.accelTable[rampTables.currentStep]);
			SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, rampTables.accelTable[rampTables.currentStep]);
		}
	}
	// Обработка торможения
	else if (Status == statusMotor::BRAKING) {
		// Увеличиваем счетчик шагов в таблице
		rampTables.currentStep++;

		// Проверяем, достигли ли конца таблицы торможения
		if (rampTables.currentStep >= rampTables.decelSteps) {
			// Достигнута минимальная скорость
			//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, MinSpeed);
			SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, MinSpeed);

			// В зависимости от режима работы, можем остановиться или продолжить на минимальной скорости
			switch (settings->mod_rotation) {
				case step_inf:
				case bldc_inf:
					// В бесконечных режимах останавливаемся
					stop(statusTarget_t::finished);
					break;
				default:
					// В других режимах продолжаем на минимальной скорости до достижения целевой позиции
					break;
			}

			//STM_LOG("Deceleration complete, reached minimum speed");
		} else {
			// Устанавливаем следующее значение ARR из таблицы
			//__HAL_TIM_SET_AUTORELOAD(TimFrequencies, rampTables.decelTable[rampTables.currentStep]);
			SET_TIM_ARR_AND_PULSE(TimFrequencies, ChannelClock, rampTables.decelTable[rampTables.currentStep]);
		}
	}
}

extern_driver::~extern_driver() {

}

extern_driver::extern_driver(settings_t *set, TIM_HandleTypeDef *timCount,
		TIM_HandleTypeDef *timFreq, uint32_t channelFreq,
		TIM_HandleTypeDef *timDebounce, TIM_HandleTypeDef *timENC) :
		settings(set), TimCountAllSteps(timCount), TimFrequencies(timFreq), ChannelClock(
				channelFreq), debounceTimer(timDebounce), TimEncoder(timENC) {
}

/*
 *
 *
 	switch (settings->mod_rotation) {
		case step_by_meter_enc_intermediate:
		case step_by_meter_enc_limit:
		case step_by_meter_timer_intermediate:
		case step_by_meter_timer_limit:
		case bldc_limit:
		case calibration:
		{
			break;
		}
    	case step_inf:
    	case bldc_inf:
		{
			break;
		}
		default:
		{
			break;
		}
    }
    */
