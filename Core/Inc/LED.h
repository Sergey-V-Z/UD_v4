#include "main.h"
#include "cmsis_os.h"

enum class mode{ON_OFF, BLINK, PULSE};
enum class status{ON, OFF};
//******************
// CLASS: led
//
// DESCRIPTION:
//  Класс для работы светодиода
//
// CREATED: 16.12.2020, by Sergey
//
// FILE: LED.cpp
//
class led{
public:
  led();
  ~led();
  void poll();
  void Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
  void setParameters(mode Mode, uint32_t period, uint32_t time_on);
  void setParameters(mode Mode);
  void LEDon();
  void LEDon(uint32_t pulses);
  void LEDoff();
  
private:
 uint32_t InitStatus = 0;
 uint32_t Counter_ms = 0;
 uint32_t PulseCount = 0;
 uint32_t Pulses = 0;
 uint32_t Timer = 0;
 uint32_t Period = 1000;
 uint32_t Time_on = 100;
 mode LED_mod = mode::BLINK;
 status StateLED = status::OFF;
 status StatusLED = status::OFF;
 
 GPIO_TypeDef* GPIOx;
 uint16_t GPIO_Pin;
};