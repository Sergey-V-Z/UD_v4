#include "LED.h"


void led::poll(){
   if(StatusLED == status::ON){
      switch(LED_mod)
      {
        case mode::BLINK:
        {
           if(Timer == Counter_ms){
              if(StateLED == status::ON){ 
                 HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // off led
                 Timer = Period - Time_on;
                 Counter_ms = 0;
                 StateLED = status::OFF;
              }else{
                 HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // on led
                 Timer = Time_on;
                 Counter_ms = 0;
                 StateLED = status::ON;            
              }
           }else{
              Counter_ms++; 
           }
          break;
        }
        case mode::ON_OFF:
        {
           if(StateLED == status::OFF) {
              HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // on led
              StateLED = status::ON;
           }
          break;
        }
        case mode::PULSE:
        {
           if(Timer == Counter_ms){
              if(StateLED == status::ON){ 
                 HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // off led
                 Timer = Period - Time_on;
                 Counter_ms = 0;
                 StateLED = status::OFF;
                 if(PulseCount == Pulses){
                    StatusLED = status::OFF;
                    LED_mod = mode::ON_OFF;
                    PulseCount = 0;
                    Pulses = 0;
                 }else{
                    PulseCount++; 
                 }
              }else{
                 HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // on led
                 Timer = Time_on;
                 Counter_ms = 0;
                 StateLED = status::ON;            
              }
           }else{
              Counter_ms++; 
           }
          break;
        }
        default:
        {
          break;
        }
        
      }

      if(LED_mod == mode::BLINK){   

      }else{

      }
   }else{
      HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // off led
      Counter_ms = 0;
      StateLED = status::OFF;
   }
}

void led::Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
   led::GPIOx = GPIOx;
   led::GPIO_Pin = GPIO_Pin;
   HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // off led
   InitStatus = 1;
}

void led::LEDon(){
   StatusLED = status::ON;
}

void led::LEDon(uint32_t pulses){
   StatusLED = status::ON;
   LED_mod = mode::PULSE;
   Pulses += pulses;
}

void led::LEDoff(){
   StatusLED = status::OFF;
}

void led::setParameters(mode Mode, uint32_t period, uint32_t time_on){
   LED_mod = Mode;
   Period = period;
   Time_on = time_on;
}

void led::setParameters(mode Mode){
   LED_mod = Mode;
}

led::led(){

}

led::~led(){

}
