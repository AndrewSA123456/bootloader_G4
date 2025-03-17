#ifndef __MAIN_H
#define __MAIN_H

#include "stm32g4xx_hal.h"

void Error_Handler(void);
 
#define user_led_Pin GPIO_PIN_5 
#define user_led_GPIO_Port GPIOA
  
 
#endif /* __MAIN_H */
