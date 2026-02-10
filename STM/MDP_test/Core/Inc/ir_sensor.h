#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "stm32f4xx_hal.h"  // CHANGE to your STM32 family header if needed

// === Pin mapping (you said C6 and C9 are free) ===
#define IR_LEFT_GPIO_Port    GPIOC
#define IR_LEFT_Pin          GPIO_PIN_6   // LEFT  -> PC6

#define IR_RIGHT_GPIO_Port   GPIOC
#define IR_RIGHT_Pin         GPIO_PIN_9   // RIGHT -> PC9
// ================================================

// 0 = active LOW (most obstacle modules). 1 = active HIGH.
// If your module drives HIGH when an object is present, set to 1.
#define IR_ACTIVE_HIGH  0

#ifdef __cplusplus
extern "C" {
#endif

void IR_Sensors_Init(void);

// Returns 1 = object detected, 0 = clear
static inline uint8_t IR_LeftDetected(void){
#if IR_ACTIVE_HIGH
  return (HAL_GPIO_ReadPin(IR_LEFT_GPIO_Port, IR_LEFT_Pin) == GPIO_PIN_SET);
#else
  return (HAL_GPIO_ReadPin(IR_LEFT_GPIO_Port, IR_LEFT_Pin) == GPIO_PIN_RESET);
#endif
}

static inline uint8_t IR_RightDetected(void){
#if IR_ACTIVE_HIGH
  return (HAL_GPIO_ReadPin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin) == GPIO_PIN_SET);
#else
  return (HAL_GPIO_ReadPin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin) == GPIO_PIN_RESET);
#endif
}

#ifdef __cplusplus
}
#endif

#endif // IR_SENSOR_H
