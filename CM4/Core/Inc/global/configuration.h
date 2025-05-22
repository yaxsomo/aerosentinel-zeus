/*
 * configuration.h
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */

#ifndef INC_GLOBAL_CONFIGURATION_H_
#define INC_GLOBAL_CONFIGURATION_H_

#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/* RGB LED CONFIGURATION */
// Timer and PWM Settings
#define RGB_TIM_HANDLE      &htim4
#define RGB_CHANNEL_RED     TIM_CHANNEL_1
#define RGB_CHANNEL_GREEN   TIM_CHANNEL_2
#define RGB_CHANNEL_BLUE    TIM_CHANNEL_3
// PWM resolution (ARR)
#define RGB_PWM_MAX         999  // Must match TIM4 ARR value

/* BUZZER CONFIGURATION */
#define BUZZER_TIM_HANDLE    &htim5
#define BUZZER_TIM_CHANNEL   TIM_CHANNEL_1
#define BUZZER_PWM_PRESCALER 239
#define BUZZER_PWM_PERIOD    369
#define BUZZER_PWM_PULSE     184 // 50% duty cycle




#endif /* INC_GLOBAL_CONFIGURATION_H_ */
