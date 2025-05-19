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

// Timer and PWM Settings
#define RGB_TIM_HANDLE      &htim4
#define RGB_CHANNEL_RED     TIM_CHANNEL_1
#define RGB_CHANNEL_GREEN   TIM_CHANNEL_2
#define RGB_CHANNEL_BLUE    TIM_CHANNEL_3

// PWM resolution (ARR)
#define RGB_PWM_MAX         999  // Must match your TIM ARR value

#endif /* INC_GLOBAL_CONFIGURATION_H_ */
