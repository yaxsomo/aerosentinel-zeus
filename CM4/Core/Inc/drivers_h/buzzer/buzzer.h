/*
 * buzzer.h
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_BUZZER_BUZZER_H_
#define INC_DRIVERS_H_BUZZER_BUZZER_H_

#include "stm32h7xx_hal.h"
#include "global/configuration.h"

// Public API
void BUZZER_Init(void);
void BUZZER_On(void);
void BUZZER_Off(void);
void BUZZER_Beep(uint16_t duration_ms); // Blocking beep
void BUZZER_SetDuty(uint8_t duty_percent); // Optional: custom volume

#endif /* INC_DRIVERS_H_BUZZER_BUZZER_H_ */
