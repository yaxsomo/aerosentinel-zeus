/*
 * buzzer.c
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */


#include "drivers_h/buzzer/buzzer.h"

// === Private helper ===
static uint32_t BUZZER_PercentToCCR(uint8_t percent)
{
    if (percent > 100) percent = 100;
    return (BUZZER_PWM_PERIOD * percent) / 100;
}

// === Init ===
void BUZZER_Init(void)
{
    HAL_TIM_PWM_Start(BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL);
    BUZZER_Off(); // Start silent
}

// === Turn buzzer ON (50% duty cycle) ===
void BUZZER_On(void)
{
    __HAL_TIM_SET_COMPARE(BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL, BUZZER_PWM_PULSE);
}

// === Turn buzzer OFF ===
void BUZZER_Off(void)
{
    __HAL_TIM_SET_COMPARE(BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL, 0);
}

// === Beep for X ms (blocking) ===
void BUZZER_Beep(uint16_t duration_ms)
{
    BUZZER_On();
    HAL_Delay(duration_ms);
    BUZZER_Off();
}

// === Optional: Set custom duty (volume) ===
void BUZZER_SetDuty(uint8_t duty_percent)
{
    uint32_t ccr = BUZZER_PercentToCCR(duty_percent);
    __HAL_TIM_SET_COMPARE(BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL, ccr);
}
