/*
 * rgb_led.c
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */

#include "drivers_h/rgb_led/rgb_led.h"

// === Internal Color Lookup Table (Percent values: 0–100) ===
static const struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} color_table[RGB_COLOR_COUNT] = {
    [OFF]    = { 0,   0,   0   },
    [RED]    = { 100, 0,   0   },
    [GREEN]  = { 0,   100, 0   },
    [BLUE]   = { 0,   0,   100 },
    [WHITE]  = { 100, 100, 100 },
    [YELLOW] = { 100, 100, 0   },
    [PURPLE] = { 100, 0,   100 },
    [CYAN]   = { 0,   100, 100 },
    [ORANGE] = { 100, 60,  0   },
    [PINK]   = { 100, 20,  50  },
    [VIOLET] = { 80,  0,   90  }
};

// === Private Helper ===
static uint32_t RGB_PercentToCCR(uint8_t percent)
{
    if (percent > 100) percent = 100;
    return RGB_PWM_MAX - ((RGB_PWM_MAX * percent) / 100);
}

// === API Implementation ===
void RGB_LED_Init(void)
{
    HAL_TIM_PWM_Start(RGB_TIM_HANDLE, RGB_CHANNEL_RED);
    HAL_TIM_PWM_Start(RGB_TIM_HANDLE, RGB_CHANNEL_GREEN);
    HAL_TIM_PWM_Start(RGB_TIM_HANDLE, RGB_CHANNEL_BLUE);
}

void RGB_LED_SetRGB(uint8_t red_percent, uint8_t green_percent, uint8_t blue_percent)
{
    __HAL_TIM_SET_COMPARE(RGB_TIM_HANDLE, RGB_CHANNEL_RED,   RGB_PercentToCCR(red_percent));
    __HAL_TIM_SET_COMPARE(RGB_TIM_HANDLE, RGB_CHANNEL_GREEN, RGB_PercentToCCR(green_percent));
    __HAL_TIM_SET_COMPARE(RGB_TIM_HANDLE, RGB_CHANNEL_BLUE,  RGB_PercentToCCR(blue_percent));
}

void RGB_LED_SetPredefinedColor(RGB_Color color)
{
    if (color >= RGB_COLOR_COUNT) return;

    RGB_LED_SetRGB(
        color_table[color].red,
        color_table[color].green,
        color_table[color].blue
    );
}
