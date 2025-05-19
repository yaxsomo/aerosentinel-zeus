/*
 * rgb_led.h
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */

#ifndef INC_DRIVERS_H_RGB_LED_RGB_LED_H_
#define INC_DRIVERS_H_RGB_LED_RGB_LED_H_

#include "stm32h7xx_hal.h"
#include "global/configuration.h"

// ===== Named Color Enumeration =====
typedef enum {
    OFF = 0,
    RED,
    GREEN,
    BLUE,
    WHITE,
    YELLOW,
    PURPLE,
    CYAN,
    ORANGE,
    PINK,
    VIOLET,
	RGB_COLOR_COUNT
} RGB_Color;

// ===== API =====
void RGB_LED_Init(void);
void RGB_LED_SetRGB(uint8_t red_percent, uint8_t green_percent, uint8_t blue_percent);
void RGB_LED_SetPredefinedColor(RGB_Color color);

#endif /* INC_DRIVERS_H_RGB_LED_RGB_LED_H_ */
