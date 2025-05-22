/*
 * manager.c
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */

//Custom Libraries Includes
#include "manager/manager.h"
#include "drivers_h/rgb_led/rgb_led.h"
#include "global/configuration.h"
#include "drivers_h/buzzer/buzzer.h"
#include "sensors_hubs/redundancy_sensors.h"
//Native C Libraries Includes
#include <stdbool.h>
#include <stdio.h>

extern bool USB_Mode;

/**
  * @brief  USB Mode Check Function
  *
  * @retval         Return value (true -> USB Mode Active | false -> Normal Mode Active)
  *
  */
bool Is_USB_Mode(){
	if (USB_Mode){ // Checks if the USB_Mode variable is true
		RGB_LED_SetPredefinedColor(PURPLE); // Sets the RGBW Led to USB Mode
		return true;
	}
	return false;
}



void passive_peripherals_init(){
	// Passive Peripherals Initialization
	RGB_LED_Init(); // Start PWM on RGB channels
	BUZZER_Init();
}



/**
  * @brief  Control Center Main Function
  *
  * @retval         Return status (MANDATORY: return 0 -> No Error | 10 -> Module in USB Mode | 1 -> Something went wrong)
  *
  */
Zeus_Status NAV_MAIN()
{
    passive_peripherals_init();
    BUZZER_Beep(1000); // short beep
    HAL_Delay(900);

    // Checks if the USB Mode is active
    if (Is_USB_Mode())
    {
        // In this case, we shouldn't do anything, as the USB connection is
        // established and the user is interacting with the NAND flash memory
        return STATUS_USB_MODE;
    }

    // Normal mode
    printf("Normal Mode Active\r\n");
    RGB_LED_SetPredefinedColor(WHITE);

    RedundantSensors_Init();

    while (1)
    {
        RedundantSensors_Update();
        RedundantSensorData data = RedundantSensors_GetData();

        // --- Altimeter ---
        if (data.altimeter.ok)
        {
            printf("Altitude: %.2f m | Temperature: %.2f °C\r\n",
                   data.altimeter.altitude,
                   data.altimeter.temperature);
        }
        else
        {
            printf("MPL3115A2S not responding!\r\n");
        }

//        // --- High-G Accelerometer ---
//        if (data.high_g_acc.ok)
//        {
//            printf("H3LIS Acc X: %.2f g | Y: %.2f g | Z: %.2f g\r\n",
//                   data.high_g_acc.x_g,
//                   data.high_g_acc.y_g,
//                   data.high_g_acc.z_g);
//        }
//        else
//        {
//            printf("H3LIS331DLTR not ready\r\n");
//        }
//
//        // --- LSM6DSOXTR IMU ---
//        if (data.imu.ok)
//        {
//            printf("LSM6DSOX Acc  -> X: %.2f g | Y: %.2f g | Z: %.2f g\r\n",
//                   data.imu.acc_x,
//                   data.imu.acc_y,
//                   data.imu.acc_z);
//            printf("LSM6DSOX Gyro -> X: %.2f °/s | Y: %.2f °/s | Z: %.2f °/s\r\n",
//                   data.imu.gyro_x,
//                   data.imu.gyro_y,
//                   data.imu.gyro_z);
//        }
//        else
//        {
//            printf("LSM6DSOXTR not responding!\r\n");
//        }

        HAL_Delay(1000);
    }

    RGB_LED_SetPredefinedColor(GREEN); // Sets the RGBW Led to OK
    return STATUS_IDLE;
}
