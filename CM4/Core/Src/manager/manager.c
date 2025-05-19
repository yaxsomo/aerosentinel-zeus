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
Zeus_Status NAV_MAIN(){
	passive_peripherals_init();
    BUZZER_Beep(1000); // short beep
    HAL_Delay(900);

    RedundantSensors_Init();

	  // Checks if the USB Mode is active
	  if (Is_USB_Mode())
	  {
		  // In this case, we shouldn't do anything, as the USB connection is
		  // established and the user is interacting with the NAND flash memory (Returning 0)
		  return STATUS_USB_MODE;
	  }


		// Normal mode
		printf("Normal Mode Active\r\n");
		RGB_LED_SetPredefinedColor(WHITE);

//		FlightCard flight_card = {
//		    .rocket_name = "AeroSentinel Maiden Flight",
//		    .motor_used = "Moteur E600",
//		    .flyer = "Yassine Dehhani",
//		    .flight_date = "2024-10-20",
//		    .location = "Terrain Bordeaux",
//		    .flight_computer = "Aerosentinel Argus V2"
//		};
//
//		Set_Status_Color(STATUS_INITIALIZATION);
//		if(Blackbox_Init(&flight_card) == STATUS_OK) {
//			printf("Blackbox Initialized for current flight! \n");
//			if(SensorManager_Init() == STATUS_OK){
//				printf("All Sensors Initialized and calibrated! \n");
//				Set_Status_Color(STATUS_READING); // Sets the RGBW Led to Reading Mode
//				StartTelemetryAquisition(interval_ms,timeout_ms);
////				TestTelemetry();
//			} else {
//				Set_Status_Color(STATUS_ERROR); // Sets the RGBW Led to Error Mode
//				return SENSORS_INIT_ERROR;
//			}
//		} else {
//			Set_Status_Color(STATUS_ERROR); // Sets the RGBW Led to Error Mode
//			return BLACKBOX_ERROR;
//		}

	    while (1)
	    {
	        RedundantSensors_Update();
	        RedundantSensorData data = RedundantSensors_GetData();

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

	        HAL_Delay(1000);
	    }






	  //  TestTelemetry();
RGB_LED_SetPredefinedColor(GREEN); // Sets the RGBW Led to OK
return STATUS_IDLE;
}
