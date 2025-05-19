/*
 * redundancy_sensors.h
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */

#ifndef INC_SENSORS_HUBS_REDUNDANCY_SENSORS_H_
#define INC_SENSORS_HUBS_REDUNDANCY_SENSORS_H_


#include "stm32h7xx_hal.h"

// === Altimeter Data ===
typedef struct {
    float altitude;
    float temperature;
    uint8_t ok;
} MPL3115A2S_Data;

// === High-G Accelerometer (H3LIS331DLTR) ===
typedef struct {
    float x_g;
    float y_g;
    float z_g;
    uint8_t ok;
} H3LIS331DLTR_Data;

// === IMU with AI (LSM6DSOXTR) ===
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint8_t ai_detected_motion; // placeholder for AI output
    uint8_t ok;
} LSM6DSOXTR_Data;

// === Global Redundant Sensor State ===
typedef struct {
    MPL3115A2S_Data altimeter;
    H3LIS331DLTR_Data high_g_acc;
    LSM6DSOXTR_Data imu;
} RedundantSensorData;

// === Public API ===
void RedundantSensors_Init(void);
void RedundantSensors_Update(void);
RedundantSensorData RedundantSensors_GetData(void);

#endif /* INC_SENSORS_HUBS_REDUNDANCY_SENSORS_H_ */
