/*
 * redundancy_sensors.c
 *
 *  Created on: May 19, 2025
 *      Author: Yassine DEHHANI
 */


#include "sensors_hubs/redundancy_sensors.h"
#include "drivers_h/altimeter/mpl3115a2s.h"
// TODO: #include H3LIS331DLTR driver
// TODO: #include LSM6DSOXTR driver

static RedundantSensorData sensor_data = {0};

// === Init all sensors ===
void RedundantSensors_Init(void)
{
    MPL3115A2S_Config mpl_cfg = { .altimeter_mode = 1, .os_ratio = MPL3115A2S_CTRL_OS128 };
    MPL3115A2S_Data_Config mpl_data_cfg = { .enable_all_flags = 1 };
    MPL3115A2S_Int_Config mpl_int_cfg = {0};

    sensor_data.altimeter.ok = (MPL3115A2S_Init(&mpl_cfg, &mpl_data_cfg, &mpl_int_cfg) == MPL_OK);

    // TODO: Init H3LIS331DLTR
    // TODO: Init LSM6DSOXTR
}

// === Read and update data ===
void RedundantSensors_Update(void)
{
    float alt = 0, temp = 0;
    if (MPL3115A2S_ReadDataPolling(&alt, &temp) == MPL_OK) {
        sensor_data.altimeter.altitude = alt;
        sensor_data.altimeter.temperature = temp;
        sensor_data.altimeter.ok = 1;
    } else {
        sensor_data.altimeter.ok = 0;
    }

    // TODO: Read H3LIS331DLTR
    // TODO: Read LSM6DSOXTR
}

// === Return snapshot of current data ===
RedundantSensorData RedundantSensors_GetData(void)
{
    return sensor_data;
}
