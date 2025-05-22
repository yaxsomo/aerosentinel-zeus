#include "sensors_hubs/redundancy_sensors.h"
#include "custom_bus.h"

RedundantSensorData sensor_data = {0};



H3LIS331DL_Axes_t high_g_accelerometer_axes;
H3LIS331DL_Object_t high_g_accelerometer;

void _RedundancySensors_Init_Error_Handler(){
	while(1){

	}
}


//static int32_t LSM6DSO_SPI_Write(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len) {
//    LSM6DSO_CS_LOW();
//    uint8_t reg_addr = (uint8_t)(Reg & 0xFF);
//    HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
//    HAL_SPI_Transmit(&hspi1, pData, len, HAL_MAX_DELAY);
//    LSM6DSO_CS_HIGH();
//    return 0;
//}

//static int32_t LSM6DSO_SPI_Read(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len) {
//    LSM6DSO_CS_LOW();
//    uint8_t reg_addr = (uint8_t)(Reg | 0x80); // Read bit set
//    HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
//    HAL_SPI_Receive(&hspi1, pData, len, HAL_MAX_DELAY);
//    LSM6DSO_CS_HIGH();
//    return 0;
//}
//
//
//static int32_t h3lis331dl_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
//{
//    reg |= 0x40; // multi-byte write
//    HAL_GPIO_WritePin(H3LIS331DL_CS_PORT, H3LIS331DL_CS_PIN, GPIO_PIN_RESET);
//    HAL_SPI_Transmit(&H3LIS331DL_SPI_HANDLE, &reg, 1, H3LIS331DL_TIMEOUT);
//    HAL_SPI_Transmit(&H3LIS331DL_SPI_HANDLE, (uint8_t*)bufp, len, H3LIS331DL_TIMEOUT);
//    HAL_GPIO_WritePin(H3LIS331DL_CS_PORT, H3LIS331DL_CS_PIN, GPIO_PIN_SET);
//    return 0;
//}

//static int32_t h3lis331dl_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
//{
//    reg |= 0xC0; // read and multi-byte
//    HAL_GPIO_WritePin(H3LIS331DL_CS_PORT, H3LIS331DL_CS_PIN, GPIO_PIN_RESET);
//    HAL_SPI_Transmit(&H3LIS331DL_SPI_HANDLE, &reg, 1, H3LIS331DL_TIMEOUT);
//    HAL_SPI_Receive(&H3LIS331DL_SPI_HANDLE, bufp, len, H3LIS331DL_TIMEOUT);
//    HAL_GPIO_WritePin(H3LIS331DL_CS_PORT, H3LIS331DL_CS_PIN, GPIO_PIN_SET);
//    return 0;
//}



// === Init all sensors ===
void RedundantSensors_Init(void)
{
	// INIT IOs //
	H3LIS331DL_IO_t high_g_accelerometer_io_ctx;
	uint8_t high_g_accelerometer_id;


    // MPL3115A2S INIT
    MPL3115A2S_Config mpl_cfg = { .altimeter_mode = 1, .os_ratio = MPL3115A2S_CTRL_OS128 };
    MPL3115A2S_Data_Config mpl_data_cfg = { .enable_all_flags = 1 };
    MPL3115A2S_Int_Config mpl_int_cfg = {0};
    sensor_data.altimeter.ok = (MPL3115A2S_Init(&mpl_cfg, &mpl_data_cfg, &mpl_int_cfg) == MPL_OK);

    // H3LIS331DLTR INIT
    high_g_accelerometer_io_ctx.BusType = H3LIS331DL_SPI_4WIRES_BUS;
	high_g_accelerometer_io_ctx.Init = BSP_SPI2_Init;
	high_g_accelerometer_io_ctx.DeInit = BSP_SPI2_DeInit;
	high_g_accelerometer_io_ctx.ReadReg = BSP_SPI2_Recv;
	high_g_accelerometer_io_ctx.WriteReg = BSP_SPI2_Send;
	high_g_accelerometer_io_ctx.GetTick = BSP_GetTick;
	high_g_accelerometer_io_ctx.Delay = HAL_Delay;
	H3LIS331DL_RegisterBusIO(&high_g_accelerometer,&high_g_accelerometer_io_ctx);
	H3LIS331DL_ReadID(&high_g_accelerometer,&high_g_accelerometer_id);
	if(high_g_accelerometer_id != H3LIS331DL_ID){
		_RedundancySensors_Init_Error_Handler();
	}
	H3LIS331DL_Init(&high_g_accelerometer);
	H3LIS331DL_ACC_SetOutputDataRate(&high_g_accelerometer,H3LIS331DL_ODR_100Hz);
	H3LIS331DL_ACC_SetFullScale(&high_g_accelerometer,H3LIS331DL_400g);
	H3LIS331DL_ACC_Enable(&high_g_accelerometer);

    // LSM6DSO INIT

}

// === Read and update data ===
void RedundantSensors_Update(void)
{
    float alt = 0, temp = 0;

    // MPL3115A2S
    if (MPL3115A2S_ReadDataPolling(&alt, &temp) == MPL_OK) {
        sensor_data.altimeter.altitude = alt;
        sensor_data.altimeter.temperature = temp;
        sensor_data.altimeter.ok = 1;
    } else {
        sensor_data.altimeter.ok = 0;
    }

    // H3LIS331DLTR
//    H3LIS331DL_Update(&sensor_data.high_g_acc);
//
//    // LSM6DSOXTR
//    LSM6DSO_Axes_t acc, gyro;
//    if (LSM6DSO_ACC_GetAxes(&lsm6dso, &acc) == LSM6DSO_OK &&
//        LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro) == LSM6DSO_OK) {
//        sensor_data.imu.acc_x = acc.x / 1000.0f;
//        sensor_data.imu.acc_y = acc.y / 1000.0f;
//        sensor_data.imu.acc_z = acc.z / 1000.0f;
//
//        sensor_data.imu.gyro_x = gyro.x / 1000.0f;
//        sensor_data.imu.gyro_y = gyro.y / 1000.0f;
//        sensor_data.imu.gyro_z = gyro.z / 1000.0f;
//
//        sensor_data.imu.ok = 1;
//    } else {
//        sensor_data.imu.ok = 0;
//    }
}

// === Return snapshot of current data ===
RedundantSensorData RedundantSensors_GetData(void)
{
    return sensor_data;
}


