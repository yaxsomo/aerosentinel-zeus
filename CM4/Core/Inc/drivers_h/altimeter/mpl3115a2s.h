/**
 ******************************************************************************
 * File Name          : MPL3115A2S_Driver.h
 * Description        : MPL3115A2S Driver
 *  Supports Polling Mode and Interrupt Mode
 *  (FIFO Configuration not yet supported)
 *
 * Usage                 :
 *  1. Create an MPL3115A2S_Config, MPL3115A2S_Data_Config, and 
 *     if interrupt mode is used, the MPL3115A2S_INT_Config structs.
 *     Define each struct as {0} to use default values, then set the 
 *     relevant fields as needed.
 *  2. Call MPL3115A2S_Init() with the desired configurations.
 * 
 * Potential Improvements :
 * - Add offset setting support (OFF_P, OFF_T, OFF_H registers)
 * - Add FIFO Configuration Support
 * - Add additional interrupt decoding support (SRC_PW, TW, PTH, TTH, PCH, TCH)
 * 
 * Authors            : Chris (cjchanx)
 ******************************************************************************
*/
#ifndef INCLUDE_MPL3115A2S_DRIVER_H_
#define INCLUDE_MPL3115A2S_DRIVER_H_
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
//#include "../I2CPeripheral_Interface.h"



#include "stm32h7xx_hal.h"

// Replace this with your actual I2C handle
extern I2C_HandleTypeDef hi2c2;

/* Macros and Constants ------------------------------------------------------*/
#define MPL3115A2S_ADDR (0xC0) // 8-bit write address for MPL3115A2S

// Registers
#define MPL3115A2S_STATUS (0x00) // Status Register

#define MPL3115A2S_OUT_P_MSB (0x01) // Pressure Data Out MSB
#define MPL3115A2S_OUT_P_CSB (0x02) // Pressure Data Out CSB
#define MPL3115A2S_OUT_P_LSB (0x03) // Pressure Data Out LSB

#define MPL3115A2S_OUT_T_MSB (0x04) // Temperature Data Out MSB
#define MPL3115A2S_OUT_T_LSB (0x05) // Temperature Data Out LSB
#define MPL3115A2S_DR_STATUS (0x06) // Data Ready Status

// FIFO Registers
#define MPL3115A2S_F_SETUP (0x0F) // FIFO Setup
    #define MPL3115A2S_F_MODE_CIRCULAR (0b01 << 6) // Circular FIFO Mode
    #define MPL3115A2S_F_MODE_FIFO (0b10 << 6) // FIFO Mode Stop on Full

// Control Registers
#define MPL3115A2S_CTRL_REG1 (0x26) // Control Register 1
    #define MPL3115A2S_CTRL_ALT_SHIFT (7) // Altimeter Mode Shift
    #define MPL3115A2S_CTRL_OSR_SHIFT (3) // Oversampling Ratio Shift
    #define MPL3115A2S_CTRL_RST (0b1 << 2) // Software Reset
    #define MPL3115A2S_CTRL_OST (0b1 << 1) // One-Shot
    #define MPL3115A2S_CTRL_ACTIVE (0b1) // Active Mode Enable (from Standby)

#define MPL3115A2S_PT_DATA_CFG (0x13) // PT Data Configuration
    #define MPL3115A2S_PT_DATA_CFG_DREM_SHIFT (2) // Data Ready Event Mode Shift
    #define MPL3115A2S_PT_DATA_CFG_PDEFE_SHIFT (1) // Pressure Data Event Flag Enable Shift
    #define MPL3115A2S_PT_DATA_CFG_TDEFE_SHIFT (0) // Temperature Data Event Flag Enable Shift
    #define MPL3115A2S_PT_DATA_CFG_EN_ALL (0b111) // Enable All Flags

#define MPL3115A2S_CTRL_REG3 (0x28) // Control Register 3
    #define MPL3115A2S_CTRL_REG3_IPOL1_SHIFT (5) // Interrupt 1 Polarity Shift
    #define MPL3115A2S_CTRL_REG3_PP_OD1_SHIFT (4) // Interrupt 1 Open Drain Shift
    #define MPL3115A2S_CTRL_REG3_IPOL2_SHIFT (1) // Interrupt 2 Polarity Shift
    #define MPL3115A2S_CTRL_REG3_PP_OD2_SHIFT (0) // Interrupt 2 Open Drain Shift

#define MPL3115A2S_CTRL_REG4 (0x29) // Control Register 4
    #define MPL3115A2S_CTRL_REG4_INT_EN_DRDY_SHIFT (7) // Data Ready Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_FIFO_SHIFT (6) // FIFO Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_PW_SHIFT (5) // Pressure Window Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_TW_SHIFT (4) // Temperature Window Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_PTH_SHIFT (3) // Pressure Threshold Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_TTH_SHIFT (2) // Temperature Threshold Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_PCHG_SHIFT (1) // Pressure Change Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_TCHG_SHIFT (0) // Temperature Change Interrupt Enable Shift

#define MPL3115A2S_WHO_AM_I (0x0C) // Device ID Register
    #define MPL3115A2S_WHO_AM_I_ID (0xC4) // Device ID

#define I2C_7B_ADDR_TO_8B_W(addr) ((addr << 1) & 0xFE)
#define I2C_WRITE_TO_READ_ADDR(addr) (addr | 0x01)
#define I2C2_TIMEOUT 100

/* Enums and Structs ---------------------------------------------------------*/
// Driver Operation Status, Used as Function Return Values
typedef enum {
    MPL3115A2S_OK = 0,
    MPL_OK = MPL3115A2S_OK,
    MPL_SUCCESS = MPL3115A2S_OK,

    MPL3115A2S_ERR = 1,
    MPL_ERR = MPL3115A2S_ERR,
    MPL_INVALID_SETTING = MPL3115A2S_ERR,

    MPL3115A2S_NOT_INIT = 2,
    MPL_NOT_INIT = MPL3115A2S_NOT_INIT,

    MPL3115A2S_INVALID_ID = 3,
    MPL_INVALID_ID = MPL3115A2S_INVALID_ID,

    MPL_DATA_NOT_READY = 4,
} MPL_OP_STATUS;

// Control Register 1 Options (CTRL_REG1 = 0x26)
typedef enum {
    MPL3115A2S_CTRL_OS128 = 0b111,
    MPL3115A2S_CTRL_OS64 = 0b110,
    MPL3115A2S_CTRL_OS32 = 0b101,
    MPL3115A2S_CTRL_OS16 = 0b100,
    MPL3115A2S_CTRL_OS8 = 0b011,
    MPL3115A2S_CTRL_OS4 = 0b010,
    MPL3115A2S_CTRL_OS2 = 0b001,
    MPL3115A2S_CTRL_OS1 = 0b000
} MPL3115A2S_CTRL_OSR;

typedef struct {
    uint8_t altimeter_mode : 1; // 1 = Altimeter Mode, 0 = Barometer Mode
    MPL3115A2S_CTRL_OSR os_ratio : 3; // Oversampling Ratio 3-bit value, OS = 2^(os_ratio)
} MPL3115A2S_Config; 

// Sensor Data Register Options (PT_DATA_CFG = 0x13)
typedef struct {
    uint8_t enable_all_flags : 1; // 1 = Enable All Flags (ignore options below)

    uint8_t data_ready_event_mode : 1; // 1 = Data Ready Event Mode, 0 = Event Detection Disabled
    uint8_t pressure_data_event_flag : 1; // 1 = Enable Pressure Data Event Flag
    uint8_t temperature_data_event_flag : 1; // 1 = Enable Temperature Data Event Flag
} MPL3115A2S_Data_Config;

// Interrupt Control Register Options (CTRL_REG3,4 = 0x28,0x29)
typedef struct {
    uint8_t int1_active_high : 1; // 1 = Active High, 0 = Active Low
    uint8_t int1_open_drain : 1;  // 1 = Open Drain, 0 = Internal Pullup

    uint8_t int2_active_high : 1; // 1 = Active High, 0 = Active Low
    uint8_t int2_open_drain : 1;  // 1 = Open Drain, 0 = Internal Pullup

    uint8_t data_ready_it_enable : 1; // 1 = Data Ready Interrupt Enable
    uint8_t fifo_it_enable : 1; // 1 = FIFO Interrupt Enable

    uint8_t pressure_window_it_enable : 1; // 1 = Pressure Window Interrupt Enable
    uint8_t temperature_window_it_enable : 1; // 1 = Temperature Window Interrupt Enable
    uint8_t pressure_threshold_it_enable : 1; // 1 = Pressure Threshold Interrupt Enable
    uint8_t temperature_threshold_it_enable : 1; // 1 = Temperature Threshold Interrupt Enable
    uint8_t pressure_change_it_enable : 1; // 1 = Pressure Change Interrupt Enable
    uint8_t temperature_change_it_enable : 1; // 1 = Temperature Change Interrupt Enable
} MPL3115A2S_Int_Config; 

/* Initialization Functions -----------------------------------------------------------------*/
/**
 * @brief Initialize the MPL3115A2S Sensor with the given configurations
 *  Performs WHO_AM_I check, then sets up the sensor with the given configurations
 * 
 * @param cfg MPL3115A2S_Config
 *  Control register 1 configuration, altimeter mode select and oversampling ratio
 * @param data_cfg MPL3115A2S_Data_Config 
 *  Data configuration, data ready event mode and data event flag enables
 * @param int_cfg MPL3115A2S_Int_Config
 *  Interrupt configuration, interrupt polarity, open drain, and interrupt enables
 * @return MPL_OP_STATUS 
 */
MPL_OP_STATUS MPL3115A2S_Init(MPL3115A2S_Config* const cfg, MPL3115A2S_Data_Config* const data_cfg, MPL3115A2S_Int_Config* const int_cfg);

/* Read Data Functions (combination, float) -----------------------------------------*/
/**
 * @brief Reads pressure or altitude, and temperature data as floats
 * Note: The sensor will return the data based on the mode it is in (e.g. Pressure Mode or Altitude Mode)
 *  - This function performs a STATUS register check to determine if data is ready
 *  - This function drives the process described in page 13 of the data sheet
 * 
 * @param pres_alt Pointer to value to store pressure or altitude data
 * @param temp Pointer to value to store temperature data
 * @return MPL_OP_STATUS 
 */
MPL_OP_STATUS MPL3115A2S_ReadDataPolling(float* pres_alt, float* temp);

/**
 * @brief Reads pressure or altitude, and temperature data as floats using the MPL interrupt pins
 *        and should be called after the interrupt is triggered
 * Note: The sensor will return the data based on the mode it is in (e.g. Pressure Mode or Altitude Mode)
 * - This function performs a INT_SOURCE register check to determine if data is ready
 * - This function drives the process described in page 14 of the data sheet
 * 
 * @param pres_alt Pointer to value to store pressure or altitude data
 * @param temp Pointer to value to store temperature data
 * @return MPL_OP_STATUS
 */
MPL_OP_STATUS MPL3115A2S_ReadDataExtInterrupt(float* pres_alt, float* temp);

/* Read Data Functions (float) ---------------------------------------------------------------*/
/**
 * @brief Reads pressure data as a float
 * Note: The sensor must be in Pressure Mode (not Altitude Mode) to read correct values
 * 
 * @param pressure Pointer to value to store data
 * @return MPL_OP_STATUS 
 */
MPL_OP_STATUS MPL3115A2S_ReadPressure(float* pressure);

/**
 * @brief Reads altitude data as a float
 * Note: The sensor must be in Altitude Mode (not Pressure Mode) to read correct values
 * 
 * @param altitude Pointer to value to store data
 * @return MPL_OP_STATUS 
 */
MPL_OP_STATUS MPL3115A2S_ReadAltitude(float* altitude);

/**
 * @brief Reads temperature data as a float
 * 
 * @param temperature Pointer to value to store data
 * @return MPL_OP_STATUS
 */
MPL_OP_STATUS MPL3115A2S_ReadTemperature(float* temperature);

/* Read Data Functions (integer) ------------------------------------------------------------*/
/**
 * @brief Reads pressure data as a uint32_t
 * Note: The sensor must be in Pressure Mode (not Altitude Mode) to read correct values
 * 
 * @param multiplier Multiplier for integer value
 *  e.g. With multiplier 1, 101325.25 Pa of pressure, the value read would be 101325
 *       With multiplier 1000, the value read would be 101325250
 * @param pressure Pointer to value to store data
 * @return MPL_OP_STATUS 
 */
MPL_OP_STATUS MPL3115A2S_ReadPressureInt(uint32_t* pressure, uint16_t multiplier);

/**
 * @brief Reads altitude data as a int32_t
 * Note: The sensor must be in Altitude Mode (not Pressure Mode) to read correct values
 * 
 * @param multiplier Multiplier for integer value
 * e.g. With multiplier 1, 12345.5 m of altitude, the value read would be 12345
 *      With multiplier 1000, the value read would be 12345500
 * @param altitude Pointer to value to store data
 * @return MPL_OP_STATUS 
 */
MPL_OP_STATUS MPL3115A2S_ReadAltitudeInt(int32_t* altitude, uint16_t multiplier);

/**
 * @brief Reads temperature data as a int32_t
 * 
 * @param multiplier Multiplier for integer value
 * e.g. With multiplier 1, 123.25 C of temperature, the value read would be 123
 *      With multiplier 1000, the value read would be 123250
 * @param temperature Pointer to value to store data
 * @return MPL_OP_STATUS
 */
MPL_OP_STATUS MPL3115A2S_ReadTemperatureInt(int32_t* temperature, uint16_t multiplier);

/* Control Functions -----------------------------------------------------------------------*/
/**
 * @brief Software Reset the MPL3115A2S Sensor
 *  Note: All register values will be reset to default, must re-init after reset
 * 
 * @return MPL_OP_STATUS 
 */
MPL_OP_STATUS MPL3115A2S_Reset();

/**
 * @brief WHOAMI Check for MPL3115A2S Sensor
 * 
 * @param status 
 * @return MPL_OP_STATUS 
 */
MPL_OP_STATUS MPL3115A2S_CheckDeviceID();

/**
 * @brief Check if the MPL3115A2S has data ready through the STATUS register
 * 
 * @return MPL_OP_STATUS 
 *  MPL_DATA_NOT_READY if data is not ready
 *  MPL_OK if data is ready
 */
MPL_OP_STATUS MPL3115A2S_DataReady();

/**
 * @brief Read interrupt source register 
 * 
 * @param int_source Pointer to store the interrupt source
 * @return MPL_OP_STATUS
 */
MPL_OP_STATUS MPL3115A2S_ReadIntSource(uint8_t* int_source);

/**
 * @brief Check if the MPL3115A2S has data ready through the interrupt source register
 * 
 * @return MPL_OP_STATUS 
 *  MPL_DATA_NOT_READY if data is not ready
 *  MPL_OK if data is ready
 */
MPL_OP_STATUS MPL3115A2S_DataReadyIT();

/**
 * @brief Check if the MPL3115A2S is in Altimeter Mode
 * 
 * @return MPL_OP_STATUS
 * MPL3115A2S_OK if sensor is in Altimeter Mode
 * MPL3115A2S_INVALID_SETTING if sensor is in Barometer Mode
 * MPL_ERR if value was read, but state mismatched with stored value
 */
MPL_OP_STATUS MPL3115A2S_IsAltimeterMode();

#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_MPL3115A2S_DRIVER_H_ */
