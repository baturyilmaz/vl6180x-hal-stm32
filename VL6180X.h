/*
 * vl6180.h
 *
 *  Created on: Oct 21, 2024
 *      Author: batur.arslan
 */

#ifndef INC_VL6180_H_
#define INC_VL6180_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h" // Adjust this for your specific STM32 family

// Error codes
#define VL6180X_ERROR_NONE          0   // No error; Valid measurement
#define VL6180X_ERROR_SYSERR_1      1   // System error; VCSEL Continuity Test; No measurement possible
#define VL6180X_ERROR_SYSERR_2      2   // System error; VCSEL Watchdog Test; No measurement possible
#define VL6180X_ERROR_SYSERR_3      3   // System error; VCSEL Watchdog; No measurement possible
#define VL6180X_ERROR_SYSERR_4      4   // System error; PLL1 Lock; No measurement possible
#define VL6180X_ERROR_SYSERR_5      5   // System error; PLL2 Lock; No measurement possible
#define VL6180X_ERROR_ECEFAIL       6   // Early Convergence Estimate; Check fail
#define VL6180X_ERROR_NOCONVERGE    7   // Max convergence; System didn't converge before the specified time limit
#define VL6180X_ERROR_RANGEIGNORE   8   // Range ignore; No Target Ignore; Ignore threshold check failed
#define VL6180X_ERROR_SNR           11  // Max Signal To Noise Ratio; Ambient conditions too high
#define VL6180X_ERROR_RAWUFLOW      12  // Raw Range underflow; Target too close
#define VL6180X_ERROR_RAWOFLOW      13  // Raw Range overflow; Target too far
#define VL6180X_ERROR_RANGEUFLOW    14  // Range underflow; Target too close
#define VL6180X_ERROR_RANGEOFLOW    15  // Range overflow; Target too far

// Register addresses
typedef enum {
    IDENTIFICATION__MODEL_ID              = 0x000,
    IDENTIFICATION__MODEL_REV_MAJOR       = 0x001,
    IDENTIFICATION__MODEL_REV_MINOR       = 0x002,
    IDENTIFICATION__MODULE_REV_MAJOR      = 0x003,
    IDENTIFICATION__MODULE_REV_MINOR      = 0x004,
    IDENTIFICATION__DATE_HI               = 0x006,
    IDENTIFICATION__DATE_LO               = 0x007,
    IDENTIFICATION__TIME                  = 0x008,

    SYSTEM__MODE_GPIO0                    = 0x010,
    SYSTEM__MODE_GPIO1                    = 0x011,
    SYSTEM__HISTORY_CTRL                  = 0x012,
    SYSTEM__INTERRUPT_CONFIG_GPIO         = 0x014,
    SYSTEM__INTERRUPT_CLEAR               = 0x015,
    SYSTEM__FRESH_OUT_OF_RESET            = 0x016,
    SYSTEM__GROUPED_PARAMETER_HOLD        = 0x017,

    SYSRANGE__START                       = 0x018,
    SYSRANGE__THRESH_HIGH                 = 0x019,
    SYSRANGE__THRESH_LOW                  = 0x01A,
    SYSRANGE__INTERMEASUREMENT_PERIOD     = 0x01B,
    SYSRANGE__MAX_CONVERGENCE_TIME        = 0x01C,
    SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E,
    SYSRANGE__CROSSTALK_VALID_HEIGHT      = 0x021,
    SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  = 0x022,
    SYSRANGE__PART_TO_PART_RANGE_OFFSET   = 0x024,
    SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   = 0x025,
    SYSRANGE__RANGE_IGNORE_THRESHOLD      = 0x026,
    SYSRANGE__MAX_AMBIENT_LEVEL_MULT      = 0x02C,
    SYSRANGE__RANGE_CHECK_ENABLES         = 0x02D,
    SYSRANGE__VHV_RECALIBRATE             = 0x02E,
    SYSRANGE__VHV_REPEAT_RATE             = 0x031,

    SYSALS__START                         = 0x038,
    SYSALS__THRESH_HIGH                   = 0x03A,
    SYSALS__THRESH_LOW                    = 0x03C,
    SYSALS__INTERMEASUREMENT_PERIOD       = 0x03E,
    SYSALS__ANALOGUE_GAIN                 = 0x03F,
    SYSALS__INTEGRATION_PERIOD            = 0x040,

    RESULT__RANGE_STATUS                  = 0x04D,
    RESULT__ALS_STATUS                    = 0x04E,
    RESULT__INTERRUPT_STATUS_GPIO         = 0x04F,
    RESULT__ALS_VAL                       = 0x050,
    RESULT__HISTORY_BUFFER_0              = 0x052,
    RESULT__HISTORY_BUFFER_1              = 0x054,
    RESULT__HISTORY_BUFFER_2              = 0x056,
    RESULT__HISTORY_BUFFER_3              = 0x058,
    RESULT__HISTORY_BUFFER_4              = 0x05A,
    RESULT__HISTORY_BUFFER_5              = 0x05C,
    RESULT__HISTORY_BUFFER_6              = 0x05E,
    RESULT__HISTORY_BUFFER_7              = 0x060,
    RESULT__RANGE_VAL                     = 0x062,
    RESULT__RANGE_RAW                     = 0x064,
    RESULT__RANGE_RETURN_RATE             = 0x066,
    RESULT__RANGE_REFERENCE_RATE          = 0x068,
    RESULT__RANGE_RETURN_SIGNAL_COUNT     = 0x06C,
    RESULT__RANGE_REFERENCE_SIGNAL_COUNT  = 0x070,
    RESULT__RANGE_RETURN_AMB_COUNT        = 0x074,
    RESULT__RANGE_REFERENCE_AMB_COUNT     = 0x078,
    RESULT__RANGE_RETURN_CONV_TIME        = 0x07C,
    RESULT__RANGE_REFERENCE_CONV_TIME     = 0x080,

    RANGE_SCALER                          = 0x096,

    READOUT__AVERAGING_SAMPLE_PERIOD      = 0x10A,
    FIRMWARE__BOOTUP                      = 0x119,
    FIRMWARE__RESULT_SCALER               = 0x120,
    I2C_SLAVE__DEVICE_ADDRESS             = 0x212,
    INTERLEAVED_MODE__ENABLE              = 0x2A3,
} vl6180x_reg_t;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t last_status;
    uint8_t address;
    uint8_t scaling;
    int8_t ptp_offset;
    uint16_t io_timeout;
    bool did_timeout;
} vl6180x_t;

// Function prototypes
void vl6180x_init(vl6180x_t *dev, I2C_HandleTypeDef *hi2c);
void vl6180x_configure_default(vl6180x_t *dev);
void vl6180x_set_address(vl6180x_t *dev, uint8_t new_addr);
void vl6180x_write_reg(vl6180x_t *dev, uint16_t reg, uint8_t value);
void vl6180x_write_reg_16bit(vl6180x_t *dev, uint16_t reg, uint16_t value);
void vl6180x_write_reg_32bit(vl6180x_t *dev, uint16_t reg, uint32_t value);
uint8_t vl6180x_read_reg(vl6180x_t *dev, uint16_t reg);
uint16_t vl6180x_read_reg_16bit(vl6180x_t *dev, uint16_t reg);
uint32_t vl6180x_read_reg_32bit(vl6180x_t *dev, uint16_t reg);
void vl6180x_set_scaling(vl6180x_t *dev, uint8_t new_scaling);
uint8_t vl6180x_read_range_single(vl6180x_t *dev);
uint16_t vl6180x_read_range_single_millimeters(vl6180x_t *dev);
uint16_t vl6180x_read_ambient_single(vl6180x_t *dev);
void vl6180x_start_range_continuous(vl6180x_t *dev, uint16_t period);
void vl6180x_start_ambient_continuous(vl6180x_t *dev, uint16_t period);
void vl6180x_start_interleaved_continuous(vl6180x_t *dev, uint16_t period);
void vl6180x_stop_continuous(vl6180x_t *dev);
uint8_t vl6180x_read_range_continuous(vl6180x_t *dev);
uint16_t vl6180x_read_range_continuous_millimeters(vl6180x_t *dev);
uint16_t vl6180x_read_ambient_continuous(vl6180x_t *dev);
void vl6180x_set_timeout(vl6180x_t *dev, uint16_t timeout);
uint16_t vl6180x_get_timeout(vl6180x_t *dev);
bool vl6180x_timeout_occurred(vl6180x_t *dev);
uint8_t vl6180x_read_range_status(vl6180x_t *dev);

#endif /* INC_VL6180_H_ */
