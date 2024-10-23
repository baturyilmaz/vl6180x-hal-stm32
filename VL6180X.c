#include "vl6180.h"

extern uint32_t HAL_GetTick(void);

extern HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
extern HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

#define ADDRESS_DEFAULT 0x29
#define RANGE_SCALER_VALUES { 0, 253, 127, 84 }

void vl6180x_init(vl6180x_t *dev, I2C_HandleTypeDef *hi2c)
{
    dev->i2c = hi2c;
    dev->address = ADDRESS_DEFAULT;
    dev->scaling = 1;
    dev->ptp_offset = vl6180x_read_reg(dev, SYSRANGE__PART_TO_PART_RANGE_OFFSET);
    dev->io_timeout = 0;
    dev->did_timeout = false;

    if (vl6180x_read_reg(dev, SYSTEM__FRESH_OUT_OF_RESET) == 1)
    {
        // Mandatory : private registers
        // an4545-vl6180x-basic-ranging-application-note-stmicroelectronics.pdf
        vl6180x_write_reg(dev, 0x207, 0x01);
        vl6180x_write_reg(dev, 0x208, 0x01);
        vl6180x_write_reg(dev, 0x096, 0x00);
        vl6180x_write_reg(dev, 0x097, 0xFD);
        vl6180x_write_reg(dev, 0x0E3, 0x01);
        vl6180x_write_reg(dev, 0x0E4, 0x03);
        vl6180x_write_reg(dev, 0x0E5, 0x02);
        vl6180x_write_reg(dev, 0x0E6, 0x01);
        vl6180x_write_reg(dev, 0x0E7, 0x03);
        vl6180x_write_reg(dev, 0x0F5, 0x02);
        vl6180x_write_reg(dev, 0x0D9, 0x05);
        vl6180x_write_reg(dev, 0x0DB, 0xCE);
        vl6180x_write_reg(dev, 0x0DC, 0x03);
        vl6180x_write_reg(dev, 0x0DD, 0xF8);
        vl6180x_write_reg(dev, 0x09F, 0x00);
        vl6180x_write_reg(dev, 0x0A3, 0x3C);
        vl6180x_write_reg(dev, 0x0B7, 0x00);
        vl6180x_write_reg(dev, 0x0BB, 0x3C);
        vl6180x_write_reg(dev, 0x0B2, 0x09);
        vl6180x_write_reg(dev, 0x0CA, 0x09);
        vl6180x_write_reg(dev, 0x198, 0x01);
        vl6180x_write_reg(dev, 0x1B0, 0x17);
        vl6180x_write_reg(dev, 0x1AD, 0x00);
        vl6180x_write_reg(dev, 0x0FF, 0x05);
        vl6180x_write_reg(dev, 0x100, 0x05);
        vl6180x_write_reg(dev, 0x199, 0x05);
        vl6180x_write_reg(dev, 0x1A6, 0x1B);
        vl6180x_write_reg(dev, 0x1AC, 0x3E);
        vl6180x_write_reg(dev, 0x1A7, 0x1F);
        vl6180x_write_reg(dev, 0x030, 0x00);

        vl6180x_write_reg(dev, SYSTEM__FRESH_OUT_OF_RESET, 0);
    }
    else
    {
        uint16_t s = vl6180x_read_reg_16bit(dev, RANGE_SCALER);
        const uint16_t ScalerValues[] = RANGE_SCALER_VALUES;

        if      (s == ScalerValues[3]) { dev->scaling = 3; }
        else if (s == ScalerValues[2]) { dev->scaling = 2; }
        else                           { dev->scaling = 1; }

        dev->ptp_offset *= dev->scaling;
    }
}

void vl6180x_configure_default(vl6180x_t *dev)
{
    vl6180x_write_reg(dev, READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);
    vl6180x_write_reg(dev, SYSALS__ANALOGUE_GAIN, 0x46);
    vl6180x_write_reg(dev, SYSRANGE__VHV_REPEAT_RATE, 0xFF);
    vl6180x_write_reg_16bit(dev, SYSALS__INTEGRATION_PERIOD, 0x0063);
    vl6180x_write_reg(dev, SYSRANGE__VHV_RECALIBRATE, 0x01);
    vl6180x_write_reg(dev, SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);
    vl6180x_write_reg(dev, SYSALS__INTERMEASUREMENT_PERIOD, 0x31);
    vl6180x_write_reg(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);
    vl6180x_write_reg(dev, SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);
    vl6180x_write_reg(dev, INTERLEAVED_MODE__ENABLE, 0);
    vl6180x_set_scaling(dev, 1);
}

void vl6180x_set_address(vl6180x_t *dev, uint8_t new_addr)
{
    vl6180x_write_reg(dev, I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
    // if (dev->last_status == 0) {
    dev->address = new_addr;
    // }
}

void vl6180x_write_reg(vl6180x_t *dev, uint16_t reg, uint8_t value)
{
    uint8_t buffer[3];
    buffer[0] = (reg >> 8) & 0xFF;
    buffer[1] = reg & 0xFF;
    buffer[2] = value;

    dev->last_status = HAL_I2C_Master_Transmit(dev->i2c, dev->address << 1, buffer, 3, HAL_MAX_DELAY);
    /*
        dev->address << 1
        This left shift is necessary because the STM32 HAL library expects the 7-bit I2C address
        to be combined with the R/W bit as part of the 8-bit format expected by the I2C hardware.
    */
}

void vl6180x_write_reg_16bit(vl6180x_t *dev, uint16_t reg, uint16_t value)
{
    uint8_t buffer[4];
    buffer[0] = (reg >> 8) & 0xFF;
    buffer[1] = reg & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;

    dev->last_status = HAL_I2C_Master_Transmit(dev->i2c, dev->address << 1, buffer, 4, HAL_MAX_DELAY);
}

void vl6180x_write_reg_32bit(vl6180x_t *dev, uint16_t reg, uint32_t value)
{
    uint8_t buffer[6];
    buffer[0] = (reg >> 8) & 0xFF;
    buffer[1] = reg & 0xFF;
    buffer[2] = (value >> 24) & 0xFF;
    buffer[3] = (value >> 16) & 0xFF;
    buffer[4] = (value >> 8) & 0xFF;
    buffer[5] = value & 0xFF;

    dev->last_status = HAL_I2C_Master_Transmit(dev->i2c, dev->address << 1, buffer, 6, HAL_MAX_DELAY);
}

uint8_t vl6180x_read_reg(vl6180x_t *dev, uint16_t reg)
{
    uint8_t value;
    uint8_t reg_addr[2];
    reg_addr[0] = (reg >> 8) & 0xFF;
    reg_addr[1] = reg & 0xFF;

    dev->last_status = HAL_I2C_Master_Transmit(dev->i2c, dev->address << 1, reg_addr, 2, HAL_MAX_DELAY);
    dev->last_status = HAL_I2C_Master_Receive(dev->i2c, dev->address << 1, &value, 1, HAL_MAX_DELAY);

    return value;
}

uint16_t vl6180x_read_reg_16bit(vl6180x_t *dev, uint16_t reg)
{
    uint8_t value[2];
    uint8_t reg_addr[2];
    reg_addr[0] = (reg >> 8) & 0xFF;
    reg_addr[1] = reg & 0xFF;

    dev->last_status = HAL_I2C_Master_Transmit(dev->i2c, dev->address << 1, reg_addr, 2, HAL_MAX_DELAY);
    dev->last_status = HAL_I2C_Master_Receive(dev->i2c, dev->address << 1, value, 2, HAL_MAX_DELAY);

    return (uint16_t)value[0] << 8 | value[1];
}

uint32_t vl6180x_read_reg_32bit(vl6180x_t *dev, uint16_t reg)
{
    uint8_t value[4];
    uint8_t reg_addr[2];
    reg_addr[0] = (reg >> 8) & 0xFF;
    reg_addr[1] = reg & 0xFF;

    dev->last_status = HAL_I2C_Master_Transmit(dev->i2c, dev->address << 1, reg_addr, 2, HAL_MAX_DELAY);
    dev->last_status = HAL_I2C_Master_Receive(dev->i2c, dev->address << 1, value, 4, HAL_MAX_DELAY);

    return (uint32_t)value[0] << 24 | (uint32_t)value[1] << 16 | (uint32_t)value[2] << 8 | value[3];
}

void vl6180x_set_scaling(vl6180x_t *dev, uint8_t new_scaling)
{
    uint8_t const DefaultCrosstalkValidHeight = 20; // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT
    const uint16_t ScalerValues[] = RANGE_SCALER_VALUES;

    if (new_scaling < 1 || new_scaling > 3) { return; }

    dev->scaling = new_scaling;
    vl6180x_write_reg_16bit(dev, RANGE_SCALER, ScalerValues[dev->scaling]);

    vl6180x_write_reg(dev, SYSRANGE__PART_TO_PART_RANGE_OFFSET, dev->ptp_offset / dev->scaling);

    vl6180x_write_reg(dev, SYSRANGE__CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / dev->scaling);

    uint8_t rce = vl6180x_read_reg(dev, SYSRANGE__RANGE_CHECK_ENABLES);
    vl6180x_write_reg(dev, SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (dev->scaling == 1));
}

uint8_t vl6180x_read_range_single(vl6180x_t *dev)
{
    vl6180x_write_reg(dev, SYSRANGE__START, 0x01);
    return vl6180x_read_range_continuous(dev);
}

uint16_t vl6180x_read_range_single_millimeters(vl6180x_t *dev)
{
    return (uint16_t)dev->scaling * vl6180x_read_range_single(dev);
}

uint16_t vl6180x_read_ambient_single(vl6180x_t *dev)
{
    vl6180x_write_reg(dev, SYSALS__START, 0x01);
    return vl6180x_read_ambient_continuous(dev);
}

void vl6180x_start_range_continuous(vl6180x_t *dev, uint16_t period)
{
    int16_t period_reg = (period / 10) - 1;
    period_reg = (period_reg < 0) ? 0 : ((period_reg > 254) ? 254 : period_reg);

    vl6180x_write_reg(dev, SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg);
    vl6180x_write_reg(dev, SYSRANGE__START, 0x03);
}

void vl6180x_start_ambient_continuous(vl6180x_t *dev, uint16_t period)
{
    int16_t period_reg = (period / 10) - 1;
    period_reg = (period_reg < 0) ? 0 : ((period_reg > 254) ? 254 : period_reg);

    vl6180x_write_reg(dev, SYSALS__INTERMEASUREMENT_PERIOD, period_reg);
    vl6180x_write_reg(dev, SYSALS__START, 0x03);
}

void vl6180x_start_interleaved_continuous(vl6180x_t *dev, uint16_t period)
{
    int16_t period_reg = (period / 10) - 1;
    period_reg = (period_reg < 0) ? 0 : ((period_reg > 254) ? 254 : period_reg);

    vl6180x_write_reg(dev, INTERLEAVED_MODE__ENABLE, 1);
    vl6180x_write_reg(dev, SYSALS__INTERMEASUREMENT_PERIOD, period_reg);
    vl6180x_write_reg(dev, SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg);
    vl6180x_write_reg(dev, SYSALS__START, 0x03);
    vl6180x_write_reg(dev, SYSRANGE__START, 0x03);
}

void vl6180x_stop_continuous(vl6180x_t *dev)
{
    vl6180x_write_reg(dev, SYSRANGE__START, 0x01);
    vl6180x_write_reg(dev, SYSALS__START, 0x01);
    vl6180x_write_reg(dev, INTERLEAVED_MODE__ENABLE, 0);
}

uint8_t vl6180x_read_range_continuous(vl6180x_t *dev)
{
    uint32_t start = HAL_GetTick();
    while ((vl6180x_read_reg(dev, RESULT__INTERRUPT_STATUS_GPIO) & 0x07) != 0x04)
    {
        if (dev->io_timeout > 0 && ((HAL_GetTick() - start) > dev->io_timeout))
        {
            dev->did_timeout = true;
            return 255;
        }
    }

    uint8_t range = vl6180x_read_reg(dev, RESULT__RANGE_VAL);
    vl6180x_write_reg(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);

    return range;
}

uint16_t vl6180x_read_range_continuous_millimeters(vl6180x_t *dev)
{
    return (uint16_t)dev->scaling * vl6180x_read_range_continuous(dev);
}

uint16_t vl6180x_read_ambient_continuous(vl6180x_t *dev)
{
    uint32_t start = HAL_GetTick();
    while ((vl6180x_read_reg(dev, RESULT__INTERRUPT_STATUS_GPIO) & 0x38) != 0x20)
    {
        if (dev->io_timeout > 0 && ((HAL_GetTick() - start) > dev->io_timeout))
        {
            dev->did_timeout = true;
            return 0;
        }
    }

    uint16_t ambient = vl6180x_read_reg_16bit(dev, RESULT__ALS_VAL);
    vl6180x_write_reg(dev, SYSTEM__INTERRUPT_CLEAR, 0x02);

    return ambient;
}

void vl6180x_set_timeout(vl6180x_t *dev, uint16_t timeout)
{
    dev->io_timeout = timeout;
}

uint16_t vl6180x_get_timeout(vl6180x_t *dev)
{
    return dev->io_timeout;
}

bool vl6180x_timeout_occurred(vl6180x_t *dev)
{
    bool tmp = dev->did_timeout;
    dev->did_timeout = false;
    return tmp;
}

uint8_t vl6180x_read_range_status(vl6180x_t *dev)
{
    return (vl6180x_read_reg(dev, RESULT__RANGE_STATUS) >> 4);
}
