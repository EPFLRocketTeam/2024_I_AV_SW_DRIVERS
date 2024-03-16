#include "BMP.h"



static int8_t power_up_check(struct bmp5_dev *dev);
static int8_t check_deepstandby_mode(enum bmp5_powermode *powermode, struct bmp5_dev *dev);
static int8_t set_deep_standby_mode(struct bmp5_dev *dev);
static int8_t set_power_mode(enum bmp5_powermode powermode, struct bmp5_dev *dev);
static int8_t validate_chip_id(uint8_t chip_id, struct bmp5_dev *dev);
static int8_t get_nvm_status(uint8_t *nvm_status, struct bmp5_dev *dev);
static uint32_t power(uint8_t base, uint8_t resolution);
static int8_t set_standby_mode(struct bmp5_dev *dev);
static int8_t null_ptr_check(const struct bmp5_dev *dev);
void read_bmp(BMP581 &sens)
{
  bmp5_sensor_data bmpData = {0, 0};// this is a structure with 2 variable pressure and temperature
  int8_t bmpErr = sens.getSensorData(&bmpData);
  // Read and print BMP581 sensor data if available
  if (bmpErr == BMP5_OK)
  {
    Serial.print("BMP581 - Temperature (C): ");
    Serial.print(bmpData.temperature);
    Serial.print("\t\tPressure (Pa): ");
    Serial.println(bmpData.pressure);
  }
  else
  {
    Serial.print("Error getting BMP581 data! Error code: ");
    Serial.println(bmpErr);
  }
 
}
//done
BMP581::BMP581()
{
    // Nothing to do
}
//done
///int8_t BMP581::beginI2C(uint8_t address, TwoWire& wirePort)
int8_t BMP581::beginI2C(uint8_t address, WIRE_TEENSY& wirePort)
{
    // Check whether address is valid option
    if(address != BMP5_I2C_ADDR_PRIM && address != BMP5_I2C_ADDR_SEC)
    {
      
        // Invalid option, don't do anything
        return BMP5_E_INVALID_SETTING;
    }
    // Address is valid option
    interfaceData.i2cAddress = address;
    interfaceData.i2cPort = &wirePort;  
    // Set interface
    sensor.intf = BMP5_I2C_INTF;
    interfaceData.interface = BMP5_I2C_INTF;

    // Initialize sensor
    return begin();

}
//done
static int8_t set_standby_mode(struct bmp5_dev *dev)
{
    int8_t rslt;
    enum bmp5_powermode pwrmode;

    rslt = bmp5_get_power_mode(&pwrmode, dev);

    if (rslt == BMP5_OK)
    {
        if (pwrmode == BMP5_POWERMODE_DEEP_STANDBY)
        {
            rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
        }
    }

    return rslt;
}
//done
int8_t BMP581::begin()
{
    // Variable to track errors returned by API calls
    int8_t err = BMP5_OK;
    

    // Initialize the sensor
    err = init();

    if(err != BMP5_OK)
    {
        return err;
    }

    // Enable both pressure and temperature sensors
    err = enablePress(BMP5_ENABLE);

    if(err != BMP5_OK)
    {
        return err;
    }
    // Set to normal mode
    return setMode(BMP5_POWERMODE_NORMAL);

}
//done
int8_t BMP581::enablePress(uint8_t pressEnable)
{
    osrOdrConfig.press_en = pressEnable;
    return bmp5_set_osr_odr_press_config(&osrOdrConfig, &sensor);
}
//done
int8_t bmp5_set_osr_odr_press_config(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt = 0;

    /* Variable to set ODR and OSR config */
    uint8_t reg_data[2] = { 0 };

    if (osr_odr_press_cfg != NULL)
    {
        /* If ODR is set to a value higher than 5Hz then powermode is set as standby mode, as ODR value greater than 5HZ
         * without disabling deep-standby mode makes powermode invalid.
         * NOTE: Register value for 5Hz is greater compared to ODRs higher than it. Thus in this below condition odr
         * is checked whether less than 5Hz macro.
         */
        if (osr_odr_press_cfg->odr < BMP5_ODR_05_HZ)
        {
            rslt = set_standby_mode(dev);
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_regs(BMP5_REG_OSR_CONFIG, reg_data, 2, dev);

            if (rslt == BMP5_OK)
            {
                reg_data[0] = BMP5_SET_BITS_POS_0(reg_data[0], BMP5_TEMP_OS, osr_odr_press_cfg->osr_t);
                reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_PRESS_OS, osr_odr_press_cfg->osr_p);
                reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_PRESS_EN, osr_odr_press_cfg->press_en);
                reg_data[1] = BMP5_SET_BITSLICE(reg_data[1], BMP5_ODR, osr_odr_press_cfg->odr);

                /* Set ODR and OSR configuration */
                rslt = bmp5_set_regs(BMP5_REG_OSR_CONFIG, reg_data, 2, dev);
            }
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

//done
int8_t bmp5_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmp5_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMP5_OK) && (reg_data != NULL))
    {

        /* Read the data from the reg_addr */
        dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);

        if (dev->intf_rslt != BMP5_INTF_RET_SUCCESS)
        {
            /* Failure case */
            rslt = BMP5_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}
//done
int8_t bmp5_get_power_mode(enum bmp5_powermode *powermode, struct bmp5_dev *dev)
{
    int8_t rslt;
    uint8_t deep_dis;
    uint8_t reg_data;
    uint8_t pwrmode;

    if (powermode != NULL)
    {
        /* Read the power mode register */
        rslt = bmp5_get_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);

        if (rslt == BMP5_OK)
        {
            pwrmode = BMP5_GET_BITS_POS_0(reg_data, BMP5_POWERMODE);

            switch (pwrmode)
            {
                case BMP5_POWERMODE_STANDBY:

                    /* Getting deep disable status */
                    deep_dis = BMP5_GET_BITSLICE(reg_data, BMP5_DEEP_DISABLE);

                    /* Checking deepstandby status only when powermode is in standby mode */

                    /* If deep_dis = 0(BMP5_DEEP_ENABLED) then deepstandby mode is enabled.
                     * If deep_dis = 1(BMP5_DEEP_DISABLED) then deepstandby mode is disabled
                     */
                    if (deep_dis == BMP5_DEEP_ENABLED)
                    {
                        rslt = check_deepstandby_mode(powermode, dev);
                    }
                    else
                    {
                        *powermode = BMP5_POWERMODE_STANDBY;
                    }

                    break;
                case BMP5_POWERMODE_NORMAL:
                    *powermode = BMP5_POWERMODE_NORMAL;
                    break;
                case BMP5_POWERMODE_FORCED:
                    *powermode = BMP5_POWERMODE_FORCED;
                    break;
                case BMP5_POWERMODE_CONTINOUS:
                    *powermode = BMP5_POWERMODE_CONTINOUS;
                    break;
                default:
                    rslt = BMP5_E_INVALID_POWERMODE;
                    break;
            }
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}
//done
int8_t bmp5_set_power_mode(enum bmp5_powermode powermode, struct bmp5_dev *dev)
{
    int8_t rslt;
    enum bmp5_powermode lst_pwrmode;

    /* Existing power mode of the device is received in lst_pwrmode */
    rslt = bmp5_get_power_mode(&lst_pwrmode, dev);

    if (rslt == BMP5_OK)
    {
        /* If the sensor is not in standby mode set the device to
         *  standby mode.
         */
        if (lst_pwrmode != BMP5_POWERMODE_STANDBY)
        {
            /* Device should be set to standby before transiting to
             * forced mode or normal mode or continous mode.
             */
            rslt = set_power_mode(BMP5_POWERMODE_STANDBY, dev);


            if (rslt == BMP5_OK)
            {
                /* Give t_standby(as per data sheet) time for device to go into standby mode */
                dev->delay_us(BMP5_DELAY_US_STANDBY, dev->intf_ptr);
            }
        }

        /* Set the desired power mode */
        if (rslt == BMP5_OK)
        {
            switch (powermode)
            {
                case BMP5_POWERMODE_DEEP_STANDBY:
                    rslt = set_deep_standby_mode(dev);
                    break;
                case BMP5_POWERMODE_STANDBY:

                    /* Since switching between powermodes require sensor to be in standby mode
                     * it is performed above. So it is not explicitly performed here.
                     */
                    break;
                case BMP5_POWERMODE_NORMAL:
                case BMP5_POWERMODE_FORCED:
                case BMP5_POWERMODE_CONTINOUS:
                    rslt = set_power_mode(powermode, dev);
                    break;
                default:
                    rslt = BMP5_E_INVALID_POWERMODE;
                    break;
            }
        }
    }

    return rslt;
}
//done
static int8_t set_power_mode(enum bmp5_powermode powermode, struct bmp5_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    rslt = bmp5_get_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);

    if (rslt == BMP5_OK)
    {
        /* Setting deep_dis = 1(BMP5_DEEP_DISABLED) disables the deep standby mode */
        reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);

        reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_POWERMODE, powermode);

        rslt = bmp5_set_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);
    }

    return rslt;
}
//done
static int8_t set_deep_standby_mode(struct bmp5_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    rslt = bmp5_get_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);

    if (rslt == BMP5_OK)
    {
        /* Setting deep_dis = 0(BMP5_DEEP_ENABLED) enables the deep standby mode */
        reg_data = BMP5_SET_BIT_VAL_0(reg_data, BMP5_DEEP_DISABLE);

        /* Set ODR less then 5Hz - ODR used is 1Hz */
        reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_ODR, BMP5_ODR_01_HZ);

        /* Write the value to the odr config register(0x37) */
        rslt = bmp5_set_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_regs(BMP5_REG_DSP_IIR, &reg_data, 1, dev);

            if (rslt == BMP5_OK)
            {
                /* Set iir_t and iir_p as Bypass(0x00) */

                /* The register holds only iir_t and iir_p and the last 2 bits are reserved.
                 * Thus using the macro BMP5_IIR_BYPASS(0xC0) the register value is set as zero.
                 */
                reg_data = reg_data & BMP5_IIR_BYPASS;

                /* Write the value to the IIR register(0x31) */
                rslt = bmp5_set_regs(BMP5_REG_DSP_IIR, &reg_data, 1, dev);
            }
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_regs(BMP5_REG_FIFO_SEL, &reg_data, 1, dev);

            if (rslt == BMP5_OK)
            {
                /* Disable fifo frame selct */
                reg_data = BMP5_SET_BIT_VAL_0(reg_data, BMP5_FIFO_FRAME_SEL);

                /* Write the value to the fifo select register(0x18) */
                rslt = bmp5_set_regs(BMP5_REG_FIFO_SEL, &reg_data, 1, dev);
            }
        }
    }

    return rslt;
}
//done
int8_t bmp5_get_iir_config(struct bmp5_iir_config *iir_cfg, struct bmp5_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Variable to get IIR config */
    uint8_t reg_data[2];

    if (iir_cfg != NULL)
    {
        /* Get IIR configuration */
        rslt = bmp5_get_regs(BMP5_REG_DSP_CONFIG, reg_data, 2, dev);

        iir_cfg->shdw_set_iir_t = BMP5_GET_BITSLICE(reg_data[0], BMP5_SHDW_SET_IIR_TEMP);
        iir_cfg->shdw_set_iir_p = BMP5_GET_BITSLICE(reg_data[0], BMP5_SHDW_SET_IIR_PRESS);
        iir_cfg->iir_flush_forced_en = BMP5_GET_BITSLICE(reg_data[0], BMP5_IIR_FLUSH_FORCED_EN);

        iir_cfg->set_iir_t = BMP5_GET_BITS_POS_0(reg_data[1], BMP5_SET_IIR_TEMP);
        iir_cfg->set_iir_p = BMP5_GET_BITSLICE(reg_data[1], BMP5_SET_IIR_PRESS);
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}
//done
static int8_t check_deepstandby_mode(enum bmp5_powermode *powermode, struct bmp5_dev *dev)
{
    int8_t rslt;
    uint8_t fifo_frame_sel;
    struct bmp5_iir_config iir_cfg = { 0 };
    struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };

    rslt = bmp5_get_regs(BMP5_REG_FIFO_SEL, &fifo_frame_sel, 1, dev);
    fifo_frame_sel = BMP5_GET_BITS_POS_0(fifo_frame_sel, BMP5_FIFO_FRAME_SEL);

    if (rslt == BMP5_OK)
    {
        rslt = bmp5_get_osr_odr_press_config(&osr_odr_press_cfg, dev);

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_iir_config(&iir_cfg, dev);
        }
    }

    /* As per datasheet odr should be less than 5Hz. But register value for 5Hz is less than 4Hz and so,
     * thus in this below condition odr is checked whether greater than 5Hz macro.
     */
    if ((osr_odr_press_cfg.odr > BMP5_ODR_05_HZ) && (fifo_frame_sel == BMP5_DISABLE) &&
        (iir_cfg.set_iir_t == BMP5_IIR_FILTER_BYPASS) && (iir_cfg.set_iir_p == BMP5_IIR_FILTER_BYPASS))
    {
        *powermode = BMP5_POWERMODE_DEEP_STANDBY;
    }

    return rslt;
}
//done
int8_t bmp5_get_osr_odr_press_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Variable to store OSR and ODR config */
    uint8_t reg_data[2];

    if (osr_odr_press_cfg != NULL)
    {
        /* Get OSR and ODR configuration in burst read */
        rslt = bmp5_get_regs(BMP5_REG_OSR_CONFIG, reg_data, 2, dev);

        if (rslt == BMP5_OK)
        {
            osr_odr_press_cfg->osr_t = BMP5_GET_BITS_POS_0(reg_data[0], BMP5_TEMP_OS);
            osr_odr_press_cfg->osr_p = BMP5_GET_BITSLICE(reg_data[0], BMP5_PRESS_OS);
            osr_odr_press_cfg->press_en = BMP5_GET_BITSLICE(reg_data[0], BMP5_PRESS_EN);
            osr_odr_press_cfg->odr = BMP5_GET_BITSLICE(reg_data[1], BMP5_ODR);
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}
//done
int8_t BMP581::init()
{
    // Variable to track errors returned by API calls
    int8_t err = BMP5_OK;

    // Initialize config values
    osrOdrConfig = {0,0,0,0};
    fifo = {0,0,0,0,0,0,0,0,0};

    // Set helper function pointers
    sensor.read = readRegisters;
    sensor.write = writeRegisters;
    sensor.delay_us = usDelay;
    sensor.intf_ptr = &interfaceData;

    // Reset the sensor
    err = bmp5_soft_reset(&sensor);
    if(err != BMP5_OK)
    {
        return err;
    }

    // Initialize the sensor
    return bmp5_init(&sensor);
}
//done
int8_t bmp5_soft_reset(struct bmp5_dev *dev)
{
    int8_t rslt;
    uint8_t por_status;
    uint8_t data = BMP5_SOFT_RESET_CMD;

    /* Reset the device */
    rslt = bmp5_set_regs(BMP5_REG_CMD, &data, 1, dev);

    if (rslt == BMP5_OK)
    {
        /* Soft-reset execution takes 2 ms */
        dev->delay_us(BMP5_DELAY_US_SOFT_RESET, dev->intf_ptr);

#if 0 // Suggestion by Bosch to fix init problem with checking INT_STATUS.por bit
        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_interrupt_status(&por_status, dev);

            if (rslt == BMP5_OK)
            {
                if (por_status & BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE)
                {
                    rslt = BMP5_OK;
                }
                else
                {
                    rslt = BMP5_E_POR_SOFTRESET;
                }
            }
        }
#endif
    }

    (void)por_status; // Avoid compiler warning-as-error

    return rslt;
}
//done
int8_t bmp5_init(struct bmp5_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == BMP5_OK)
    {
        if (rslt == BMP5_OK)
        {
            /* Read chip_id */
            rslt = bmp5_get_regs(BMP5_REG_CHIP_ID, &chip_id, 1, dev);

            if (rslt == BMP5_OK)
            {
                if (chip_id != 0)
                {
                    /* Validate post power-up procedure */
                    rslt = power_up_check(dev);
                }
                else
                {
                    rslt = BMP5_E_INVALID_CHIP_ID;
                }

                if (rslt == BMP5_OK)
                {
                    rslt = validate_chip_id(chip_id, dev);
                }
            }
        }
    }

    return rslt;
}
//done
int8_t bmp5_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bmp5_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMP5_OK) && (reg_data != NULL))
    {

        if ((dev->intf == BMP5_I2C_INTF) )
        {
            /* Write the data to the reg_addr */
            dev->intf_rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr);
        }

        if (dev->intf_rslt != BMP5_INTF_RET_SUCCESS)
        {
            /* Failure case */
            rslt = BMP5_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}
//done
int8_t bmp5_get_interrupt_status(uint8_t *int_status, struct bmp5_dev *dev)
{
    int8_t rslt;

    if (int_status != NULL)
    {
        rslt = bmp5_get_regs(BMP5_REG_INT_STATUS, int_status, 1, dev);
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}
//done
static int8_t power_up_check(struct bmp5_dev *dev)
{
    int8_t rslt;
    uint8_t nvm_status;
    uint8_t por_status;

    rslt = get_nvm_status(&nvm_status, dev);

    if (rslt == BMP5_OK)
    {
        /* Check if nvm_rdy status = 1 and nvm_err status = 0 to proceed */
        if ((nvm_status & BMP5_INT_NVM_RDY) && (!(nvm_status & BMP5_INT_NVM_ERR)))
        {
            rslt = bmp5_get_interrupt_status(&por_status, dev);

            if (rslt == BMP5_OK)
            {
                /* Check if por/soft-reset complete status = 1 to proceed */
                if (por_status & BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE)
                {
                    rslt = BMP5_OK;
                }
                else
                {
                    rslt = BMP5_E_POWER_UP;
                }
            }
        }
        else
        {
            rslt = BMP5_E_POWER_UP;
        }
    }

    return rslt;
}
//done
static int8_t validate_chip_id(uint8_t chip_id, struct bmp5_dev *dev)
{
    int8_t rslt;

    if (chip_id == BMP5_CHIP_ID)
    {
        /* Updating chip_id in device structure */
        dev->chip_id = chip_id;
        rslt = BMP5_OK;
    }
    else
    {
        rslt = BMP5_E_DEV_NOT_FOUND;
    }

    return rslt;
}
//done
static int8_t get_nvm_status(uint8_t *nvm_status, struct bmp5_dev *dev)
{
    int8_t rslt;

    if (nvm_status != NULL)
    {
        rslt = bmp5_get_regs(BMP5_REG_STATUS, nvm_status, 1, dev);
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}
//done
int8_t BMP581::getSensorData(bmp5_sensor_data* data)
{
    return bmp5_get_sensor_data(data, &osrOdrConfig, &sensor);
}
//done
int8_t bmp5_get_sensor_data(struct bmp5_sensor_data *sensor_data,
                            const struct bmp5_osr_odr_press_config *osr_odr_press_cfg,
                            struct bmp5_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[6] = { 0 };
    int32_t raw_data_t;
    uint32_t raw_data_p;

    rslt = bmp5_get_regs(BMP5_REG_TEMP_DATA_XLSB, reg_data, 6, dev);

    if (rslt == BMP5_OK)
    {
        raw_data_t = (int32_t)(((uint32_t)reg_data[2] << 16) | ((uint16_t)reg_data[1] << 8) | reg_data[0]);

#ifdef BMP5_USE_FIXED_POINT

        /* Division by 2^16(whose equivalent value is 65536) is performed to get temperature data and followed by fixed point digit
         * precision in deg C
         */
        sensor_data->temperature =
            (int64_t)((raw_data_t / (int64_t)65536.0) * (power(10, BMP5_FIXED_POINT_DIGIT_PRECISION)));
#else

        /* Division by 2^16(whose equivalent value is 65536) is performed to get temperature data in deg C */
        sensor_data->temperature = (float)(raw_data_t / 65536.0);
#endif

        if (osr_odr_press_cfg->press_en == BMP5_ENABLE)
        {
            raw_data_p = (uint32_t)(((uint32_t)reg_data[5] << 16) | ((uint16_t)reg_data[4] << 8) | reg_data[3]);

#ifdef BMP5_USE_FIXED_POINT

            /* Division by 2^6(whose equivalent value is 64) is performed to get pressure data and followed by fixed point digit
             * precision in Pa
             */
            sensor_data->pressure =
                (uint64_t)((raw_data_p / (int64_t)64.0) * (power(10, BMP5_FIXED_POINT_DIGIT_PRECISION)));
#else

            /* Division by 2^6(whose equivalent value is 64) is performed to get pressure data in Pa */
            sensor_data->pressure = (float)(raw_data_p / 64.0);
#endif
        }
        else
        {
            sensor_data->pressure = 0.0;
        }
    }

    return rslt;
}
//done
static uint32_t power(uint8_t base, uint8_t resolution)
{
    /* Initialize loop */
    uint8_t loop = 1;

    /* Initialize variable to store the power of 2 value */
    uint32_t value = 1;

    for (; loop <= resolution; loop++)
    {
        value = (uint32_t)(value * base);
    }

    return value;
}
//done
static int8_t null_ptr_check(const struct bmp5_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMP5_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMP5_OK;
    }

    return rslt;
}

BMP5_INTF_RET_TYPE BMP581::readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr)
{
    // Make sure the number of bytes is valid
    if(numBytes == 0)
    {
        return BMP5_E_COM_FAIL;
    }

    // Get interface data
    BMP581_InterfaceData* interfaceData = (BMP581_InterfaceData*) interfacePtr;
    // Jump to desired register address
    interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);
    interfaceData->i2cPort->write(regAddress);
    if(interfaceData->i2cPort->endTransmission())
    {
      return BMP5_E_COM_FAIL;
    }
    // Read bytes from these registers
    interfaceData->i2cPort->requestFrom(interfaceData->i2cAddress, numBytes);
    // Store all requested bytes
    for(uint32_t i = 0; i < numBytes && interfaceData->i2cPort->available(); i++)
    {
      dataBuffer[i] = interfaceData->i2cPort->read();
    }
    return BMP5_OK;
}

BMP5_INTF_RET_TYPE BMP581::writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr)
{
    // Make sure the number of bytes is valid
    if(numBytes == 0)
    {
        return BMP5_E_COM_FAIL;
    }
    // Get interface data
    BMP581_InterfaceData* interfaceData = (BMP581_InterfaceData*) interfacePtr;
    // Begin transmission
    interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);

    // Write the address
    interfaceData->i2cPort->write(regAddress);
            
    // Write all the data
    for(uint32_t i = 0; i < numBytes; i++)
    {
      interfaceData->i2cPort->write(dataBuffer[i]);
    }
    // End transmission
    if(interfaceData->i2cPort->endTransmission())
    {
      return BMP5_E_COM_FAIL;
    }
    return BMP5_OK;
}

void BMP581::usDelay(uint32_t period, void* interfacePtr)
{
    delayMicroseconds(period);
}

int8_t BMP581::setMode(bmp5_powermode mode)
{
    return bmp5_set_power_mode(mode, &sensor);
}
