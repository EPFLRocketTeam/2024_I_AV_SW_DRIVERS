
#ifndef BMP_def_H
#define BMP_def_H

#include "I2c_interface.h"

//#include <stdint.h>
//#include <stddef.h>
//#include <math.h>
//#include <Wire.h>
/*!
 * BMP5_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BMP5_INTF_RET_TYPE
#define BMP5_INTF_RET_TYPE                        int8_t
#endif

/*!
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BMP5_INTF_RET_SUCCESS
#define BMP5_INTF_RET_SUCCESS                     INT8_C(0)
#endif

/*! @name BOOLEAN TYPES */
#ifndef TRUE
#define TRUE                                      UINT8_C(0x01)
#endif

#ifndef FALSE
#define FALSE                                     UINT8_C(0x00)
#endif

#ifndef NULL
#define NULL                                      UINT8_C(0x00)
#endif

#ifndef ABS
#define ABS(a)                                    ((a) > 0 ? (a) : -(a))    /*!< Absolute value */
#endif


/*! @name BIT SLICE GET AND SET FUNCTIONS */
#define BMP5_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)

#define BMP5_SET_BITSLICE(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | \
     ((val << bitname##_POS) & bitname##_MSK))

#define BMP5_GET_LSB(var)                         (uint8_t)(var & BMP5_SET_LOW_BYTE)
#define BMP5_GET_MSB(var)                         (uint8_t)((var & BMP5_SET_HIGH_BYTE) >> 8)

#define BMP5_SET_BIT_VAL_0(reg_data, bitname)     (reg_data & ~(bitname##_MSK))

#define BMP5_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMP5_GET_BITS_POS_0(reg_data, bitname)    (reg_data & (bitname##_MSK))

/*! @name Chip id of BMP5 */
#define BMP5_CHIP_ID                              UINT8_C(0x50)

/*! @name API success code */
#define BMP5_OK                                   INT8_C(0)

/*! @name API error codes */
#define BMP5_E_NULL_PTR                           INT8_C(-1)
#define BMP5_E_COM_FAIL                           INT8_C(-2)
#define BMP5_E_DEV_NOT_FOUND                      INT8_C(-3)
#define BMP5_E_INVALID_CHIP_ID                    INT8_C(-4)
#define BMP5_E_POWER_UP                           INT8_C(-5)
#define BMP5_E_POR_SOFTRESET                      INT8_C(-6)
#define BMP5_E_INVALID_POWERMODE                  INT8_C(-7)
#define BMP5_E_INVALID_THRESHOLD                  INT8_C(-8)
#define BMP5_E_FIFO_FRAME_EMPTY                   INT8_C(-9)
#define BMP5_E_NVM_INVALID_ADDR                   INT8_C(-10)
#define BMP5_E_NVM_NOT_READY                      INT8_C(-11)

#define BMP5_ENABLE                               UINT8_C(0x01)
#define BMP5_DISABLE                              UINT8_C(0x00)

/*! @name Register addresses */
#define BMP5_REG_CHIP_ID                          UINT8_C(0x01)
#define BMP5_REG_REV_ID                           UINT8_C(0x02)
#define BMP5_REG_CHIP_STATUS                      UINT8_C(0x11)
#define BMP5_REG_DRIVE_CONFIG                     UINT8_C(0x13)
#define BMP5_REG_INT_CONFIG                       UINT8_C(0x14)
#define BMP5_REG_INT_SOURCE                       UINT8_C(0x15)
#define BMP5_REG_FIFO_CONFIG                      UINT8_C(0x16)
#define BMP5_REG_FIFO_COUNT                       UINT8_C(0x17)
#define BMP5_REG_FIFO_SEL                         UINT8_C(0x18)
#define BMP5_REG_TEMP_DATA_XLSB                   UINT8_C(0x1D)
#define BMP5_REG_TEMP_DATA_LSB                    UINT8_C(0x1E)
#define BMP5_REG_TEMP_DATA_MSB                    UINT8_C(0x1F)
#define BMP5_REG_PRESS_DATA_XLSB                  UINT8_C(0x20)
#define BMP5_REG_PRESS_DATA_LSB                   UINT8_C(0x21)
#define BMP5_REG_PRESS_DATA_MSB                   UINT8_C(0x22)
#define BMP5_REG_INT_STATUS                       UINT8_C(0x27)
#define BMP5_REG_STATUS                           UINT8_C(0x28)
#define BMP5_REG_FIFO_DATA                        UINT8_C(0x29)
#define BMP5_REG_NVM_ADDR                         UINT8_C(0x2B)
#define BMP5_REG_NVM_DATA_LSB                     UINT8_C(0x2C)
#define BMP5_REG_NVM_DATA_MSB                     UINT8_C(0x2D)
#define BMP5_REG_DSP_CONFIG                       UINT8_C(0x30)
#define BMP5_REG_DSP_IIR                          UINT8_C(0x31)
#define BMP5_REG_OOR_THR_P_LSB                    UINT8_C(0x32)
#define BMP5_REG_OOR_THR_P_MSB                    UINT8_C(0x33)
#define BMP5_REG_OOR_RANGE                        UINT8_C(0x34)
#define BMP5_REG_OOR_CONFIG                       UINT8_C(0x35)
#define BMP5_REG_OSR_CONFIG                       UINT8_C(0x36)
#define BMP5_REG_ODR_CONFIG                       UINT8_C(0x37)
#define BMP5_REG_OSR_EFF                          UINT8_C(0x38)
#define BMP5_REG_CMD                              UINT8_C(0x7E)

/*! @name I2C addresses */
#define BMP5_I2C_ADDR_PRIM                        UINT8_C(0x46)
#define BMP5_I2C_ADDR_SEC                         UINT8_C(0x47)

/*! @name NVM addresses */
#define BMP5_NVM_START_ADDR                       UINT8_C(0x20)
#define BMP5_NVM_END_ADDR                         UINT8_C(0x22)

/*! @name Interface settings */
#define BMP5_SPI_RD_MASK                          UINT8_C(0x80)

/*! @name Delay definition */
#define BMP5_DELAY_US_SOFT_RESET                  UINT16_C(2000)
#define BMP5_DELAY_US_STANDBY                     UINT16_C(2500)
#define BMP5_DELAY_US_NVM_READY_READ              UINT8_C(800)
#define BMP5_DELAY_US_NVM_READY_WRITE             UINT16_C(10000)

/*! @name Soft reset command */
#define BMP5_SOFT_RESET_CMD                       UINT8_C(0xB6)

/*! NVM command */
#define BMP5_NVM_FIRST_CMND                       UINT8_C(0x5D)
#define BMP5_NVM_READ_ENABLE_CMND                 UINT8_C(0xA5)
#define BMP5_NVM_WRITE_ENABLE_CMND                UINT8_C(0xA0)

/*! @name Deepstandby enable/disable */
#define BMP5_DEEP_ENABLED                         UINT8_C(0)
#define BMP5_DEEP_DISABLED                        UINT8_C(1)

/*! @name ODR settings */
#define BMP5_ODR_240_HZ                           UINT8_C(0x00)
#define BMP5_ODR_218_5_HZ                         UINT8_C(0x01)
#define BMP5_ODR_199_1_HZ                         UINT8_C(0x02)
#define BMP5_ODR_179_2_HZ                         UINT8_C(0x03)
#define BMP5_ODR_160_HZ                           UINT8_C(0x04)
#define BMP5_ODR_149_3_HZ                         UINT8_C(0x05)
#define BMP5_ODR_140_HZ                           UINT8_C(0x06)
#define BMP5_ODR_129_8_HZ                         UINT8_C(0x07)
#define BMP5_ODR_120_HZ                           UINT8_C(0x08)
#define BMP5_ODR_110_1_HZ                         UINT8_C(0x09)
#define BMP5_ODR_100_2_HZ                         UINT8_C(0x0A)
#define BMP5_ODR_89_6_HZ                          UINT8_C(0x0B)
#define BMP5_ODR_80_HZ                            UINT8_C(0x0C)
#define BMP5_ODR_70_HZ                            UINT8_C(0x0D)
#define BMP5_ODR_60_HZ                            UINT8_C(0x0E)
#define BMP5_ODR_50_HZ                            UINT8_C(0x0F)
#define BMP5_ODR_45_HZ                            UINT8_C(0x10)
#define BMP5_ODR_40_HZ                            UINT8_C(0x11)
#define BMP5_ODR_35_HZ                            UINT8_C(0x12)
#define BMP5_ODR_30_HZ                            UINT8_C(0x13)
#define BMP5_ODR_25_HZ                            UINT8_C(0x14)
#define BMP5_ODR_20_HZ                            UINT8_C(0x15)
#define BMP5_ODR_15_HZ                            UINT8_C(0x16)
#define BMP5_ODR_10_HZ                            UINT8_C(0x17)
#define BMP5_ODR_05_HZ                            UINT8_C(0x18)
#define BMP5_ODR_04_HZ                            UINT8_C(0x19)
#define BMP5_ODR_03_HZ                            UINT8_C(0x1A)
#define BMP5_ODR_02_HZ                            UINT8_C(0x1B)
#define BMP5_ODR_01_HZ                            UINT8_C(0x1C)
#define BMP5_ODR_0_5_HZ                           UINT8_C(0x1D)
#define BMP5_ODR_0_250_HZ                         UINT8_C(0x1E)
#define BMP5_ODR_0_125_HZ                         UINT8_C(0x1F)

/*! @name Oversampling for temperature and pressure */
#define BMP5_OVERSAMPLING_1X                      UINT8_C(0x00)
#define BMP5_OVERSAMPLING_2X                      UINT8_C(0x01)
#define BMP5_OVERSAMPLING_4X                      UINT8_C(0x02)
#define BMP5_OVERSAMPLING_8X                      UINT8_C(0x03)
#define BMP5_OVERSAMPLING_16X                     UINT8_C(0x04)
#define BMP5_OVERSAMPLING_32X                     UINT8_C(0x05)
#define BMP5_OVERSAMPLING_64X                     UINT8_C(0x06)
#define BMP5_OVERSAMPLING_128X                    UINT8_C(0x07)

/*! @name IIR filter for temperature and pressure */
#define BMP5_IIR_FILTER_BYPASS                    UINT8_C(0x00)
#define BMP5_IIR_FILTER_COEFF_1                   UINT8_C(0x01)
#define BMP5_IIR_FILTER_COEFF_3                   UINT8_C(0x02)
#define BMP5_IIR_FILTER_COEFF_7                   UINT8_C(0x03)
#define BMP5_IIR_FILTER_COEFF_15                  UINT8_C(0x04)
#define BMP5_IIR_FILTER_COEFF_31                  UINT8_C(0x05)
#define BMP5_IIR_FILTER_COEFF_63                  UINT8_C(0x06)
#define BMP5_IIR_FILTER_COEFF_127                 UINT8_C(0x07)

/*! Fifo frame configuration */
#define BMP5_FIFO_EMPTY                           UINT8_C(0X7F)
#define BMP5_FIFO_MAX_THRESHOLD_P_T_MODE          UINT8_C(0x0F)
#define BMP5_FIFO_MAX_THRESHOLD_P_MODE            UINT8_C(0x1F)

/*! @name Macro is used to bypass both iir_t and iir_p together */
#define BMP5_IIR_BYPASS                           UINT8_C(0xC0)

/*! @name Pressure Out-of-range count limit */
#define BMP5_OOR_COUNT_LIMIT_1                    UINT8_C(0x00)
#define BMP5_OOR_COUNT_LIMIT_3                    UINT8_C(0x01)
#define BMP5_OOR_COUNT_LIMIT_7                    UINT8_C(0x02)
#define BMP5_OOR_COUNT_LIMIT_15                   UINT8_C(0x03)

/*! @name Interrupt configurations */
#define BMP5_INT_MODE_PULSED                      UINT8_C(0)
#define BMP5_INT_MODE_LATCHED                     UINT8_C(1)

#define BMP5_INT_POL_ACTIVE_LOW                   UINT8_C(0)
#define BMP5_INT_POL_ACTIVE_HIGH                  UINT8_C(1)

#define BMP5_INT_OD_PUSHPULL                      UINT8_C(0)
#define BMP5_INT_OD_OPENDRAIN                     UINT8_C(1)

/*! @name NVM and Interrupt status asserted macros */
#define BMP5_INT_ASSERTED_DRDY                    UINT8_C(0x01)
#define BMP5_INT_ASSERTED_FIFO_FULL               UINT8_C(0x02)
#define BMP5_INT_ASSERTED_FIFO_THRES              UINT8_C(0x04)
#define BMP5_INT_ASSERTED_PRESSURE_OOR            UINT8_C(0x08)
#define BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE  UINT8_C(0x10)
#define BMP5_INT_NVM_RDY                          UINT8_C(0x02)
#define BMP5_INT_NVM_ERR                          UINT8_C(0x04)
#define BMP5_INT_NVM_CMD_ERR                      UINT8_C(0x08)

/*! @name Interrupt configurations */
#define BMP5_INT_MODE_MSK                         UINT8_C(0x01)

#define BMP5_INT_POL_MSK                          UINT8_C(0x02)
#define BMP5_INT_POL_POS                          UINT8_C(1)

#define BMP5_INT_OD_MSK                           UINT8_C(0x04)
#define BMP5_INT_OD_POS                           UINT8_C(2)

#define BMP5_INT_EN_MSK                           UINT8_C(0x08)
#define BMP5_INT_EN_POS                           UINT8_C(3)

#define BMP5_INT_DRDY_EN_MSK                      UINT8_C(0x01)

#define BMP5_INT_FIFO_FULL_EN_MSK                 UINT8_C(0x02)
#define BMP5_INT_FIFO_FULL_EN_POS                 UINT8_C(1)

#define BMP5_INT_FIFO_THRES_EN_MSK                UINT8_C(0x04)
#define BMP5_INT_FIFO_THRES_EN_POS                UINT8_C(2)

#define BMP5_INT_OOR_PRESS_EN_MSK                 UINT8_C(0x08)
#define BMP5_INT_OOR_PRESS_EN_POS                 UINT8_C(3)

/*! @name ODR configuration */
#define BMP5_ODR_MSK                              UINT8_C(0x7C)
#define BMP5_ODR_POS                              UINT8_C(2)

/*! @name OSR configurations */
#define BMP5_TEMP_OS_MSK                          UINT8_C(0x07)

#define BMP5_PRESS_OS_MSK                         UINT8_C(0x38)
#define BMP5_PRESS_OS_POS                         UINT8_C(3)

/*! @name Pressure enable */
#define BMP5_PRESS_EN_MSK                         UINT8_C(0x40)
#define BMP5_PRESS_EN_POS                         UINT8_C(6)

/*! @name IIR configurations */
#define BMP5_SET_IIR_TEMP_MSK                     UINT8_C(0x07)

#define BMP5_SET_IIR_PRESS_MSK                    UINT8_C(0x38)
#define BMP5_SET_IIR_PRESS_POS                    UINT8_C(3)

#define BMP5_OOR_SEL_IIR_PRESS_MSK                UINT8_C(0x80)
#define BMP5_OOR_SEL_IIR_PRESS_POS                UINT8_C(7)

#define BMP5_SHDW_SET_IIR_TEMP_MSK                UINT8_C(0x08)
#define BMP5_SHDW_SET_IIR_TEMP_POS                UINT8_C(3)

#define BMP5_SHDW_SET_IIR_PRESS_MSK               UINT8_C(0x20)
#define BMP5_SHDW_SET_IIR_PRESS_POS               UINT8_C(5)

#define BMP5_SET_FIFO_IIR_TEMP_MSK                UINT8_C(0x10)
#define BMP5_SET_FIFO_IIR_TEMP_POS                UINT8_C(4)

#define BMP5_SET_FIFO_IIR_PRESS_MSK               UINT8_C(0x40)
#define BMP5_SET_FIFO_IIR_PRESS_POS               UINT8_C(6)

#define BMP5_IIR_FLUSH_FORCED_EN_MSK              UINT8_C(0x04)
#define BMP5_IIR_FLUSH_FORCED_EN_POS              UINT8_C(2)

/*! @name Effective OSR configurations and ODR valid status */
#define BMP5_OSR_TEMP_EFF_MSK                     UINT8_C(0x07)

#define BMP5_OSR_PRESS_EFF_MSK                    UINT8_C(0x38)
#define BMP5_OSR_PRESS_EFF_POS                    UINT8_C(3)

#define BMP5_ODR_IS_VALID_MSK                     UINT8_C(0x80)
#define BMP5_ODR_IS_VALID_POS                     UINT8_C(7)

/*! @name Powermode */
#define BMP5_POWERMODE_MSK                        UINT8_C(0x03)

#define BMP5_DEEP_DISABLE_MSK                     UINT8_C(0x80)
#define BMP5_DEEP_DISABLE_POS                     UINT8_C(7)

/*! @name Fifo configurations */
#define BMP5_FIFO_THRESHOLD_MSK                   UINT8_C(0x1F)

#define BMP5_FIFO_MODE_MSK                        UINT8_C(0x20)
#define BMP5_FIFO_MODE_POS                        UINT8_C(5)

#define BMP5_FIFO_DEC_SEL_MSK                     UINT8_C(0x1C)
#define BMP5_FIFO_DEC_SEL_POS                     UINT8_C(2)

#define BMP5_FIFO_COUNT_MSK                       UINT8_C(0x3F)

#define BMP5_FIFO_FRAME_SEL_MSK                   UINT8_C(0x03)

/*! @name Out-of-range configuration */
#define BMP5_OOR_THR_P_LSB_MSK                    UINT32_C(0x0000FF)

#define BMP5_OOR_THR_P_MSB_MSK                    UINT32_C(0x00FF00)

#define BMP5_OOR_THR_P_XMSB_MSK                   UINT32_C(0x010000)
#define BMP5_OOR_THR_P_XMSB_POS                   UINT16_C(16)

/* Macro to mask xmsb value of oor threshold from register(0x35) value */
#define BMP5_OOR_THR_P_XMSB_REG_MSK               UINT8_C(0x01)

#define BMP5_OOR_COUNT_LIMIT_MSK                  UINT8_C(0xC0)
#define BMP5_OOR_COUNT_LIMIT_POS                  UINT8_C(6)

/*! @name NVM configuration */
#define BMP5_NVM_ADDR_MSK                         UINT8_C(0x3F)

#define BMP5_NVM_PROG_EN_MSK                      UINT8_C(0x40)
#define BMP5_NVM_PROG_EN_POS                      UINT8_C(6)

#define BMP5_NVM_DATA_LSB_MSK                     UINT16_C(0x00FF)

#define BMP5_NVM_DATA_MSB_MSK                     UINT16_C(0xFF00)
#define BMP5_E_INVALID_SETTING (BMP5_E_NVM_NOT_READY - 1)

// SparkFun's default I2C address is opposite of Bosch's default
#define BMP581_I2C_ADDRESS_DEFAULT BMP5_I2C_ADDR_SEC    // 0x47
#define BMP581_I2C_ADDRESS_SECONDARY BMP5_I2C_ADDR_PRIM // 0x46


#endif