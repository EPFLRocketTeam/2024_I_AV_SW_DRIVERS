#ifndef BMP_H
#define BMP_H


#include "BMP_def.hpp"



typedef BMP5_INTF_RET_TYPE (*bmp5_read_fptr_t)(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);
typedef BMP5_INTF_RET_TYPE (*bmp5_write_fptr_t)(uint8_t reg_addr, const uint8_t *read_data, uint32_t len,
                                                void *intf_ptr);
typedef void (*bmp5_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

enum bmp5_intf {
    /*! SPI interface */
    BMP5_SPI_INTF,
    /*! I2C interface */
    BMP5_I2C_INTF,
    /*! I3C interface */
    BMP5_I3C_INTF
};

struct BMP581_InterfaceData
{
    // Communication interface (I2C or SPI)
    bmp5_intf interface;

    // I2C settings
    uint8_t i2cAddress;
    ////TwoWire* i2cPort;
   // WIRE_TEENSY* i2cPort;
    I2C_INTERFACE* i2cPort;

    /*// SPI settings
    uint8_t spiCSPin;
    uint32_t spiClockFrequency;*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};
struct bmp5_dev
{
    /*! Chip ID */
    uint8_t chip_id;

    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *intf_ptr;

    /*! Read function pointer */
    bmp5_read_fptr_t read;

    /*! Write function pointer */
    bmp5_write_fptr_t write;

    /*! Delay function pointer */
    bmp5_delay_us_fptr_t delay_us;

    /*! To store interface pointer error */
    BMP5_INTF_RET_TYPE intf_rslt;

    /*! Type of Interface  */
    enum bmp5_intf intf;
};
enum bmp5_powermode {
    /*! Standby powermode */
    BMP5_POWERMODE_STANDBY,
    /*! Normal powermode */
    BMP5_POWERMODE_NORMAL,
    /*! Forced powermode */
    BMP5_POWERMODE_FORCED,
    /*! Continous powermode */
    BMP5_POWERMODE_CONTINOUS,
    /*! Deep standby powermode */
    BMP5_POWERMODE_DEEP_STANDBY
};
struct bmp5_osr_odr_press_config
{
    /*! Temperature oversampling
     * Assignable macros :
     * - BMP5_OVERSAMPLING_1X
     * - BMP5_OVERSAMPLING_2X
     * - BMP5_OVERSAMPLING_4X
     * - BMP5_OVERSAMPLING_8X
     * - BMP5_OVERSAMPLING_16X
     * - BMP5_OVERSAMPLING_32X
     * - BMP5_OVERSAMPLING_64X
     * - BMP5_OVERSAMPLING_128X
     */
    uint8_t osr_t;

    /*! Pressure oversampling
     * Assignable macros :
     * - BMP5_OVERSAMPLING_1X
     * - BMP5_OVERSAMPLING_2X
     * - BMP5_OVERSAMPLING_4X
     * - BMP5_OVERSAMPLING_8X
     * - BMP5_OVERSAMPLING_16X
     * - BMP5_OVERSAMPLING_32X
     * - BMP5_OVERSAMPLING_64X
     * - BMP5_OVERSAMPLING_128X
     */
    uint8_t osr_p;

    /*! Enable pressure
     * BMP5_ENABLE  = Enables pressure data
     * BMP5_DISABLE = Disables pressure data
     */
    uint8_t press_en;

    /*! Output Data Rate */
    uint8_t odr;
};
struct bmp5_fifo
{
    /*! Pointer to fifo data */
    uint8_t *data;

    /*! Length of user defined bytes of fifo to be read */
    uint16_t length;

    /*! Fifo frame data source selection
     * Assignable macros :
     * - BMP5_FIFO_NOT_ENABLED
     * - BMP5_FIFO_TEMPERATURE_DATA
     * - BMP5_FIFO_PRESSURE_DATA
     * - BMP5_FIFO_PRESS_TEMP_DATA
     */
    uint8_t frame_sel;

    /*! Fifo decimation factor(downsampling) selection
     * Assignable macros :
     * - BMP5_FIFO_NO_DOWNSAMPLING
     * - BMP5_FIFO_DOWNSAMPLING_2X
     * - BMP5_FIFO_DOWNSAMPLING_4X
     * - BMP5_FIFO_DOWNSAMPLING_8X
     * - BMP5_FIFO_DOWNSAMPLING_16X
     * - BMP5_FIFO_DOWNSAMPLING_32X
     * - BMP5_FIFO_DOWNSAMPLING_64X
     * - BMP5_FIFO_DOWNSAMPLING_128X
     */
    uint8_t dec_sel;

    /*! Fifo frame count */
    uint8_t fifo_count;

    /*! Fifo mode selection
     * Assignable macros :
     * - BMP5_FIFO_MODE_STREAMING
     * - BMP5_FIFO_MODE_STOP_ON_FULL
     */
    uint8_t mode;

    /*! Threshold for fifo */
    uint8_t threshold;

    /*! Fifo temperature IIR
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t set_fifo_iir_t;

    /*! Fifo pressure IIR
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t set_fifo_iir_p;
};
struct bmp5_sensor_data
{
    /*! Pressure data */
    float pressure;

    /*! Temperature data */
    float temperature;
};
struct bmp5_iir_config
{
    /*! Temperature IIR
     * Assignable macros :
     * - BMP5_IIR_FILTER_BYPASS
     * - BMP5_IIR_FILTER_0_20980
     * - BMP5_IIR_FILTER_0_08045
     * - BMP5_IIR_FILTER_0_03695
     * - BMP5_IIR_FILTER_0_01785
     * - BMP5_IIR_FILTER_0_00880
     * - BMP5_IIR_FILTER_0_00435
     * - BMP5_IIR_FILTER_0_00220
     */
    uint8_t set_iir_t;

    /*! Pressure IIR
     * Assignable macros :
     * - BMP5_IIR_FILTER_BYPASS
     * - BMP5_IIR_FILTER_0_20980
     * - BMP5_IIR_FILTER_0_08045
     * - BMP5_IIR_FILTER_0_03695
     * - BMP5_IIR_FILTER_0_01785
     * - BMP5_IIR_FILTER_0_00880
     * - BMP5_IIR_FILTER_0_00435
     * - BMP5_IIR_FILTER_0_00220
     */
    uint8_t set_iir_p;

    /*! Shadow IIR selection for temperature
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t shdw_set_iir_t;

    /*! Shadow IIR selection for pressure
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t shdw_set_iir_p;

    /*! IIR flush in forced mode enable
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t iir_flush_forced_en;
};


int8_t bmp5_set_power_mode(enum bmp5_powermode powermode, struct bmp5_dev *dev);
int8_t bmp5_set_osr_odr_press_config(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);
int8_t bmp5_get_interrupt_status(uint8_t *int_status, struct bmp5_dev *dev);
int8_t bmp5_set_regs(uint8_t reg_addr, const uint8_t *data, uint32_t len, struct bmp5_dev *dev);
int8_t bmp5_soft_reset(struct bmp5_dev *dev);
int8_t bmp5_init(struct bmp5_dev *dev);
int8_t bmp5_get_regs(uint8_t reg_addr, uint8_t *data, uint32_t len, struct bmp5_dev *dev);
int8_t bmp5_get_power_mode(enum bmp5_powermode *powermode, struct bmp5_dev *dev);
int8_t bmp5_get_iir_config(struct bmp5_iir_config *iir_cfg, struct bmp5_dev *dev);
int8_t bmp5_get_sensor_data(struct bmp5_sensor_data *sensor_data,
                            const struct bmp5_osr_odr_press_config *osr_odr_press_cfg,
                            struct bmp5_dev *dev);
int8_t bmp5_get_osr_odr_press_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);


class BMP581
{
    public:
        // Constructor
        BMP581();

        // Sensor initialization, must specify communication interface
        int8_t beginI2C(uint8_t address = BMP5_I2C_ADDR_SEC, I2C_INTERFACE* wirePort= nullptr );
        // Configuration control, the begin functions will set defaults for these
        int8_t init();
        int8_t setMode(bmp5_powermode mode);
        int8_t getMode(bmp5_powermode* mode);
        int8_t enablePress(uint8_t pressEnable);

        // Data acquisistion
        int8_t getSensorData(bmp5_sensor_data* data);

    private:
        // Sensor initialization, after communication interface has been selected
        int8_t begin();

        // Read/write helper functions
        static BMP5_INTF_RET_TYPE readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr);
        static BMP5_INTF_RET_TYPE writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr);

        // Deley helper function
        static void usDelay(uint32_t period, void* interfacePtr);

        // Reference to the sensor
        struct bmp5_dev sensor;

        // Information about the selected communication interface (I2C or SPI)
        BMP581_InterfaceData interfaceData;

        // Place to store OSR/ODR config values
        bmp5_osr_odr_press_config osrOdrConfig;

        // Place to store FIFO config values
        bmp5_fifo fifo;
};
//read the data
void read_bmp(BMP581 &sens);

#endif