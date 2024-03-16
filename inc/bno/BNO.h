
#ifndef BNO_H
#define BNO_H


#include "I2c_interface.h"
//#include <Wire.h>
#define BNO055_ADDRESS 0x28
/** BNO055 ID **/
#define BNO055_ID (0xA0)

const float gyroScale = 16.0; 
const float accelScale = 100.0;
const float magScale = 16.0;
struct BNO_data
{
    int sysCalib,gyroCalib,accelCalib,magCalib ;
    double accelX,laccelX,magX,gyroX ;
    double accelY,magY,gyroY,laccelY ;
    double accelZ,magZ,gyroZ,laccelZ;
};

void read_calibration();
void print_bno_data();
void read_data_all(BNO_data* data);
void read_calibration(BNO_data* data);
void read_linear(BNO_data* data);
void read_gyro(BNO_data* data);
void read_mad(BNO_data* data);
void read_accel(BNO_data* data);

class Adafruit_BNO055 {
public:
typedef enum {
  /* Page id register definition */
    BNO055_PAGE_ID_ADDR = 0X07,
    /* PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_ACCEL_REV_ID_ADDR = 0x01,
    BNO055_MAG_REV_ID_ADDR = 0x02,
    BNO055_GYRO_REV_ID_ADDR = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR = 0x05,
    BNO055_BL_REV_ID_ADDR = 0X06,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR = 0X3D,
    BNO055_PWR_MODE_ADDR = 0X3E,

    BNO055_SYS_TRIGGER_ADDR = 0X3F,
    BNO055_TEMP_SOURCE_ADDR = 0X40,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR = 0X67,
    ACCEL_RADIUS_MSB_ADDR = 0X68,
    MAG_RADIUS_LSB_ADDR = 0X69,
    MAG_RADIUS_MSB_ADDR = 0X6A
  } adafruit_bno055_reg_t;

  /** BNO055 power settings */
  typedef enum {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
  } adafruit_bno055_powermode_t;
  /** Operation mode settings **/
  typedef enum {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
} adafruit_bno055_opmode_t;

  Adafruit_BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS,
                  WIRE_TEENSY *theWire = &Wire_r);

  bool begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
  void setMode(adafruit_bno055_opmode_t mode);


  bool begin_(bool addr_detect = true);
  bool detected(void);
  size_t maxBufferSize() { return _maxBufferSize; }
  bool read(uint8_t *buffer, size_t len, bool stop = true);
  bool write(const uint8_t *buffer, size_t len, bool stop = true,
             const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
  bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                       uint8_t *read_buffer, size_t read_len,
                       bool stop = false);
  bool _read(uint8_t *buffer, size_t len, bool stop);

private:
  byte read8(adafruit_bno055_reg_t);
  bool readLen(adafruit_bno055_reg_t, byte *buffer, uint8_t len);
  bool write8(adafruit_bno055_reg_t, byte value);
  int32_t _sensorID;
  adafruit_bno055_opmode_t _mode;

  uint8_t _addr;
  WIRE_TEENSY *_wire;
  bool _begun;
  size_t _maxBufferSize =32;
};
#endif