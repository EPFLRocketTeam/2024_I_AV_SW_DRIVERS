#include "BNO.h"




bool Adafruit_BNO055::begin_(bool addr_detect) {


  _wire->begin();
  _begun = true;

  if (addr_detect) {
    return detected();
  }
  return true;
}



bool Adafruit_BNO055::write_then_read(const uint8_t *write_buffer,
                                         size_t write_len, uint8_t *read_buffer,
                                         size_t read_len, bool stop) {
  if (!write(write_buffer, write_len, stop)) {

    
    return false;
  }


  return read(read_buffer, read_len);
}

bool Adafruit_BNO055::read(uint8_t *buffer, size_t len, bool stop) {
  size_t pos = 0;
  while (pos < len) {
    size_t read_len =
        ((len - pos) > maxBufferSize()) ? maxBufferSize() : (len - pos);
    bool read_stop = (pos < (len - read_len)) ? false : stop;
    if (!_read(buffer + pos, read_len, read_stop))
      return false;
    pos += read_len;
  }
  return true;
}
bool Adafruit_BNO055::_read(uint8_t *buffer, size_t len, bool stop) {
  size_t recv = _wire->requestFrom((uint8_t)_addr, (uint8_t)len, (uint8_t)stop);
  if (recv != len) {
    // Not enough data available to fulfill our obligation!

    return false;
  }
  for (uint16_t i = 0; i < len; i++) {
    buffer[i] = _wire->read();
  }
  return true;
}


bool Adafruit_BNO055::write(const uint8_t *buffer, size_t len, bool stop,
                               const uint8_t *prefix_buffer,
                               size_t prefix_len) {


  if ((len + prefix_len) > maxBufferSize()) {
    // currently not guaranteed to work if more than 32 bytes!
    // we will need to find out if some platforms have larger
    // I2C buffer sizes :/
    return false;
  }

  _wire->beginTransmission(_addr);


  // Write the prefix data (usually an address)
  if ((prefix_len != 0) && (prefix_buffer != nullptr)) {
    if (_wire->write(prefix_buffer, prefix_len) != prefix_len) {

      return false;
    }
  }


  // Write the data itself
  if (_wire->write(buffer, len) != len) {
    return false;
  }


  if (_wire->endTransmission(stop) == 0) {

    return true;
  } else {

    return false;
  }
}


bool Adafruit_BNO055::detected(void) {
  // Init I2C if not done yet
  if (!_begun && !begin()) {
    return false;
  }
  // A basic scanner, see if it ACK's
  _wire->beginTransmission(_addr);
  if (_wire->endTransmission() == 0) {
    return true;
  }
  return false;
}


bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode) {
  // Start without a detection
  begin_(false);

  // can take 850 ms to boot!
  int timeout = 850; // in ms
  while (timeout > 0) {
    if (begin_()) {
      break;
    }
    // wasnt detected... we'll retry!
    delay(10);
    timeout -= 10;
  }
  if (timeout <= 0){
    return false;
  }

  /* Make sure we have the right device */
  uint8_t id = read8(BNO055_CHIP_ID_ADDR);

  if (id != BNO055_ID) {
    delay(1000); // hold on for boot
    id = read8(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
      return false; // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);

  /* Reset */
  write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  delay(30);
  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);

  write8(BNO055_PAGE_ID_ADDR, 0);

  write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(mode);
  delay(20);

  return true;
}

void Adafruit_BNO055::setMode(adafruit_bno055_opmode_t mode) {
  _mode = mode;
  write8(BNO055_OPR_MODE_ADDR, _mode);
  delay(30);
}

byte Adafruit_BNO055::read8(adafruit_bno055_reg_t reg) {
  uint8_t buffer[1] = {reg};
  write_then_read(buffer, 1, buffer, 1);
  return (byte)buffer[0];
}

bool Adafruit_BNO055::readLen(adafruit_bno055_reg_t reg, byte *buffer,
                              uint8_t len) {
  uint8_t reg_buf[1] = {(uint8_t)reg};
  return write_then_read(reg_buf, 1, buffer, len);
}

bool Adafruit_BNO055::write8(adafruit_bno055_reg_t reg, byte value) {
  uint8_t buffer[2] = {(uint8_t)reg, (uint8_t)value};
  return write(buffer, 2);
}


Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address,
                                 WIRE_TEENSY *theWire) {

  _sensorID = sensorID;
   _begun = false;
   _addr =BNO055_ADDRESS;
   _wire=theWire;
}





//call for a read of the data
void print_bno_data(){
   // Request 18 bytes of data starting from address 0x08
  uint8_t dataBuffer[18]; // Buffer to store the received data
  Wire_r.beginTransmission(BNO055_ADDRESS); // Replace with your sensor's address
  Wire_r.write(0x08);
  Wire_r.endTransmission(false); // Send a restart condition
  Wire_r.requestFrom(BNO055_ADDRESS, 18);  
  if(Wire_r.available() == 18) 
  {
    // Read the data into the dataBuffer
    for (int i = 0; i < 18; i++) 
    {
      dataBuffer[i] = Wire_r.read();
    }

    // Divide the data into variables 
    int16_t accelX = (dataBuffer[1] << 8) | dataBuffer[0];
    int16_t accelY = (dataBuffer[3] << 8) | dataBuffer[2];
    int16_t accelZ = (dataBuffer[5] << 8) | dataBuffer[4];
    int16_t magX = (dataBuffer[7] << 8) | dataBuffer[6];
    int16_t magY = (dataBuffer[9] << 8) | dataBuffer[8];
    int16_t magZ = (dataBuffer[11] << 8) | dataBuffer[10];
    int16_t gyroX = (dataBuffer[13] << 8) | dataBuffer[12];
    int16_t gyroY = (dataBuffer[15] << 8) | dataBuffer[14];
    int16_t gyroZ = (dataBuffer[17] << 8) | dataBuffer[16];

    // Convert raw accelerometer values 
    double accelX_g = accelX / accelScale;
    double accelY_g = accelY / accelScale;
    double accelZ_g = accelZ / accelScale;

    // Print the converted values
    Serial.print("Accel X (g): ");
    Serial.print(accelX_g);
    Serial.print("     Y (g): ");
    Serial.print(accelY_g);
    Serial.print("     Z (g): ");
    Serial.println(accelZ_g);

    // Convert raw magnetometer values to microteslas
    double magX_uT = magX / magScale;
    double magY_uT = magY / magScale;
    double magZ_uT = magZ / magScale;

    // Print the converted values
    Serial.print("Mag X (uT): ");
    Serial.print(magX_uT);
    Serial.print("   Y (uT): ");
    Serial.print(magY_uT);
    Serial.print("   Z (uT): ");
    Serial.println(magZ_uT);

    // Convert raw gyroscope values 
    double gyroX_dps = gyroX / gyroScale;
    double gyroY_dps = gyroY / gyroScale;
    double gyroZ_dps = gyroZ / gyroScale;

    // Print the converted values
    Serial.print("Gyro X (°/s): ");
    Serial.print(gyroX_dps);
    Serial.print("    Y (°/s): ");
    Serial.print(gyroY_dps);
    Serial.print("    Z (°/s): ");
    Serial.println(gyroZ_dps);
  }
  // Read linear accel
  Wire_r.beginTransmission(BNO055_ADDRESS);
  Wire_r.write(0x28);//linearaddress
  Wire_r.endTransmission(false);
  Wire_r.requestFrom(BNO055_ADDRESS,6); 
  if (Wire_r.available() == 6) 
  {
    for (int i = 0; i < 6; i++) 
    {
      dataBuffer[i] = Wire_r.read();
    }
        // Divide the data into variables 
    int16_t laccelX = (dataBuffer[1] << 8) | dataBuffer[0];
    int16_t laccelY = (dataBuffer[3] << 8) | dataBuffer[2];
    int16_t laccelZ = (dataBuffer[5] << 8) | dataBuffer[4];

    // Convert raw linear accel values 
    double laccelX_c = laccelX / accelScale;
    double laccelY_c = laccelY / accelScale;
    double laccelZ_c = laccelZ / accelScale;

    Serial.print("linear Accel X (g): ");
    Serial.print(laccelX_c );
    Serial.print("   Y (g): ");
    Serial.print(laccelY_c );
    Serial.print("   Z (g): ");
    Serial.println(laccelZ_c );
  }

  // Read calibration status
  Wire_r.beginTransmission(BNO055_ADDRESS);
  Wire_r.write(0x35);//calibration address
  Wire_r.endTransmission(false);
  Wire_r.requestFrom(BNO055_ADDRESS, 1); // Request 1 byte for calibration status

  if (Wire_r.available()) 
  {
    byte calibStatus = Wire_r.read();
    // Interpret the calibration status
    int sysCalib = (calibStatus >> 6) & 0x03;
    int gyroCalib = (calibStatus >> 4) & 0x03;
    int accelCalib = (calibStatus >> 2) & 0x03;
    int magCalib = calibStatus & 0x03;
    
    Serial.print("Calibration - Sys: ");
    Serial.print(sysCalib);
    Serial.print(", Gyro: ");
    Serial.print(gyroCalib);
    Serial.print(", Accel: ");
    Serial.print(accelCalib);
    Serial.print(", Mag: ");
    Serial.println(magCalib);
  
  }
}
//this funtion reads the calibration and save then in a structure BNO_data
void read_calibration(BNO_data* data)
{
  byte calibStatus;
  Wire_r.beginTransmission(BNO055_ADDRESS);
  Wire_r.write(0x35);//calibration address
  Wire_r.endTransmission(false);
  Wire_r.requestFrom(BNO055_ADDRESS, 1); // Request 1 byte for calibration status
  if (Wire_r.available()) 
  {
    calibStatus = Wire_r.read();
    data->sysCalib = (calibStatus >> 6) & 0x03;
    data->gyroCalib = (calibStatus >> 4) & 0x03;
    data->accelCalib = (calibStatus >> 2) & 0x03;
    data->magCalib = calibStatus & 0x03;
  }

}
//this funtion reads the all the data and save then in a structure BNO_data
void read_data_all(BNO_data* data)
{
  uint8_t dataBuffer[18]; 
  Wire_r.beginTransmission(BNO055_ADDRESS); 
  Wire_r.write(0x08);
  Wire_r.endTransmission(false); 
  Wire_r.requestFrom(BNO055_ADDRESS, 18);  
  if(Wire_r.available() == 18) 
  {
    // Read the data into the dataBuffer
    for (int i = 0; i < 18; i++) 
    {
      dataBuffer[i] = Wire_r.read();
    }

    // Divide the data into variables 
    int16_t accelX = (dataBuffer[1] << 8) | dataBuffer[0];
    int16_t accelY = (dataBuffer[3] << 8) | dataBuffer[2];
    int16_t accelZ = (dataBuffer[5] << 8) | dataBuffer[4];
    int16_t magX = (dataBuffer[7] << 8) | dataBuffer[6];
    int16_t magY = (dataBuffer[9] << 8) | dataBuffer[8];
    int16_t magZ = (dataBuffer[11] << 8) | dataBuffer[10];
    int16_t gyroX = (dataBuffer[13] << 8) | dataBuffer[12];
    int16_t gyroY = (dataBuffer[15] << 8) | dataBuffer[14];
    int16_t gyroZ = (dataBuffer[17] << 8) | dataBuffer[16];


    // Convert raw accelerometer values 
    data->accelX = accelX / accelScale;
    data->accelY = accelY / accelScale;
    data->accelZ = accelZ / accelScale;


    // Convert raw magnetometer values to microteslas
    data->magX = magX / magScale;
    data->magY = magY / magScale;
    data->magZ = magZ / magScale;


    // Convert raw gyroscope values 
    data->gyroX  = gyroX / gyroScale;
    data->gyroY = gyroY / gyroScale;
    data->gyroZ = gyroZ / gyroScale;

  }
  Wire_r.beginTransmission(BNO055_ADDRESS);
  Wire_r.write(0x28);//linear_address
  Wire_r.endTransmission(false);
  Wire_r.requestFrom(BNO055_ADDRESS,6); 
  if (Wire_r.available() == 6) 
  {
    for (int i = 0; i < 6; i++) 
    {
      dataBuffer[i] = Wire_r.read();
    }
        // Divide the data into variables 
    int16_t accelX = (dataBuffer[1] << 8) | dataBuffer[0];
    int16_t accelY = (dataBuffer[3] << 8) | dataBuffer[2];
    int16_t accelZ = (dataBuffer[5] << 8) | dataBuffer[4];

    // Convert raw linear accel values 
    data->laccelX = accelX / accelScale;
    data->laccelY = accelY / accelScale;
    data->laccelZ = accelZ / accelScale;
  }

}

void read_accel(BNO_data* data)
{
  uint8_t dataBuffer[6]; // Buffer to store the received data
  Wire_r.beginTransmission(BNO055_ADDRESS); 
  Wire_r.write(0x08);
  Wire_r.endTransmission(false); // Send a restart condition
  Wire_r.requestFrom(BNO055_ADDRESS, 6);  
  if(Wire_r.available() == 6)
  {
    // Read the data into the dataBuffer
    for (int i = 0; i < 6; i++) 
    {
      dataBuffer[i] = Wire_r.read();
    }
    int16_t accelX = (dataBuffer[1] << 8) | dataBuffer[0];
    int16_t accelY = (dataBuffer[3] << 8) | dataBuffer[2];
    int16_t accelZ = (dataBuffer[5] << 8) | dataBuffer[4];
        // Convert raw accelerometer values 
    data->accelX = accelX / accelScale;
    data->accelY = accelY / accelScale;
    data->accelZ = accelZ / accelScale;

  }
}


void read_mad(BNO_data* data)
{
  uint8_t dataBuffer[6]; // Buffer to store the received data
  Wire_r.beginTransmission(BNO055_ADDRESS); 
  Wire_r.write(0X0E);
  Wire_r.endTransmission(false); // Send a restart condition
  Wire_r.requestFrom(BNO055_ADDRESS, 6);  
  if(Wire_r.available() == 6)
  {
    // Read the data into the dataBuffer
    for (int i = 0; i < 6; i++) 
    {
      dataBuffer[i] = Wire_r.read();
    }
    int16_t magX  = (dataBuffer[1] << 8) | dataBuffer[0];
    int16_t magY  = (dataBuffer[3] << 8) | dataBuffer[2];
    int16_t magZ  = (dataBuffer[5] << 8) | dataBuffer[4];
    // Convert raw magnetometer values to microteslas
    data->magX = magX / magScale;
    data->magY = magY / magScale;
    data->magZ = magZ / magScale;

  }
}

void read_gyro(BNO_data* data)
{
  uint8_t dataBuffer[6]; // Buffer to store the received data
  Wire_r.beginTransmission(BNO055_ADDRESS);
  Wire_r.write(0X14);
  Wire_r.endTransmission(false); // Send a restart condition
  Wire_r.requestFrom(BNO055_ADDRESS, 6);  
  if(Wire_r.available() == 6)
  {
    // Read the data into the dataBuffer
    for (int i = 0; i < 6; i++) 
    {
      dataBuffer[i] = Wire_r.read();
    }
    int16_t gyroX   = (dataBuffer[1] << 8) | dataBuffer[0];
    int16_t gyroY  = (dataBuffer[3] << 8) | dataBuffer[2];
    int16_t gyroZ   = (dataBuffer[5] << 8) | dataBuffer[4];
    // Convert raw gyroscope values 
    data->gyroX  = gyroX / gyroScale;
    data->gyroY = gyroY / gyroScale;
    data->gyroZ = gyroZ / gyroScale;

  }
}


void read_linear(BNO_data* data)
{
  uint8_t dataBuffer[6]; // Buffer to store the received data
  Wire_r.beginTransmission(BNO055_ADDRESS);
  Wire_r.write(0x28);//linear_address
  Wire_r.endTransmission(false);
  Wire_r.requestFrom(BNO055_ADDRESS,6); 
  if (Wire_r.available() == 6) 
  {
    for (int i = 0; i < 6; i++) 
    {
      dataBuffer[i] = Wire_r.read();
    }
        // Divide the data into variables 
    int16_t accelX = (dataBuffer[1] << 8) | dataBuffer[0];
    int16_t accelY = (dataBuffer[3] << 8) | dataBuffer[2];
    int16_t accelZ = (dataBuffer[5] << 8) | dataBuffer[4];

    // Convert raw linear accel values 
    data->laccelX = accelX / accelScale;
    data->laccelY = accelY / accelScale;
    data->laccelZ = accelZ / accelScale;
  }
}

