#ifndef I2c_interface
#define I2c_interface

#include <cstdint>
#include <cstddef>

class I2C_INTERFACE
{
  public:
  I2C_INTERFACE(){}
  
  virtual void begin() = 0;
  virtual void begin(uint8_t address) = 0;
  virtual void begin(int address) = 0;
  virtual void end() = 0;
  virtual void beginTransmission(uint8_t address) = 0;
  virtual void beginTransmission(int address) = 0;
  virtual uint8_t endTransmission(uint8_t sendStop) = 0;
  virtual uint8_t endTransmission(void) = 0;
  virtual uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) = 0;
  virtual uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop) = 0;
  virtual uint8_t requestFrom(uint8_t address, uint8_t quantity) = 0;
  virtual uint8_t requestFrom(int address, int quantity, int sendStop) = 0;
  virtual uint8_t requestFrom(int address, int quantity) = 0;
  virtual uint8_t requestFrom(uint8_t addr, uint8_t qty, uint32_t iaddr, uint8_t n, uint8_t stop) = 0;
  virtual int read(void) = 0;
  virtual int available(void)=0;
  virtual void setClock(uint32_t frequency) = 0;
  virtual void configSDApin(uint8_t i) = 0;
  virtual void configSCLpin(uint8_t i) = 0;
  virtual bool wait_idle() = 0;
  virtual size_t write(uint8_t data) = 0;
  virtual size_t write(const uint8_t *data, size_t quantity) = 0;
  virtual bool force_clock() = 0;

  private:
  
};



#endif