#ifndef I2c_interface
#define I2c_interface

#include <cstdint>
#include <cstddef>
#include "Arduino.h"
#include "twi.h"
#define BUFFER_LENGTH 136
#define CLOCK_STRETCH_TIMEOUT 15000
#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif

#ifndef TWI_BUFFER_LENGTH
#define TWI_BUFFER_LENGTH 32
#endif

#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4

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
  virtual void setClock(uint32_t frequency) = 0;
  virtual void configSDApin(uint8_t i) = 0;
  virtual void configSCLpin(uint8_t i) = 0;
  virtual bool wait_idle() = 0;
  virtual size_t write(uint8_t data) = 0;
  virtual size_t write(const uint8_t *data, size_t quantity) = 0;
  virtual bool force_clock() = 0;

  private:
  
};

class WIRE_TEENSY: public I2C_INTERFACE
{
public:
  //WIRE_TEENSY();
	static const uint8_t cnt_sda_pins = 2;
	static const uint8_t cnt_scl_pins = 2;
  typedef struct {
		const uint8_t 		pin;		// The pin number
		const uint32_t 		mux_val;	// Value to set for mux;
		volatile uint32_t	*select_input_register; // Which register controls the selection
		const uint32_t		select_val;	// Value for that selection
	} pin_info_t;

  typedef struct 
  {
    volatile uint32_t &clock_gate_register;
		uint32_t clock_gate_mask;
		pin_info_t sda_pins[cnt_sda_pins];
		pin_info_t scl_pins[cnt_scl_pins];
		IRQ_NUMBER_t irq_number;
		void (*irq_function)(void);
  } I2C_Hardware_t;
  static const I2C_Hardware_t i2c1_hardware;

  void begin();
  void begin(uint8_t address);
	void begin(int address) {
		begin((uint8_t)address);
	}
  void end();
  void beginTransmission(uint8_t address) 
  {
		txBuffer[0] = (address << 1);
		transmitting = 1;
		txBufferLength = 1;
	}
  void beginTransmission(int address) {
		beginTransmission((uint8_t)address);
	}
  virtual int available(void) {
		return rxBufferLength - rxBufferIndex;
	}
  uint8_t endTransmission(uint8_t sendStop);
	uint8_t endTransmission(void) {
		return endTransmission(1);
	}
  uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
  uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop) 
  {
    return requestFrom(address, quantity, (uint8_t)(sendStop ? 1 : 0));
  }
  uint8_t requestFrom(uint8_t address, uint8_t quantity) 
  {
    return requestFrom(address, quantity, (uint8_t)1);
  }
  uint8_t requestFrom(int address, int quantity, int sendStop)
  {
    return requestFrom((uint8_t)address, (uint8_t)quantity,
        (uint8_t)(sendStop ? 1 : 0));
  }
    uint8_t requestFrom(int address, int quantity) {
      return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)1);
    }
  uint8_t requestFrom(uint8_t addr, uint8_t qty, uint32_t iaddr, uint8_t n, uint8_t stop);
  int read(void);
  void setClock(uint32_t frequency);
  void configSDApin(uint8_t i);
  void configSCLpin(uint8_t i);
  bool wait_idle();
  size_t write(uint8_t data);
  size_t write(const uint8_t *data, size_t quantity);
  bool force_clock();


   WIRE_TEENSY(const uintptr_t _portAddr, const I2C_Hardware_t &myhardware)
		: portAddr(_portAddr), hardware(myhardware) {
	}
  void setWriteError(int err = 1) { write_error = err; }
  void isr(void);




private:
  const uintptr_t portAddr;
	uint8_t transmitting = 0;
  uint8_t slave_mode = 0;
  const I2C_Hardware_t &hardware;
  uint8_t	sda_pin_index_ = 0x0;	// default is always first item
	uint8_t	scl_pin_index_ = 0x0;
	uint8_t rxBuffer[BUFFER_LENGTH] = {};
  uint8_t rxBufferIndex= 0;
  uint8_t rxBufferLength = 0;
  uint8_t txBuffer[BUFFER_LENGTH+1] = {};
	uint8_t txBufferIndex = 0;
	uint8_t txBufferLength = 0;
  int write_error=0;
  void (*user_onRequest)(void) = nullptr;
  void (*user_onReceive)(int) = nullptr;

};

extern WIRE_TEENSY Wire_r;


//*/
#endif