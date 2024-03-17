#ifndef WIRE_TEENSY_H
#define WIRE_TEENSY_H

#include "I2c_interface.hpp"
#include <inttypes.h>
#include "Arduino.h"



#define BUFFER_LENGTH 136
#define CLOCK_STRETCH_TIMEOUT 15000


class WIRE_TEENSY: public virtual I2C_INTERFACE
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
  static const I2C_Hardware_t i2c3_hardware;
	static const I2C_Hardware_t i2c4_hardware;
  static const I2C_Hardware_t i2c2_hardware;

  void begin()override;
  void begin(uint8_t address)override;
	void begin(int address)override {
		begin((uint8_t)address);
	}
  void end()override;
  void beginTransmission(uint8_t address) override
  {
		txBuffer[0] = (address << 1);
		transmitting = 1;
		txBufferLength = 1;
	}
  void beginTransmission(int address) override{
		beginTransmission((uint8_t)address);
	}
  virtual int available(void)override {
		return rxBufferLength - rxBufferIndex;
	}
  uint8_t endTransmission(uint8_t sendStop)override;
	uint8_t endTransmission(void)override {
		return endTransmission(1);
	}
  uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)override;
  uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop) override
  {
    return requestFrom(address, quantity, (uint8_t)(sendStop ? 1 : 0));
  }
  uint8_t requestFrom(uint8_t address, uint8_t quantity) override
  {
    return requestFrom(address, quantity, (uint8_t)1);
  }
  uint8_t requestFrom(int address, int quantity, int sendStop)override
  {
    return requestFrom((uint8_t)address, (uint8_t)quantity,
        (uint8_t)(sendStop ? 1 : 0));
  }
    uint8_t requestFrom(int address, int quantity) override{
      return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)1);
    }
  uint8_t requestFrom(uint8_t addr, uint8_t qty, uint32_t iaddr, uint8_t n, uint8_t stop)override;
  int read(void) override;
  void setClock(uint32_t frequency) override;
  void configSDApin(uint8_t i) override;
  void configSCLpin(uint8_t i) override;
  bool wait_idle()override;
  size_t write(uint8_t data)override;
  size_t write(const uint8_t *data, size_t quantity)override;
  bool force_clock()override;
  //construtor
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
	uint8_t	scl_pin_index_ = 0x0; //
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
//constante qui remplace Wire
extern WIRE_TEENSY Wire_r;
extern WIRE_TEENSY Wire_1;
extern WIRE_TEENSY Wire_2;
#endif