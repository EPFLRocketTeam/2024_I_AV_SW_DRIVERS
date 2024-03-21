//
// Created by Samuel on 21/03/2024.
//

#ifndef INC_2024_I_AV_SW_DRIVERS_LORA_HPP
#define INC_2024_I_AV_SW_DRIVERS_LORA_HPP

#include "SPI_interface.hpp"

#define LORA_DEFAULT_SPI           SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2

#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1
#define PA_BOOST                 0x80

enum LoRaRegisters {
	REG_FIFO = 0x00,
	REG_OP_MODE = 0x01,
	REG_FRF_MSB = 0x06,
	REG_FRF_MID = 0x07,
	REG_FRF_LSB = 0x08,
	REG_PA_CONFIG = 0x09,
	REG_OCP = 0x0b,
	REG_LNA = 0x0c,
	REG_FIFO_ADDR_PTR = 0x0d,
	REG_FIFO_TX_BASE_ADDR = 0x0e,
	REG_FIFO_RX_BASE_ADDR = 0x0f,
	REG_FIFO_RX_CURRENT_ADDR = 0x10,
	REG_IRQ_FLAGS = 0x12,
	REG_RX_NB_BYTES = 0x13,
	REG_PKT_SNR_VALUE = 0x19,
	REG_PKT_RSSI_VALUE = 0x1a,
	REG_RSSI_VALUE = 0x1b,
	REG_MODEM_CONFIG_1 = 0x1d,
	REG_MODEM_CONFIG_2 = 0x1e,
	REG_PREAMBLE_MSB = 0x20,
	REG_PREAMBLE_LSB = 0x21,
	REG_PAYLOAD_LENGTH = 0x22,
	REG_MODEM_CONFIG_3 = 0x26,
	REG_FREQ_ERROR_MSB = 0x28,
	REG_FREQ_ERROR_MID = 0x29,
	REG_FREQ_ERROR_LSB = 0x2a,
	REG_RSSI_WIDEBAND = 0x2c,
	REG_DETECTION_OPTIMIZE = 0x31,
	REG_INVERTIQ = 0x33,
	REG_DETECTION_THRESHOLD = 0x37,
	REG_SYNC_WORD = 0x39,
	REG_INVERTIQ2 = 0x3b,
	REG_DIO_MAPPING_1 = 0x40,
	REG_VERSION = 0x42,
	REG_PA_DAC =  0x4d,
};

enum LoRaModes {
	MODE_LONG_RANGE_MODE = 0x80,
	MODE_SLEEP = 0x00,
	MODE_STDBY = 0x01,
	MODE_TX = 0x03,
	MODE_RX_CONTINUOUS = 0x05,
	MODE_RX_SINGLE = 0x06,
	MODE_CAD = 0x07,
};

enum LoRaIRQ {
	IRQ_TX_DONE_MASK = 0x08,
	IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20,
	IRQ_RX_DONE_MASK = 0x40,
	IRQ_CAD_DONE_MASK = 0x04,
	IRQ_CAD_DETECTED_MASK = 0x01,
};

#define LORA_PACKET_SIZE 256

#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

struct lora_packet_t {
	union {
		uint8_t raw[LORA_PACKET_SIZE];

		struct {
			uint8_t dest;
			uint8_t localAddr;
			uint8_t id;
		} meta;

		struct {

		} data;
	};
};

/* MUST SET UP INTERRUPTS & INIT PINS */
class LoRaClass {
public:
	LoRaClass(uint8_t addr, SPI_Interface* spi, const spi_settings_t& spiSettings, uint32_t rst, uint32_t irq, uint32_t minIntervalMs);

	bool begin(long frequency);
	void end();

	void writeRegister(uint8_t addr, uint8_t value);
	uint8_t readRegister(uint8_t addr);

	void receive(int payloadSize = 0);
	void sendPacket(const lora_packet_t& packet);

	void onReceive(void(*callback)(int));
	void onTransmissionDone(void(*callback)());
	void onCanDone(void(*callback)(bool));

	void implicitHeaderMode();
	void explicitHeaderMode();

	void idle();
	void sleep();

	bool available();
	bool read(uint8_t& out);
	bool peek(uint8_t& out);

	void onInterrupt();

	void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
	void setFrequency(long frequency);
	void setSpreadingFactor(int sf);
	void setSignalBandwidth(long sbw);
	void setCodingRate4(int denominator);
	void setPreambleLength(long length);
	void setSyncWord(int sw);
	void enableCrc();
	void disableCrc();
	void enableInvertIQ();
	void disableInvertIQ();
	void enableLowDataRateOptimize();
	void disableLowDataRateOptimize();
	bool isTransmitting();
	int parsePacket(int size);
	int packetRssi();
	float packetSnr();
	long packetFrequencyError();
	int rssi();
	void channelActivityDetection(void);
	int getSpreadingFactor();
	long getSignalBandwidth();
	void setLdoFlag();
	void setLdoFlagForced(const bool ldoOn);
	void setOCP(uint8_t mA);
	void setGain(uint8_t gain);
	uint8_t random();
private:

	uint8_t _addr;
	uint32_t _rst;
	uint32_t _irq;
	uint32_t _minIntervalMs;
	int _implicitHeaderMode;
	spi_settings_t _spiSettings;

	SPI_Interface* _spi;
	long _frequency;
	int _packetIndex;
	void (*_onReceive)(int);
	void (*_onTransmissionDone)();
	void (*_onCadDone)(bool);
};

#endif //INC_2024_I_AV_SW_DRIVERS_LORA_HPP
