//
// Created by Samuel on 21/03/2024.
//

#include "LoRa.hpp"

#define bitWrite(x,n,b) x = ((b) ? x | (1U << n) : x & ~(1U<<n))

LoRaClass::LoRaClass(uint8_t addr, SPI_Interface *spi, const spi_settings_t &spiSettings, uint32_t rst, uint32_t irq, uint32_t minIntervalMs) {
	_addr = addr;
	_spi = spi;
	_spiSettings = spiSettings;
	_rst = rst;
	_irq = irq;
	_minIntervalMs = minIntervalMs;
}

bool LoRaClass::begin(long frequency) {

	// start SPI
	_spi->begin(_spiSettings);

	// check version
	uint8_t version = readRegister(LoRaRegisters::REG_VERSION);
	if (version != 0x12) {
		return false;
	}

	// put in sleep mode
	sleep();

	// set frequency
	setFrequency(frequency);

	// set base addresses
	writeRegister(LoRaRegisters::REG_FIFO_TX_BASE_ADDR, 0);
	writeRegister(LoRaRegisters::REG_FIFO_RX_BASE_ADDR, 0);

	// set LNA boost
	writeRegister(LoRaRegisters::REG_LNA, readRegister(LoRaRegisters::REG_LNA) | 0x03);

	// set auto AGC
	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_3, 0x04);

	// set output power to 17 dBm
	setTxPower(17);

	// put in standby mode
	idle();

	return true;
}

void LoRaClass::end() {
	sleep();
}

/**/

void LoRaClass::writeRegister(uint8_t addr, uint8_t value) {
	_spi->beginTransaction(_spiSettings.cs);
	_spi->transfer(addr | 0x80);
	_spi->transfer(value);
	_spi->endTransaction();
}

uint8_t LoRaClass::readRegister(uint8_t addr) {
	_spi->beginTransaction(_spiSettings.cs);
	_spi->transfer(addr & 0x7f);
	uint8_t response = _spi->transfer(0x00);
	_spi->endTransaction();
	return response;
}

/**/

void LoRaClass::receive(int payloadSize) {
	writeRegister(LoRaRegisters::REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE
	if (payloadSize > 0) {
		implicitHeaderMode();

		writeRegister(LoRaRegisters::REG_PAYLOAD_LENGTH, payloadSize & 0xff); // min(payloadSize, 0xff)
	} else {
		explicitHeaderMode();
	}
}

void LoRaClass::sendPacket(const lora_packet_t &packet) {
	// write data
	for (unsigned char data : packet.raw) {
		writeRegister(LoRaRegisters::REG_FIFO, data);
	}

	writeRegister(LoRaRegisters::REG_PAYLOAD_LENGTH, (uint8_t) LORA_PACKET_SIZE);
}

/**/

void LoRaClass::onReceive(void (*callback)(int)) {
	_onReceive = callback;
}

void LoRaClass::onTransmissionDone(void (*callback)()) {
	_onTransmissionDone = callback;
}

void LoRaClass::onCanDone(void (*callback)(bool)) {
	_onCadDone = callback;
}

/**/

void LoRaClass::implicitHeaderMode() {
	_implicitHeaderMode = 1;
	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_1, readRegister(LoRaRegisters::REG_MODEM_CONFIG_1) | 0x01);

}

void LoRaClass::explicitHeaderMode() {
	_implicitHeaderMode = 0;
	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_1, readRegister(LoRaRegisters::REG_MODEM_CONFIG_1) & 0xfe);
}

/**/

bool LoRaClass::available() {
	return (readRegister(LoRaRegisters::REG_RX_NB_BYTES) - _packetIndex);
}

bool LoRaClass::read(uint8_t& out) {
	if(!available())
		return false;

	_packetIndex++;
	out = readRegister(LoRaRegisters::REG_FIFO);
	return true;
}

bool LoRaClass::peek(uint8_t &out) {
	if(!available())
		return false;

	uint32_t currentAddr = readRegister(LoRaRegisters::REG_FIFO_ADDR_PTR);
	out = readRegister(LoRaRegisters::REG_FIFO);

	writeRegister(LoRaRegisters::REG_FIFO_ADDR_PTR, currentAddr);
	return true;
}

/**/

void LoRaClass::idle() {
	writeRegister(LoRaRegisters::REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaClass::sleep() {
	writeRegister(LoRaRegisters::REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**/

void LoRaClass::onInterrupt() {
	int irqFlags = readRegister(LoRaRegisters::REG_IRQ_FLAGS);

	// clear IRQ's
	writeRegister(LoRaRegisters::REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_CAD_DONE_MASK) != 0) {
		return;
		if (_onCadDone) {
			_onCadDone((irqFlags & LoRaIRQ::IRQ_CAD_DETECTED_MASK) != 0);
		}
	} else if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
			// received a packet
			_packetIndex = 0;

			// read packet length
			int packetLength = _implicitHeaderMode ? readRegister(LoRaRegisters::REG_PAYLOAD_LENGTH) : readRegister(LoRaRegisters::REG_RX_NB_BYTES);

			// set FIFO address to current RX address
			writeRegister(LoRaRegisters::REG_FIFO_ADDR_PTR, readRegister(LoRaRegisters::REG_FIFO_RX_CURRENT_ADDR));

			if (_onReceive) {
				_onReceive(packetLength);
			}
		} else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
			if (_onTransmissionDone) {
				_onTransmissionDone();
			}
		}
	}
}

/**/

bool LoRaClass::isTransmitting() {
	if ((readRegister(LoRaRegisters::REG_OP_MODE) & MODE_TX) == MODE_TX) {
		return true;
	}

	if (readRegister(LoRaRegisters::REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
		// clear IRQ's
		writeRegister(LoRaRegisters::REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}

	return false;
}

bool LoRaClass::parsePacket(int size) {
	if(size != LORA_PACKET_SIZE)
		return false;

	int packetLength = 0;
	int irqFlags = readRegister(LoRaRegisters::REG_IRQ_FLAGS);

	implicitHeaderMode();
	writeRegister(LoRaRegisters::REG_PAYLOAD_LENGTH, LORA_PACKET_SIZE & 0xff);

	// clear IRQ's
	writeRegister(LoRaRegisters::REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		// received a packet
		_packetIndex = 0;

		// read packet length
		if (_implicitHeaderMode) {
			packetLength = readRegister(LoRaRegisters::REG_PAYLOAD_LENGTH);
		} else {
			packetLength = readRegister(LoRaRegisters::REG_RX_NB_BYTES);
		}

		// set FIFO address to current RX address
		writeRegister(LoRaRegisters::REG_FIFO_ADDR_PTR, readRegister(LoRaRegisters::REG_FIFO_RX_CURRENT_ADDR));

		// put in standby mode
		idle();
	} else if (readRegister(LoRaRegisters::REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
		// not currently in RX mode

		// reset FIFO address
		writeRegister(LoRaRegisters::REG_FIFO_ADDR_PTR, 0);

		// put in single RX mode
		writeRegister(LoRaRegisters::REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}

	return true;
}

int LoRaClass::packetRssi() {
	return (readRegister(LoRaRegisters::REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRaClass::packetSnr() {
	return ((int8_t)readRegister(LoRaRegisters::REG_PKT_SNR_VALUE)) * 0.25;
}

long LoRaClass::packetFrequencyError() {
	int32_t freqError = 0;
	freqError = static_cast<int32_t>(readRegister(LoRaRegisters::REG_FREQ_ERROR_MSB) & 0b111);
	freqError <<= 8L;
	freqError += static_cast<int32_t>(readRegister(LoRaRegisters::REG_FREQ_ERROR_MID));
	freqError <<= 8L;
	freqError += static_cast<int32_t>(readRegister(LoRaRegisters::REG_FREQ_ERROR_LSB));

	if (readRegister(LoRaRegisters::REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
		freqError -= 524288; // 0b1000'0000'0000'0000'0000
	}

	const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
	const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

	return static_cast<long>(fError);
}

int LoRaClass::rssi(){
	return (readRegister(LoRaRegisters::REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

void LoRaClass::channelActivityDetection(void) {
	writeRegister(LoRaRegisters::REG_DIO_MAPPING_1, 0x80);// DIO0 => CADDONE
	writeRegister(LoRaRegisters::REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

void LoRaClass::setTxPower(int level, int outputPin){
	if (PA_OUTPUT_RFO_PIN == outputPin) {
		// RFO
		if (level < 0) {
			level = 0;
		} else if (level > 14) {
			level = 14;
		}

		writeRegister(LoRaRegisters::REG_PA_CONFIG, 0x70 | level);
	} else {
		// PA BOOST
		if (level > 17) {
			if (level > 20) {
				level = 20;
			}

			// subtract 3 from level, so 18 - 20 maps to 15 - 17
			level -= 3;

			// High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
			writeRegister(LoRaRegisters::REG_PA_DAC, 0x87);
			setOCP(140);
		} else {
			if (level < 2) {
				level = 2;
			}
			//Default value PA_HF/LF or +17dBm
			writeRegister(LoRaRegisters::REG_PA_DAC, 0x84);
			setOCP(100);
		}

		writeRegister(LoRaRegisters::REG_PA_CONFIG, PA_BOOST | (level - 2));
	}
}

void LoRaClass::setFrequency(long frequency){
	_frequency = frequency;

	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	writeRegister(LoRaRegisters::REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeRegister(LoRaRegisters::REG_FRF_MID, (uint8_t)(frf >> 8));
	writeRegister(LoRaRegisters::REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LoRaClass::getSpreadingFactor(){
	return readRegister(LoRaRegisters::REG_MODEM_CONFIG_2) >> 4;
}

void LoRaClass::setSpreadingFactor(int sf){
	if (sf < 6) {
		sf = 6;
	} else if (sf > 12) {
		sf = 12;
	}

	if (sf == 6) {
		writeRegister(LoRaRegisters::REG_DETECTION_OPTIMIZE, 0xc5);
		writeRegister(LoRaRegisters::REG_DETECTION_THRESHOLD, 0x0c);
	} else {
		writeRegister(LoRaRegisters::REG_DETECTION_OPTIMIZE, 0xc3);
		writeRegister(LoRaRegisters::REG_DETECTION_THRESHOLD, 0x0a);
	}

	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_2, (readRegister(LoRaRegisters::REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
	setLdoFlag();
}

long LoRaClass::getSignalBandwidth(){
	uint8_t bw = (readRegister(LoRaRegisters::REG_MODEM_CONFIG_1) >> 4);

	switch (bw) {
		case 0: return 7.8E3;
		case 1: return 10.4E3;
		case 2: return 15.6E3;
		case 3: return 20.8E3;
		case 4: return 31.25E3;
		case 5: return 41.7E3;
		case 6: return 62.5E3;
		case 7: return 125E3;
		case 8: return 250E3;
		case 9: return 500E3;
	}

	return -1;
}

void LoRaClass::setSignalBandwidth(long sbw){
	int bw;

	if (sbw <= 7.8E3) {
		bw = 0;
	} else if (sbw <= 10.4E3) {
		bw = 1;
	} else if (sbw <= 15.6E3) {
		bw = 2;
	} else if (sbw <= 20.8E3) {
		bw = 3;
	} else if (sbw <= 31.25E3) {
		bw = 4;
	} else if (sbw <= 41.7E3) {
		bw = 5;
	} else if (sbw <= 62.5E3) {
		bw = 6;
	} else if (sbw <= 125E3) {
		bw = 7;
	} else if (sbw <= 250E3) {
		bw = 8;
	} else /*if (sbw <= 250E3)*/ {
		bw = 9;
	}

	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_1, (readRegister(LoRaRegisters::REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
	setLdoFlag();
}

void LoRaClass::setLdoFlag(){
	// Section 4.1.1.5
	long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

	// Section 4.1.1.6
	bool ldoOn = symbolDuration > 16;

	uint8_t config3 = readRegister(LoRaRegisters::REG_MODEM_CONFIG_3);
	bitWrite(config3, 3, ldoOn);
	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_3, config3);
}

void LoRaClass::setLdoFlagForced(const bool ldoOn){
	uint8_t config3 = readRegister(LoRaRegisters::REG_MODEM_CONFIG_3);
	bitWrite(config3, 3, ldoOn);
	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_3, config3);
}

void LoRaClass::setCodingRate4(int denominator) {
	if (denominator < 5) {
		denominator = 5;
	} else if (denominator > 8) {
		denominator = 8;
	}

	int cr = denominator - 4;

	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_1, (readRegister(LoRaRegisters::REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRaClass::setPreambleLength(long length){
	writeRegister(LoRaRegisters::REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
	writeRegister(LoRaRegisters::REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRaClass::setSyncWord(int sw){
	writeRegister(LoRaRegisters::REG_SYNC_WORD, sw);
}

void LoRaClass::enableCrc(){
	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_2, readRegister(LoRaRegisters::REG_MODEM_CONFIG_2) | 0x04);
}

void LoRaClass::disableCrc(){
	writeRegister(LoRaRegisters::REG_MODEM_CONFIG_2, readRegister(LoRaRegisters::REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaClass::enableInvertIQ(){
	writeRegister(LoRaRegisters::REG_INVERTIQ,  0x66);
	writeRegister(LoRaRegisters::REG_INVERTIQ2, 0x19);
}

void LoRaClass::disableInvertIQ(){
	writeRegister(LoRaRegisters::REG_INVERTIQ,  0x27);
	writeRegister(LoRaRegisters::REG_INVERTIQ2, 0x1d);
}

void LoRaClass::enableLowDataRateOptimize(){
	setLdoFlagForced(true);
}

void LoRaClass::disableLowDataRateOptimize(){
	setLdoFlagForced(false);
}

void LoRaClass::setOCP(uint8_t mA)
{
	uint8_t ocpTrim = 27;

	if (mA <= 120) {
		ocpTrim = (mA - 45) / 5;
	} else if (mA <=240) {
		ocpTrim = (mA + 30) / 10;
	}

	writeRegister(LoRaRegisters::REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRaClass::setGain(uint8_t gain)
{
	// check allowed range
	if (gain > 6) {
		gain = 6;
	}

	// set to standby
	idle();

	// set gain
	if (gain == 0) {
		// if gain = 0, enable AGC
		writeRegister(LoRaRegisters::REG_MODEM_CONFIG_3, 0x04);
	} else {
		// disable AGC
		writeRegister(LoRaRegisters::REG_MODEM_CONFIG_3, 0x00);

		// clear Gain and set LNA boost
		writeRegister(LoRaRegisters::REG_LNA, 0x03);

		// set gain
		writeRegister(LoRaRegisters::REG_LNA, readRegister(LoRaRegisters::REG_LNA) | (gain << 5));
	}
}

uint8_t LoRaClass::random(){
	return readRegister(LoRaRegisters::REG_RSSI_WIDEBAND);
}
