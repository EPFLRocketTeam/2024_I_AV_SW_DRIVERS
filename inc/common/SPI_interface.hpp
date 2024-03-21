//
// Created by Samuel on 21/03/2024.
//

#ifndef INC_2024_I_AV_SW_DRIVERS_SPI_INTERFACE_HPP
#define INC_2024_I_AV_SW_DRIVERS_SPI_INTERFACE_HPP

#include <cstdint>

struct spi_settings_t {
	uint32_t cs;
	uint32_t speedMaximum;
	uint32_t dataOrder;
	uint32_t dataMode;
};

class SPI_Interface {
public:
	virtual bool begin(const spi_settings_t& settings) = 0;
	virtual void beginTransaction(uint32_t cs) = 0;
	virtual uint8_t transfer(uint8_t b) = 0;
	virtual void endTransaction() = 0;
};

#endif //INC_2024_I_AV_SW_DRIVERS_SPI_INTERFACE_HPP
