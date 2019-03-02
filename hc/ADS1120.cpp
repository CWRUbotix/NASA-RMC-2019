#include "ADS1120.h"


ADS1120::ADS1120(int cs){
    this->cs_pin = cs;
    this->ADC_SPI_settings = SPISettings(ADS1120_SPEED, MSBFIRST, SPI_MODE1);
}

void ADS1120::setup(uint8_t config){
	bool write_ok;
	uint8_t data[1] = {};
	// CONFIG REG 0
	if(config == CONFIG_FOR_POT){ 	// BYPASS THE PGA OR NOT
		bool read_ok = this->_read_reg(0, 1, data);
		data[0] |= 1; 				// sets bypass PGA to true
		write_ok = this->_write_reg(0, 1, data);
	}

	// CONFIG REG 1
	data[0] = (1 << 2); 			// SET CONTINUOUS CONVERSION MODE
	data[0] |= b01000000; 			// SET 180 SPS in TURBO MODE
	data[0] |= b00010000; 			// SET TURBO MODE
	write_ok = this->_write_reg(1, 1, data);

	// CONFIG REG 2
	data[0] = b10000000; 			//chooses AIN0 and AIN3 as vref

	if(config == CONFIG_FOR_LOAD_CELL){
		data[0] |= (1 << 3); 		// ENABLES LOW SIDE POWER SWITCH
	}
	write_ok = this->_write_reg(2, 1, data);

	// CONFIG REG 3
	data[0] = (1 << 1); 			// sets DRDYM to 1 (useful for single shot mode)
	write_ok = this->_write_reg(3, 1, data);
}

void ADS1120::_set_single_shot(){
	bool ok;
	uint8_t data[1] = {};
	ok = this->_read_reg(1, 1, data);
	data[0] &= ~(1 << 2);
	ok = this->_write_reg(1, 1, data);
}

void ADS1120::_set_continuous_conv(){
	bool ok;
	uint8_t data[1] = {};
	ok = this->_read_reg(1, 1, data);
	data[0] |= (1 << 2);
	ok = this->_write_reg(1, 1, data);
}
