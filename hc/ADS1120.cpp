#include "ADS1120.h"


ADS1120::ADS1120(int cs){
    this->cs_pin = cs;
    this->cur_mode = 0_single;
}

//set the gain to 2^n
//keep n at 7 or lower
void ADS1120::set_gain(uint8_t n){
    uint8_t config_reg = 0;
    _read_reg(0,1,&config_reg);
    config_reg &= 0b11110001;
    config_reg |= (n<<1);
    _write_reg(0,1,&config_reg);
    return; 
}

void ADS1120::set_mux_input(adc_mux_input mode){
    uint8_t config_reg = 0;
    _read_reg(0,1,&config_reg);
    config_reg &= 0b00001111;
    config_reg |= ( ((uint8_t)input) <<4);   
    _write_reg(0,1,&config_reg);
    this->cur_mux_input = input;
    return;
}

bool ADS1120::read_channel(adc_mux_input mode, uint16_t *output){

    //if the request matchtes the current mux mode then read out a value
    if(mode == this->cur_mux_input){

        digitalWrite(this->cs_pin, LOW);
        PAUSE_SHORT;
        SPI.beginTransation(this->ADC_SPI_settings);
        uint8_t data_msb = SPI.transfer(com_rdata);
        uint8_t data_lsb = SPI.transfer(0x00);
        SPI.endTransaction();
        digitalWrite(this->cs_pin,HIGH);
        uint16_t output = data_msb;
        output = output<<8;
        output += data_lsb;
        return true;
    //if the request does not match the current mux mode then change modes and read
    }else{
        uint8_t prev_mux_input = this->cur_mux_input;
        set_mux_input(mode);
        _set_single_shot();
        
        uint32_t count = 0;
        bool got_data = false;
        
        digitalWrite(this->cs_pin,LOW);
        PAUSE_SHORT;
        SPI.beginTransaction(this->ADC_SPI_settings);
        SPI.transfer(com_start);

        while(count<500 && !got_data){
            count++;
            got_data = !digitalRead(miso_pin);
        }

        if(!got_data) return false;
        uint8_t data_msb = SPI.transfer(0x00);
        uint8_t data_lsb = SPI.transfer(0x00);
        SPI.endTransaction();
        digitalWrite(this->cs_pin,HIGH);
        uint16_t output = data_msb;
        output = output<<8;
        output += data_lsb;
        _set_continuous_mode();
        set_mux_input(prev_mux_input);
        return true;
 
    }
           
}

