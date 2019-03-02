#include <SPI.h>


enum commands:uint8_t{
    com_reset = 0b00000110;
    com_start = 0b00001000; 
    com_powerdown = 0b00000010;
    com_rdata = 0b00010000;
    com_rreg = 0b00100000; //apply OR mask to lower nibble
    com_wreg = 0b01000000; //apply OR mask to lower nibble
};


//nibble for mux
enum adc_mux_input:uint8_t{
    0_1_diff = 0x0;
    2_3_diff = 0x5,
    0_single = 0x8,
    1_single = 0x9,
    2_single = 0xA,
    3_single = 0xB
};

class ADS1120{
  
  public:

    bool read_channel(adc_mux_input mode, uint16_t *output);
    void set_mux_input(adc_mux_input mode);
    void set_gain(uint8_t gain);

  private:
    constexpr int miso_pin = 12; 
    mux_input cur_mux_input;  
    int cs_pin;
 
    /*
 * Reads registers from the chip
 * @param start first register to read
 * @param regs number of registers to be read
 * @param *buf pointer to a uint8_t array of length regs 
 * @return true if read was successful
 * Note: only the lower two bits of start and regs may be set, other  bits must be cleared 
 */
    inline bool _read_reg(uint8_t start, uint8_t regs, uint8_t *buf){
        digitalWrite(cs_pin,HIGH);
         
        uint8_t command = com_rreg + (regs - 1) + (start<<2);
        SPI.transfer(command);
        for( uint8_t i=0;i<regs;i++){
            buf[i] = SPI.transfer(0);
        }
        
        digitalWrite(cs_pin,LOW);
         
        return true; 
    } 


    /*
 * Writes registers to the chip
 * @param start first register to write to 
 * @param regs number of registers to be written to 
 * @param *buf pointer to a uint8_t array of length regs 
 * @return true if write was successful
 * Note: only the lower two bits of start and regs are read
 */
    inline bool _write_reg(uint8_t start, uint8_t regs, uint8_t *buf){
        
        digitalWrite(cs_pin,HIGH);
                
        uint8_t command = com_wreg + (regs - 1) + (start<<2);
        SPI.transfer(command);
        for( uint8_t i=0;i<regs;i++){
            SPI.transfer(buf[i]);
        }
        
        digitalWrite(cs_pin,LOW);
        
        return true; 
    }
}
