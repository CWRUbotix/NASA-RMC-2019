#include <SPI.h>

#define ADS1120_SPEED       2500000

#ifndef PAUSE_SHORT
#define PAUSE_SHORT __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
#endif

enum FLAGS {
    CONFIG_FOR_LOAD_CELL,
    CONFIG_FOR_POT
};

enum commands:uint8_t{
    com_reset = 0b00000110;
    com_start = 0b00001000; 
    com_powerdown = 0b00000010;
    com_rdata = 0b00010000;
    com_rreg = 0b00100000; //apply OR mask to lower nibble
    com_wreg = 0b01000000; //apply OR mask to lower nibble
};
class ADS1120{
  
  public:
    ADS1120(int cs_pin);
    void setup(uint8_t config);
    uint16_t read_channel(uint8_t);

  private:
    SPISettings ADC_SPI_settings;
    int cs_pin;
    void _set_single_shot(void);
    void _set_continuous_conv(void);
 
    /*
 * Reads registers from the chip
 * @param start first register to read
 * @param regs number of registers to be read
 * @param *buf pointer to a uint8_t array of length regs 
 * @return true if read was successful
 * Note: only the lower two bits of start and regs may be set, other  bits must be cleared 
 */
    inline bool _read_reg(uint8_t start, uint8_t regs, uint8_t *buf){
        SPI.beginTransaction(ADC_SPI_settings);
        digitalWrite(cs_pin,LOW);
        PAUSE_SHORT;
        uint8_t command = com_rreg + (regs - 1) + (start<<2);
        SPI.transfer(command);
        for( uint8_t i=0;i<regs;i++){
            buf[i] = SPI.transfer(0);
        }
        
        digitalWrite(cs_pin,HIGH);

        SPI.endTransaction();
         
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
        SPI.beginTransaction(ADC_SPI_settings);
        digitalWrite(cs_pin,LOW);
        PAUSE_SHORT;

        uint8_t command = com_wreg + (regs - 1) + (start<<2);
        SPI.transfer(command);
        for( uint8_t i=0;i<regs;i++){
            SPI.transfer(buf[i]);
        }
        
        digitalWrite(cs_pin,HIGH);
        SPI.endTransaction();
        return true; 
    }
}
