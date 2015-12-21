#include "stm32f10x.h"

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_i2c.h"
#include "misc.h"
#include "ad5934.h"
#include "I2CRoutines.h"
#include "delays.h"

uint8_t range_mode = HIGH_RANGE_CONDUCTIVITY;
uint32_t start_freq = 1950; // Set start freq, < 100Khz
uint32_t incre_freq = 975; // Set freq increment
uint8_t adg_conf[2];


// init ADG715 switch to use for EC or Temp measurements, or EC or Temp calibration
void init_adg715(uint8_t mode){
	range_mode = mode;
	adg_conf[0] = mode;
	uint8_t i=0;
	// write config
	I2C_Master_BufferWrite(EVAL0349_I2CX,adg_conf,1,Polling, ADG715_ADDR);
	Delay_us_(100000);
	// and read it back for assurance. Could be disabled to enhance the speed
	I2C_Master_BufferRead(EVAL0349_I2CX, adg_conf, 1,DMA,ADG715_ADDR);
	Delay_us_(100000);
}

// setup for AD5934 (following the AN-1252)
void init_ad5934(void){
  // 1. Reset
  ad5934_write_data(AD5934_CTRL_R1, 0x01);    // range 1, gain x1
  ad5934_write_data(AD5934_CTRL_R1, 0x01);    // range 1, gain x1
  ad5934_write_data(AD5934_CTRL_R2, 0x10);

  // 2. Program Start Frequency
  ad5934_write_data(AD5934_START_FREQ_R1, 0x10);
  ad5934_write_data(AD5934_START_FREQ_R2, 0xF3);
  ad5934_write_data(AD5934_START_FREQ_R3, 0xC5);

  // 3. Define frequency increment
  ad5934_write_data(AD5934_FREG_INC_R1, 0x00);
  ad5934_write_data(AD5934_FREG_INC_R2, 0x79);
  ad5934_write_data(AD5934_FREG_INC_R3, 0xE2);

  // 4. Program the number of increments
  ad5934_write_data(AD5934_NUM_INC_R1, 0x00);
  ad5934_write_data(AD5934_NUM_INC_R2, 0x0A);

  // 5. Program the delay of measurments. Worst case at maximum freq
  // Fmax = 1950 + (975*10) = 11700Hz. 1ms * 11700 ~=12ms
  ad5934_write_data(AD5934_MEAS_DELAY_R1, 0x00);
  ad5934_write_data(AD5934_MEAS_DELAY_R2, 0x0C);

  // 6. Initialize the system
  ad5934_write_data(AD5934_CTRL_R1, 0x11);

}



void ad5934_write_data(int addr, int data){
	uint8_t buf[2];
	buf[0] = addr;	// AD5934 register address to be written
	buf[1] = data;	// register data to be written
	I2C_Master_BufferWrite(EVAL0349_I2CX,buf,2,DMA, AD5934_ADDR);

}

// set pointer and read the data from AD5934
uint8_t ad5934_read_data(int addr){
	uint8_t buf[2];
	buf[0] = AD5934_SET_ADDR_PNTR;
	buf[1] = addr;

	// send Set Pointer command to AD5934
	I2C_Master_BufferWrite(EVAL0349_I2CX,buf,2,DMA, AD5934_ADDR);

	Delay_us_(1000);
	// Read the data from AD5934. It sends the data from the register address with pointer set
	I2C_Master_BufferRead(EVAL0349_I2CX, buf, 1,DMA,AD5934_ADDR);
	vTaskDelay(50);
	return buf[0];
}

/*
uint8_t ad5934_read_data_byte(int addr){
	uint8_t buf[2];
	buf[0] = AD5934_SET_ADDR_PNTR;
	buf[1] = addr;

	I2C_Master_BufferWrite(EVAL0349_I2CX,buf,2,DMA, AD5934_ADDR);

	Delay_us_(1000);
	I2C_Master_BufferRead(EVAL0349_I2CX, buf, 1,DMA,AD5934_ADDR);
	vTaskDelay(50);
	return buf[0];
}
*/

// convert signed integer to unsigned one
uint8_t mod4int(uint8_t arg){
	if (arg>127) {
		arg = ~(arg) + 1;
	}
	return arg;

}

// read AD5934
uint32_t runSweep(uint8_t mode) {
    uint32_t ec = 0;
	uint16_t re = 0;
	uint16_t img = 0;
	uint32_t freq = 0;
	uint32_t mag = 0;
	uint32_t phase = 0;
	uint32_t gain = 0;
	uint32_t impedance = 0;
	uint32_t increment = 0;
	uint32_t tmp = 0;
	uint8_t i = 0;
	uint8_t flag = 0;
	uint16_t R1 = 0;
	uint16_t R2 = 0;

	uint8_t ctrl_r1 = 0;
	uint8_t ctrl_r2 = 0;
	uint8_t ctrl_r3 = 0;
	uint8_t ctrl_r4 = 0;


	// 1. Standby '10110000' Mask D8-10 of avoid tampering with gains
	ad5934_write_data(AD5934_CTRL_R1,(ad5934_read_data(AD5934_CTRL_R1) & 0x07) | 0xB0);

	// 2. Initialize sweep
	ad5934_write_data(AD5934_CTRL_R1,(ad5934_read_data(AD5934_CTRL_R1) & 0x07) | 0x10);

	// 3. Start sweep
	ad5934_write_data(AD5934_CTRL_R1,(ad5934_read_data(AD5934_CTRL_R1) & 0x07) | 0x20);

	vTaskDelay(100);

	while((ad5934_read_data(AD5934_STATUS_REG) & 0x07) < 4 ) {  // Check that status reg != 4, sweep not complete
		vTaskDelay(100); // delay between measurements

		flag = ad5934_read_data(AD5934_STATUS_REG)& 2;

		if (flag==2) {
			// read two bytes of real part
			R1 = ad5934_read_data(AD5934_REAL_DATA_R1);
			R2 = ad5934_read_data(AD5934_REAL_DATA_R2);
			re = (R1 << 8) | R2;
			re &= ~(1 << 15);

			// and read two another bytes of Imaginary part
			R1  = mod4int(ad5934_read_data(AD5934_IMG_DATA_R1));
			R2  = mod4int(ad5934_read_data(AD5934_IMG_DATA_R2));

			img = (R1 << 8) | R2;

			// here we need to calculate square root of powed real + powed imaginary
			// first calculate (real^2+img^2)
			tmp = (re*re)+(img*img);
			mag = asqrt(tmp);			// and square root
//			ec += freq;
			vTaskDelay(50);

			// detect edge of range to switch feedback resistor
            if (mag>15000 && range_mode==LOW_RANGE_CONDUCTIVITY) {
            	init_adg715(HIGH_RANGE_CONDUCTIVITY);
            }
            else if (mag<2500 && range_mode==HIGH_RANGE_CONDUCTIVITY) {
            	init_adg715(LOW_RANGE_CONDUCTIVITY);
            }

            else {	// if no range switches, then calculate current EC
				if (range_mode==LOW_RANGE_CONDUCTIVITY) {
					ec = mag/10;	// calculate EC for 25uS..2500uS range (CN0349)
					return ec;
				}
				else if (range_mode==HIGH_RANGE_CONDUCTIVITY) {
					ec = mag;		// calculate EC for 0.5ms..200ms range
					return ec;
				}
				else {
					return ~(ec);	// wrong reading
				}
            }

            return ~(ec);			// wrong reading

		}
	}
	ad5934_write_data(AD5934_CTRL_R1,(ad5934_read_data(AD5934_CTRL_R1) & 0x07) | 0xA0);
}










uint16_t asqrt(uint32_t x) {
  /*      From http://medialab.freaknet.org/martin/src/sqrt/sqrt.c
   *   Logically, these are unsigned. We need the sign bit to test
   *   whether (op - res - one) underflowed.
   */
  uint32_t op, res, one;

  op = x;
  res = 0;

  /* "one" starts at the highest power of four <= than the argument. */

  one = 1 << 30;   /* second-to-top bit set */
  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op = op - (res + one);
      res = res +  2 * one;
    }
    res /= 2;
    one /= 4;
  }
  return (uint16_t) (res);
}
