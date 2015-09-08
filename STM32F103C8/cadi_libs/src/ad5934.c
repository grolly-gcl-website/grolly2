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


void init_adg715(uint8_t mode){
	range_mode = mode;
	adg_conf[0] = mode;
	uint8_t i=0;


		I2C_Master_BufferWrite(I2C2,adg_conf,1,Polling, ADG715_ADDR);
		Delay_us_(100000);


		// huge piece of code for ADG715 specific readout
		__IO uint32_t temp = 0;
		__IO uint32_t Timeout = 0;
		uint8_t Address = 0;
					   Timeout = 0xFFFF;
					/* Send START condition */
					I2C2->CR1 |= CR1_START_Set;
					/* Wait until SB flag is set: EV5  */
					while ((I2C2->SR1&0x0001) != 0x0001)
					{
						if (Timeout-- == 0)
							return Error;
					}
					/* Send slave address */
					/* Reset the address bit0 for read */
					//SlaveAddress |= OAR1_ADD0_Set;
					Address = (ADG715_ADDR|1);
					/* Send the slave address */
					I2C2->DR = Address;
					/* Wait until ADDR is set: EV6_3, then program ACK = 0, clear ADDR
					and program the STOP just after ADDR is cleared. The EV6_3
					software sequence must complete before the current byte end of transfer.*/
					/* Wait until ADDR is set */
					Timeout = 0xFFFF;
					while ((I2C2->SR1&0x0002) != 0x0002)
					{
						if (Timeout-- == 0)
							return Error;
					}
					/* Clear ACK bit */
					I2C2->CR1 &= CR1_ACK_Reset;
					/* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
					software sequence must complete before the current byte end of transfer */
					__disable_irq();
					/* Clear ADDR flag */
					temp = I2C2->SR2;
					/* Program the STOP */
					I2C2->CR1 |= CR1_STOP_Set;
					/* Re-enable IRQs */
					__enable_irq();
					/* Wait until a data is received in DR register (RXNE = 1) EV7 */
					while ((I2C2->SR1 & 0x00040) != 0x000040);
					/* Read the data */
					adg_conf[1] = I2C2->DR;
					/* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
					while ((I2C2->CR1&0x200) == 0x200);
					/* Enable Acknowledgement to be ready for another reception */
					I2C2->CR1 |= CR1_ACK_Set;
		Delay_us_(100000);

}


uint8_t ad5934_read_byte(uint8_t addr){
	uint8_t buf = 0;
		uint8_t i = 0;
		for (i=0;i<8;i++) {		// send address
			gpio_set(11,(AD5934_ADDR|1),i);
			Delay_us_(2);
			sck_low();
			Delay_us_(3);
			sck_high();
			Delay_us_(5);
		}
		read_ack();
		for (i=0;i<8;i++) {		// send address
			gpio_set(11,addr,i);
			Delay_us_(2);
			sck_low();
			Delay_us_(3);
			sck_high();
			Delay_us_(5);
		}
		read_ack();


		return buf;
}

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
	buf[0] = addr;
	buf[1] = data;
	I2C_Master_BufferWrite(I2C2,buf,2,DMA, AD5934_ADDR);

}

uint8_t ad5934_read_data(int addr){
	uint8_t buf[2];
	buf[0] = AD5934_SET_ADDR_PNTR;
	buf[1] = addr;

	I2C_Master_BufferWrite(I2C2,buf,2,DMA, AD5934_ADDR);

	Delay_us_(1000);
	I2C_Master_BufferRead(I2C2, buf, 1,DMA,AD5934_ADDR);
	vTaskDelay(50);
	return buf[0];
}


uint8_t ad5934_read_data_byte(int addr){
	uint8_t buf[2];
	buf[0] = AD5934_SET_ADDR_PNTR;
	buf[1] = addr;

	I2C_Master_BufferWrite(I2C2,buf,2,DMA, AD5934_ADDR);

	Delay_us_(1000);
	I2C_Master_BufferRead(I2C2, buf, 1,DMA,AD5934_ADDR);
	vTaskDelay(50);
	return buf[0];
}

uint8_t mod4int(uint8_t arg){
	if (arg>127) {
		arg = ~(arg) + 1;
	}
	return arg;

}

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

			R1 = ad5934_read_data(AD5934_REAL_DATA_R1);
			R2 = ad5934_read_data(AD5934_REAL_DATA_R2);
			re = (R1 << 8) | R2;
			re &= ~(1 << 15);



			R1  = mod4int(ad5934_read_data(AD5934_IMG_DATA_R1));
			R2  = mod4int(ad5934_read_data(AD5934_IMG_DATA_R2));



			img = (R1 << 8) | R2;

			img &= ~(1 << 15);	// reset signedness flag if present


	//		freq = (((uint16_t)(ad5934_read_data(AD5934_START_FREQ_R2)<<8))&0xFF00) + (((uint16_t)ad5934_read_data(AD5934_START_FREQ_R3))&0x00FF);
	//		mag = sqrt(pow(double(re),2)+pow(double(img),2));
			tmp = (re*re)+(img*img);
			mag = asqrt(tmp);
//			ec += freq;
			vTaskDelay(50);

            if (range_mode==LOW_RANGE_CONDUCTIVITY) {
                ec = mag;
            }
            else if (range_mode==HIGH_RANGE_CONDUCTIVITY) {
                ec = mag;
            }
            else {
                return 0;
            }

            if (mag>15000 && range_mode==LOW_RANGE_CONDUCTIVITY) {
  //          	init_adg715(HIGH_RANGE_CONDUCTIVITY);
            }


            else if (mag<2500 && range_mode==HIGH_RANGE_CONDUCTIVITY) {
  //          	init_adg715(LOW_RANGE_CONDUCTIVITY);
            }

            if (mode==LOW_RANGE_CONDUCTIVITY) {

            }

            if (mode==HIGH_RANGE_CONDUCTIVITY) {

            }
            return ec;

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
