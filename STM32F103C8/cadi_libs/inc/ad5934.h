#define AD5934_ADDR (0x0D<<1)		// needs to be <<1 for use in 7bit mode
#define ADG715_ADDR (0x48<<1)

// conductivity measurement ranges setup for ADG715 on EVAL-CN0349-PMDZ. See CN-0349

#define LOW_RANGE_CONDUCTIVITY  0b10000010
//#define LOW_RANGE_CONDUCTIVITY  0b01000001
#define HIGH_RANGE_CONDUCTIVITY  0b10000001

// AD5934 system init registers
#define AD5934_SET_ADDR_PNTR  0xB0
#define AD5934_STATUS_REG 0x8F

// 2. Program start frequency (1950Hz = 0x00@0x82, 0xF3@0x83, 0xC5@0x84)
#define AD5934_START_FREQ_R1 0x82
#define AD5934_START_FREQ_R2 0x83
#define AD5934_START_FREQ_R3 0x84

// 3. Program the Frequency increment (for 975Hz increment set 0x0079E2: 0x00@0x85, 0x79@0x86, 0xE2@0x87)
#define AD5934_FREG_INC_R1 0x85
#define AD5934_FREG_INC_R2 0x86
#define AD5934_FREG_INC_R3 0x87

// 4. Program the number of increments (10 increments: 0x00@0x88, 0x0A@0x89)
#define AD5934_NUM_INC_R1 0x88
#define AD5934_NUM_INC_R2 0x89

// 5. Program the delay in the measurements. The worst case is at maximum frequency
// 1950 + (975x10) = 11700 Hz. D = 1ms x 11700 = 12sec. 0x00@0x8A, 0x0C@0x8B
#define AD5934_MEAS_DELAY_R1 0x8A
#define AD5934_MEAS_DELAY_R2 0x8B

// 6. Initialize the system, writing 0x11 to Register Address 0x80,
// wait several milliseconds

// Real part of measurement
#define AD5934_REAL_DATA_R1 0x94
#define AD5934_REAL_DATA_R2 0x95

// and Imaginary
#define AD5934_IMG_DATA_R1 0x96
#define AD5934_IMG_DATA_R2 0x97

#define AD5934_CTRL_R1 0x80
#define AD5934_CTRL_R2 0x81

#define EVAL0349_I2CX	I2C2	// I2C bus number for EC sensor



/**
 * struct ad5933_platform_data - platform specific data
 * @ext_clk_Hz:		the external clock frequency in Hz, if not set
 *			the driver uses the internal clock (16.776 MHz)
 * @vref_mv:		the external reference voltage in millivolt
 */

void init_ad5934(void);
void init_adg715(uint8_t mode);
uint8_t mod4int(uint8_t arg);
uint16_t asqrt(uint32_t x);
