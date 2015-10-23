// This is test sketch for reading the data from EVAL-CN0349-PMDZ evaluation board from Analog devices
// it uses two I2C slave devices, that needs to be configured before usage
// ADG715 and AD5934
 
 
#include "Wire.h"
#define AD5934_ADDR 0x0D
#define ADG715_ADDR 0x48
 
// conductivity measurement ranges setup for ADG715 on EVAL-CN0349-PMDZ. See CN-0349
#define LOW_RANGE_CONDUCTIVITY  0b10000010
#define HIGH_RANGE_CONDUCTIVITY  0b10000001
 
// AD5934 system init registers
#define AD5934_SET_ADRR_PNTR  0xB0
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
 
// io_reader
const float start_freq = 1950; // Set start freq, < 100Khz
const float incre_freq = 975; // Set freq increment
// EOF io_reader
 
int range_mode = LOW_RANGE_CONDUCTIVITY;
 
void setup() {
	Wire.begin();
	Serial.begin(9600);
        init_adg715(LOW_RANGE_CONDUCTIVITY);  // init ADG715 switch for using 1k resistors Rfb and Rcal
        delay(1000);
        Serial.println("init_ad5934");
        print_reg_vals();
 
}
 
void loop(){
  int cmd = 0;
  ad5934_status_req();
  runSweep(range_mode);
  delay(500);
  if (Serial.available() > 0) {
                // read the incoming byte:
                cmd = Serial.read();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(cmd, DEC);
  }
  
 switch(cmd) {
     case 'A':
       init_adg715(130);
     break;
     
     case 'B':
       init_adg715(129);
     break;
   
  }  
     
  cmd = 0;
}
 
 
void print_reg_vals(){
  int i=0;
  int tmpval = 0;
      for (i=0;i<12;i++) { 
          Serial.print((i+0x80),HEX);
          Serial.print(":");
          tmpval = ad5934_read_data(i+0x80);
          Serial.println(tmpval,HEX);
        } 
}
 
void init_ad5934(void){
  // 1. Reset
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
 
 
 
 
void init_adg715(int mode) {
  int data = 0;
  range_mode = mode;
 Wire.beginTransmission(ADG715_ADDR);
 Wire.write(mode);
 Wire.endTransmission();
 delay(100);
 	Wire.requestFrom(ADG715_ADDR,1);
 
	if (Wire.available() >= 1){
		data = Wire.read();
                Serial.print("ADG Setup data: ");
                Serial.println(data);
	}
}

 
byte adg715_req_status(void){
  int data;
  
  /*    Wire.beginTransmission(ADG715_ADDR);
        Wire.write(ADG715_ADDR);
        
	Wire.endTransmission(); */
        Wire.requestFrom(ADG715_ADDR,1);
 
	if (Wire.available() >= 1){
		data = Wire.read();
                Serial.print("CurADG715: ");
                Serial.println(data);
	}
        else {
          Serial.println("failed to read ADG715 back");
        }
        return data; 
}


void ad5934_write_data(int addr, int data){
 Wire.beginTransmission(AD5934_ADDR);
 Wire.write(addr);
 Wire.write(data);
 Wire.endTransmission();
 delay(1);
}
 
 
int ad5934_read_data(int addr){
	int data;
 
	Wire.beginTransmission(AD5934_ADDR);
	Wire.write(AD5934_SET_ADRR_PNTR);
	Wire.write(addr);
	Wire.endTransmission();
 
	delay(1);
 
	Wire.requestFrom(AD5934_ADDR,1);
 
	if (Wire.available() >= 1){
		data = Wire.read();
	}
	else {
		data = -1;
	}
 
	delay(1);
	return data;	
}
 
void ad5934_status_req(void){
  	int data;
 
	Wire.beginTransmission(AD5934_ADDR);
	Wire.write(AD5934_SET_ADRR_PNTR);
	Wire.write(AD5934_STATUS_REG);
	Wire.endTransmission();
 
	delay(1);
 
	Wire.requestFrom(AD5934_ADDR,1);
 
	if (Wire.available() >= 1){
	      data = Wire.read();
              Serial.print("Data");
              Serial.print(data, DEC);
              Serial.println(";");
	}
	else {
		data = -1;
	}
	delay(1); 
}
 
 
double runSweep(int mode) {
        double ec = 0;
	short re = 0;
	short img = 0;
	double freq = 0;
	double mag = 0;
	double phase = 0;
	double gain = 0;
	double impedance = 0;
	int i=0;
 
 adg715_req_status();
 
	// 1. Standby '10110000' Mask D8-10 of avoid tampering with gains
	ad5934_write_data(AD5934_CTRL_R1,(ad5934_read_data(AD5934_CTRL_R1) & 0x07) | 0xB0);
 
	// 2. Initialize sweep
	ad5934_write_data(AD5934_CTRL_R1,(ad5934_read_data(AD5934_CTRL_R1) & 0x07) | 0x10);
 
	// 3. Start sweep
	ad5934_write_data(AD5934_CTRL_R1,(ad5934_read_data(AD5934_CTRL_R1) & 0x07) | 0x20);	
 
 
	while((ad5934_read_data(AD5934_STATUS_REG) & 0x07) < 4 ) {  // Check that status reg != 4, sweep not complete
		delay(25); // delay between measurements
 
		int flag = ad5934_read_data(AD5934_STATUS_REG)& 2;
 
 
		if (flag==2) {
 
			byte R1 = ad5934_read_data(AD5934_REAL_DATA_R1);
			byte R2 = ad5934_read_data(AD5934_REAL_DATA_R2);
			re = (R1 << 8) | R2;

			Serial.print("RealR1: ");
			Serial.println(R1);

			Serial.print("RealR2: ");
			Serial.println(R2);

			R1  = ad5934_read_data(AD5934_IMG_DATA_R1);
 			Serial.print("ImgR1: ");
			Serial.println(R1);

			R2  = ad5934_read_data(AD5934_IMG_DATA_R2);
			img = (R1 << 8) | R2;


			Serial.print("ImgR2: ");
			Serial.println(R2);
 
			freq = start_freq + i*incre_freq;
			mag = sqrt(pow(double(re),2)+pow(double(img),2));
 

			Serial.print("Freq: ");
			Serial.print(freq/1000);
			Serial.print(",kHz;");
 
			Serial.print(" Conductivity: ");
			Serial.print(mag);
			Serial.print(",RU;");
 
			Serial.print(" real: ");
			Serial.print(re);
			Serial.print(",");
 
			Serial.print(" imaginary: ");
			Serial.print(img);
			Serial.println(",");           
 
                        if (range_mode==LOW_RANGE_CONDUCTIVITY) {
                          ec = mag/42;
                        }
                        else if (range_mode==HIGH_RANGE_CONDUCTIVITY) {
                          ec = mag/4.2;
                        }
                        else {
                          return 0;
                        }
 
                        if (ec>450 && range_mode==LOW_RANGE_CONDUCTIVITY) {
 
                          Serial.println(" SWITHING RANGE: LOW->HIGH ");
                          init_adg715(HIGH_RANGE_CONDUCTIVITY);
                        }
 
                        else if (ec<450 && range_mode==HIGH_RANGE_CONDUCTIVITY) {
 
                          Serial.println(" SWITHING RANGE: HIGH->LOW ");
                          init_adg715(LOW_RANGE_CONDUCTIVITY);
                        }
                        if (mode==LOW_RANGE_CONDUCTIVITY) {
                          Serial.print(" LOW RANGE >>: ");
                        }
 
                        if (mode==HIGH_RANGE_CONDUCTIVITY) {
                          Serial.print(" HIGH RANGE >>: ");
                        }
                        Serial.print(" EC: ");
			Serial.print(ec);
			Serial.println(" uS");
 
                        return ec;
 
		}
	}
	ad5934_write_data(AD5934_CTRL_R1,(ad5934_read_data(AD5934_CTRL_R1) & 0x07) | 0xA0);
}
