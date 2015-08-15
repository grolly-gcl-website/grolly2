#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "delay.h"
#include "LiquidCrystal_I2C.h"
#include "I2CRoutines.h"


uint8_t Buffer_Tx1[7] = {0x5, 0x6,0x8,0xA, 0x6,0x8,0xA};
uint8_t Buffer_Tx2[7] = {0x5, 0x6,0x8,0xA, 0x6,0x8,0xA};



uint8_t Buffer_Rx1[7] = {0x5, 0x6,0x8,0xA, 0x6,0x8,0xA};
uint8_t Buffer_Rx2[7] = {0x5, 0x6,0x8,0xA, 0x6,0x8,0xA};


uint8_t lcd_bl = 0;				// lcd backlight flag




//YWROBOT
//last updated on 21/12/2011
//Tim Starling Fix the reset bug (Thanks Tim)
//wiki doc http://www.dfrobot.com/wiki/index.php?title=I2C/TWI_LCD1602_Module_(SKU:_DFR0063)
//Support Forum: http://www.dfrobot.com/forum/
//Compatible with the Arduino IDE 1.0
//Library version:1.1


void LCDI2C_write(uint8_t value){
	LCDI2C_send(value, Rs);
}



// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

volatile static LiquidCrystal_I2C_Def lcdi2c;

void LCDI2C_init(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows)
{
  lcdi2c.Addr = lcd_Addr;
  lcdi2c.cols = lcd_cols;
  lcdi2c.rows = lcd_rows;
  lcdi2c.backlightval = LCD_BACKLIGHT;

//  init_I2C1(); // Wire.begin();
  lcdi2c.displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  LCDI2C_begin(lcd_cols, lcd_rows);
}

void LCDI2C_simple_send(uint8_t val, uint8_t mode){
	Buffer_Tx1[0] = 0;

#ifdef LCD_I2C_DFROBOT
	Buffer_Tx1[0] = lcd_bl | mode | En | ((val<<4) & 0xF0);
#endif

#ifdef USE_LCD
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1[0],1,Interrupt, LCD_I2C_ADDR);

	Delay_us(100);
#endif


#ifdef LCD_I2C_DFROBOT
	Buffer_Tx1[0] =  lcd_bl | mode | ((val<<4) & 0xF0);
#endif
#ifdef USE_LCD
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1[0],1,Interrupt, LCD_I2C_ADDR);
	Delay_us(100);
#endif
}

// mode - command (0) or data (1)
void LCDI2C_send(uint8_t val, uint8_t mode)
{
	Buffer_Tx1[0] = 0;

#ifdef USE_LCD
	Delay_us(7);
	Buffer_Tx1[0] = lcd_bl | En | mode |(val & 0xF0);
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1[0],1,Interrupt, LCD_I2C_ADDR);
	Delay_us(2);

	// e_0;
	Buffer_Tx1[0] = lcd_bl |mode |(val & 0xF0);
		I2C_Master_BufferWrite(I2C1, Buffer_Tx1[0],1,Interrupt, LCD_I2C_ADDR);
	Delay_us(2);

	Buffer_Tx1[0] = lcd_bl |En| mode |((val<<4) & 0xF0);
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1[0],1,Interrupt, LCD_I2C_ADDR);
	Delay_us(2);

	// e_0;
	Buffer_Tx1[0] =  lcd_bl |mode |((val<<4) & 0xF0);
		I2C_Master_BufferWrite(I2C1, Buffer_Tx1[0],1,Interrupt, LCD_I2C_ADDR);
	Delay_us(10);
#endif
}

void LCDI2C_begin(uint8_t cols, uint8_t lines) {//, uint8_t dotsize) {
	if (lines > 1) {
		lcdi2c.displayfunction |= LCD_2LINE;
	}
	lcdi2c.numlines = lines;

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
	Delay(50);

	/*
	  e_1;rs_0;rw_0;
	      Delay_us(100);	// assume 10ms
	      set4lowBits(0b0010);	// set 4 bit bus
	      e_0;
	      Delay_us(10);	// assume 10ms

	      Lcd_write_cmd(0b00101000);	// again, 4bit bus and the rest 4bits of whole command will get the destination now
	      Delay_us(10);
	  	  Lcd_write_cmd(Display_clear);
		  Lcd_write_cmd(0b00000110);	// function set
		  Lcd_write_cmd(0b00001100);	// display on cursor off
		  Lcd_write_cmd(Display_clear);	// function set
		  Lcd_write_str("12345678");
	  	Delay_us(10);
	  	*/


	// Now we pull both RS and R/W low to begin commands
//	LCDI2C_expanderWrite(lcdi2c.backlightval);	// reset expander and turn backlight off (Bit 8 =1)
	Delay(50);
//	Buffer_Tx1[0] = 0;

#ifdef USE_LCD
	Buffer_Tx1[0] = 0;
		Buffer_Tx1[0] = LCD_BACKLIGHT | En;
		I2C_Master_BufferWrite(I2C1, Buffer_Tx1[0],1,Interrupt, LCD_I2C_ADDR);
	  	//put the LCD into 4 bit mode
		// this is according to the hitachi HD44780 datasheet
		// figure 24, pg 46
		lcd_bl = LCD_BACKLIGHT;
		// next display function set 4 bit
		LCDI2C_simple_send(0x02,0);
		Delay_us(50);	// stable

		// next display function set 4 bit
		LCDI2C_simple_send(0x02,0);
		Delay_us(50);	// stable

		// next display function set 4 bit
		LCDI2C_simple_send(0x08,0);	// function set 2 lines, 5x8
		Delay_us(50);
		LCDI2C_simple_send(0x00,0);	// blinking cursor
		LCDI2C_simple_send(0x0C,0);	// blinking cursor
		Delay_us(50);
	//	LCDI2C_send(90, Rs);
		Delay_us(50);	// stable 24
	//	LCDI2C_command(0x20 | 0x08);
#endif





	// and one more setting for blinking cursor


	/*
		      Delay_us(10);
		  	  Lcd_write_cmd(Display_clear);
			  Lcd_write_cmd(0b00000110);	// function set
			  Lcd_write_cmd(0b00001100);	// display on cursor off
			  Lcd_write_cmd(Display_clear);	// function set
			  Lcd_write_str("12345678");
		  	Delay_us(10);

	*/
//	LCDI2C_command(0b00101000);
/*	Delay_us(10);
	LCDI2C_command(0x20 | 0x08);
	Delay_us(4000);
	LCDI2C_command(0x20 | 0x08);
	Delay_us(200);
	LCDI2C_command(0x20 | 0x08);
	Delay_us(50); */

/*	LCDI2C_command(0b00000001);
	LCDI2C_command(0b00000110);
	LCDI2C_command(0b00001111);
	LCDI2C_command(0b00000001);
*/
/*	// set # lines, font size, etc.
	LCDI2C_command(LCD_FUNCTIONSET | lcdi2c.displayfunction);

	// turn the display on with no cursor or blinking default
	lcdi2c.displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	LCDI2C_display();

	// clear it off
	LCDI2C_clear();

	// Initialize to default text direction (for roman languages)
	lcdi2c.displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	// set the entry mode
	LCDI2C_command(LCD_ENTRYMODESET | lcdi2c.displaymode);

	LCDI2C_home(); */

}

/********** high level commands, for the user! */
void LCDI2C_clear(){
	LCDI2C_command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	DelayMC(3000);  // this command takes a long time!
}

void LCDI2C_home(){
	LCDI2C_command(LCD_RETURNHOME);  // set cursor position to zero
	DelayMC(3000);  // this command takes a long time!
}

void LCDI2C_setCursor(uint8_t col, uint8_t row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > lcdi2c.numlines ) {
		row = lcdi2c.numlines-1;    // we count rows starting w/0
	}
	LCDI2C_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void LCDI2C_noDisplay() {
	lcdi2c.displaycontrol &= ~LCD_DISPLAYON;
	LCDI2C_command(LCD_DISPLAYCONTROL | lcdi2c.displaycontrol);
}

void LCDI2C_display() {
	lcdi2c.displaycontrol |= LCD_DISPLAYON;
	LCDI2C_command(LCD_DISPLAYCONTROL | lcdi2c.displaycontrol);
}

// Turns the underline cursor on/off
void LCDI2C_noCursor() {
	lcdi2c.displaycontrol &= ~LCD_CURSORON;
	LCDI2C_command(LCD_DISPLAYCONTROL | lcdi2c.displaycontrol);
}
void LCDI2C_cursor() {
	lcdi2c.displaycontrol |= LCD_CURSORON;
	LCDI2C_command(LCD_DISPLAYCONTROL | lcdi2c.displaycontrol);
}

// Turn on and off the blinking cursor
void LCDI2C_noBlink() {
	lcdi2c.displaycontrol &= ~LCD_BLINKON;
	LCDI2C_command(LCD_DISPLAYCONTROL | lcdi2c.displaycontrol);
}

void LCDI2C_blink() {
	lcdi2c.displaycontrol |= LCD_BLINKON;
	LCDI2C_command(LCD_DISPLAYCONTROL | lcdi2c.displaycontrol);
}

// These commands scroll the display without changing the RAM
void LCDI2C_scrollDisplayLeft(void) {
	LCDI2C_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LCDI2C_scrollDisplayRight(void) {
	LCDI2C_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LCDI2C_leftToRight(void) {
	lcdi2c.displaymode |= LCD_ENTRYLEFT;
	LCDI2C_command(LCD_ENTRYMODESET | lcdi2c.displaymode);
}

// This is for text that flows Right to Left
void LCDI2C_rightToLeft(void) {
	lcdi2c.displaymode &= ~LCD_ENTRYLEFT;
	LCDI2C_command(LCD_ENTRYMODESET | lcdi2c.displaymode);
}

// This will 'right justify' text from the cursor
void LCDI2C_autoscroll(void) {
	lcdi2c.displaymode |= LCD_ENTRYSHIFTINCREMENT;
	LCDI2C_command(LCD_ENTRYMODESET | lcdi2c.displaymode);
}

// This will 'left justify' text from the cursor
void LCDI2C_noAutoscroll(void) {
	lcdi2c.displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	LCDI2C_command(LCD_ENTRYMODESET | lcdi2c.displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LCDI2C_createChar(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	LCDI2C_command(LCD_SETCGRAMADDR | (location << 3));
	int i;
	for (i=0; i<8; i++) {
		LCDI2C_write(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void LCDI2C_noBacklight(void) {
	lcdi2c.backlightval=LCD_NOBACKLIGHT;
	LCDI2C_expanderWrite(LCD_NOBACKLIGHT);
}

void LCDI2C_backlight(void) {
	lcdi2c.backlightval=LCD_BACKLIGHT;
	LCDI2C_expanderWrite(LCD_BACKLIGHT);
}



/*********** mid level commands, for sending data/cmds */

void LCDI2C_command(uint8_t value) {
	LCDI2C_send(value, 0);
}


/************ low level data pushing commands **********/

// write either command or data
/* void LCDI2C_send(uint8_t value, uint8_t mode) {
	uint8_t highnib=value&0xf0;
	uint8_t lownib=(value<<4)&0xf0;
       LCDI2C_write4bits((highnib)|mode);
	LCDI2C_write4bits((lownib)|mode);
} */

void LCDI2C_write4bits(uint8_t value) {
	LCDI2C_expanderWrite(value);
	LCDI2C_pulseEnable(value);
}

volatile static uint8_t expandertmp[1] = {0x5};

void LCDI2C_expanderWrite(uint8_t _data){
#ifdef USE_LCD
	//uint8_t expandertmp[0];
	// uint8_t _revdata = _data&(1<<0)*8 + _data&(1<<1)*4 + _data&(1<<2)*2 + _data&(1<<3)*1;
	Buffer_Tx1[0] = _data | lcdi2c.backlightval;
//	I2C_StartTransmission (I2C1, I2C_Direction_Transmitter, lcdi2c.Addr); //Wire.beginTransmission(_Addr);
//	I2C_WriteData(I2C1, (int)(_data) | lcdi2c.backlightval);  //printIIC((int)(_data) | _backlightval);
//	I2C_GenerateSTOP(I2C1, ENABLE); //Wire.endTransmission();
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1[0],1,Interrupt, LCD_I2C_ADDR);
#endif
}


/*
 *
 * 		Delay_us(6);	// stable 240
		rs_0;
		rw_0;
		e_1;
		Delay_us(1);
		set4highBits(cmd);
		e_0;
		Delay_us(1);
		e_1;
		Delay_us(1);
		set4lowBits(cmd);
		e_0;
		Delay_us(6);
 */
void LCDI2C_pulseEnable(uint8_t _data){
	LCDI2C_expanderWrite(_data | En);	// En high
	DelayMC(1);		// enable pulse must be >450ns

	LCDI2C_expanderWrite(_data & ~En);	// En low
	DelayMC(50);		// commands need > 37us to settle
/*	LCDI2C_expanderWrite(_data & ~En);	// En low
	DelayMC(1);		// enable pulse must be >450ns

	LCDI2C_expanderWrite(_data | En);	// En high
	DelayMC(50);		// commands need > 37us to settle */
}


// Alias functions

void LCDI2C_cursor_on(){
	LCDI2C_cursor();
}

void LCDI2C_cursor_off(){
	LCDI2C_noCursor();
}

void LCDI2C_blink_on(){
	LCDI2C_blink();
}

void LCDI2C_blink_off(){
	LCDI2C_noBlink();
}

void LCDI2C_load_custom_character(uint8_t char_num, uint8_t *rows){
		LCDI2C_createChar(char_num, rows);
}

void LCDI2C_setBacklight(uint8_t new_val){
	if(new_val){
		backlight();		// turn backlight on
	}else{
		noBacklight();		// turn backlight off
	}
}

//Ã�Â¤Ã‘Æ’Ã�Â½Ã�ÂºÃ‘â€ Ã�Â¸Ã‘ï¿½ Ã�Â¿Ã�ÂµÃ‘â‚¬Ã�ÂµÃ�Â´Ã�Â°Ã‘â€¡Ã�Â¸ Ã‘ï¿½Ã‘â€šÃ‘â‚¬Ã�Â¾Ã�ÂºÃ�Â¸ Ã‘â€¡Ã�ÂµÃ‘â‚¬Ã�ÂµÃ�Â· USART
void LCDI2C_write_String(char* str) {
  uint8_t i=0;
  while(str[i])
  {
    LCDI2C_write(str[i]);
    i++;
  }
}

void LCDI2C_write_digit(uint8_t val) {
  uint8_t i=0;
  i = (val%100)/10;
    LCDI2C_write(i);
  i = val%10;
  LCDI2C_write(i);
}


