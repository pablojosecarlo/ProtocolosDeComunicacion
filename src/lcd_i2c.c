//
// file lcd_i2c.c
// LCD library with PCF8574 I2C adapter.
//
// Refurbished para usarla en c con la CIAA
//    c++      ---> c
//    wire.lib ---> No usa wire.lib
//    Arduino  ---> CIAA
//
// Basado en la siguiente librería:

// file LiquidCrystal_PCF8574.cpp
// brief LiquidCrystal (LCD) library with PCF8574 I2C adapter.
//
// Original author Matthias Hertel, http://www.mathertel.de
// copyright Copyright (c) 2014 by Matthias Hertel.\n
// This work is licensed under a BSD style license.\n
// See http://www.mathertel.de/License.aspx
//
// details
// This is a library for driving LiquidCrystal displays (LCD) by using the I2C bus and an PCF8574 I2C adapter.
// This library is derived from the original Arduino LiquidCrystal library and uses the original Wire library for communication.
//
// More documentation and source code is available at http://www.mathertel.de/Arduino
//
// Definitions on how the PCF8574 is connected to the LCD

#include "lcd_i2c.h"
#include "sapi.h"               /* <= sAPI header       */
#include "sapi_i2c.h"           /* <= sAPI I2C header   */
#include "sapi_delay.h"         /* <= sAPI Delay header */

// These are Bit-Masks for the special signals and background light
#define PCF_RS  0x01
#define PCF_RW  0x02
#define PCF_EN  0x04
#define PCF_BACKLIGHT 0x08

// Definitions on how the PCF8574 is connected to the LCD
// These are Bit-Masks for the special signals and Background
#define RSMODE_CMD  0
#define RSMODE_DATA 1

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
// El ex-creador de la clase . . . .

void setup_lcd(uint8_t addr)
{
  _Addr = addr;
  _backlight = 0;
}


void begin(uint8_t lines, uint8_t cols, uint8_t dotsize) {
  // cols ignored !
  _numLines = lines;
  _numCols = cols;

  _displayfunction = 0;

  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != 0) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  //Wire.begin();

  // initializing the display
  _write2Wire(0x00, LOW, FALSE);
  //delayMicroseconds(50000);
  delay(50); //en mSeg

  // put the LCD into 4 bit mode according to the 
  // hitachi HD44780 datasheet figure 26, pg 47
  _sendNibble(0x03, RSMODE_CMD);
  //delayMicroseconds(4500);
  delay(5);
  _sendNibble(0x03, RSMODE_CMD);
  //delayMicroseconds(4500);
  delay(5);
  _sendNibble(0x03, RSMODE_CMD);
  //delayMicroseconds(150);
  delay(5);
  // finally, set to 4-bit interface
  _sendNibble(0x02, RSMODE_CMD);

  // finally, set # lines, font size, etc.
  _command(LCD_FUNCTIONSET | _displayfunction);  

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  _command(LCD_ENTRYMODESET | _displaymode);

}

/********** high level commands, for the user! */
void clear()
{
  _command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  //delayMicroseconds(2000);  // this command takes a long time!
  delay(2);

  clearLinea ( 0 );
  clearLinea ( 1 );
  clearLinea ( 2 );
  clearLinea ( 3 );

  setCursor( 0, 0 );
}

void home()
{
  _command(LCD_RETURNHOME);  // set cursor position to zero
  //delayMicroseconds(2000);  // this command takes a long time!
  delay(2);
  posc.linea = 0;
  posc.columna = 0;
}


// Turn the display on/off (quickly)
void noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  _command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void display() {
  _displaycontrol |= LCD_DISPLAYON;
  _command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  _command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void cursor() {
  _displaycontrol |= LCD_CURSORON;
  _command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  _command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void blink() {
  _displaycontrol |= LCD_BLINKON;
  _command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void scrollDisplayLeft(void) {
  _command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void scrollDisplayRight(void) {
  _command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  _command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  _command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  _command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  _command(LCD_ENTRYMODESET | _displaymode);
}


/// Setting the brightness of the background display light.
/// The backlight can be switched on and off.
/// The current brightness is stored in the private _backlight variable 
/// to have it available for further data transfers.
void setBacklight(uint8_t brightness) {
  _backlight = brightness;
  // send no data but set the background-pin right;
  _write2Wire(0x00, RSMODE_DATA, FALSE);
} // setBacklight

//Se la llama para tener un registro de dónde esta el cursor
void _incrementarPosc(){
	posc.columna++;
	if ( posc.columna >= _numCols ){
		posc.columna = 0;
		posc.linea++;
		if ( posc.linea >= _numLines ){
			posc.linea = 0;
		}
	}
}

// Set the cursor to a new position.
void setCursor(uint8_t linea, uint8_t columna)
{
  //Corrijo todo:
	uint8_t col; uint8_t row;
	if ( columna >= _numCols ) columna = _numCols - 1;
	if ( columna < 0  ) columna = 0;
	if ( linea >= _numLines ) linea = _numLines - 1;
	if ( linea < 0 ) linea = 0;

    posc.linea = linea;
	posc.columna = columna;

  //Ahora sigue el código original

  int linea_offsets[] = { 0x00, 0x40, 0x14, 0x54 }; //{ 0, 40, 20, 60}

  _command(LCD_SETDDRAMADDR | (columna + linea_offsets[linea]));
}


//write retornaba 1 pero no tiene sentido porque no se interroga al LCD
//corrige automaticamente los saltos de lineas
void write_lcd(uint8_t value) {

	_send(value, RSMODE_DATA); //envío al LCD, esto incrementa la posición del cursor pero MAL

    _incrementarPosc();         //Incremento BIEN la Posc
    if ( posc.columna == 0 ) setCursor(posc.linea, posc.columna); //solo si se paso de renglón seteo el cursor
}

//El reemplazo al viejo print
void print_lcd(uint8_t * texto, uint8_t largo ){
	for ( uint8_t i = 0; i < largo; i++ ) {
		write_lcd( texto[i] );
	}
}

//borra la linea indicada
void clearLinea( uint8_t linea ){
	setCursor(linea, 0 );
	for( int i = 0; i < _numCols; i++){
		write_lcd( ' ' );
	}
	setCursor(linea, 0 ); //vuelvo a dejar al cursor al principio de la linea
}

//Escribe sobre la linea indicada
void writeLinea( uint8_t linea, uint8_t * texto, uint8_t largo ){
	clearLinea( linea );
	if ( largo > 20 ) largo= 20;		//solo 20 de máximo
	for( int i = 0; i < largo; i++ ){
		write_lcd( texto[ i ] );
	}
}

/* ----- low level functions ----- */

inline void _command(uint8_t value) {
  _send(value, RSMODE_CMD);
} // _command()


// write either command or data
void _send(uint8_t value, uint8_t mode) {
  // separate the 4 value-nibbles
  uint8_t valueLo = value    & 0x0F;
  uint8_t valueHi = value>>4 & 0x0F;

  _sendNibble(valueHi, mode);
  _sendNibble(valueLo, mode);
} // _send()


// write a nibble / halfByte with handshake
void _sendNibble(uint8_t halfByte, uint8_t mode) {
  _write2Wire(halfByte, mode, TRUE);
  //delayMicroseconds(1);    // enable pulse must be >450ns
  delay(1);
  _write2Wire(halfByte, mode, FALSE);
  //delayMicroseconds(37);   // commands need > 37us to settle
  delay(1);
} // _sendNibble


// private function to change the PCF8674 pins to the given value
void _write2Wire(uint8_t halfByte, uint8_t mode, uint8_t enable) {
  // map the given values to the hardware of the I2C schema
  uint8_t i2cData = halfByte << 4;
  if (mode > 0) i2cData |= PCF_RS;
  // PCF_RW is never used.
  if (enable > 0) i2cData |= PCF_EN;
  if (_backlight > 0) i2cData |= PCF_BACKLIGHT;


  i2cWrite(I2C0, _Addr, &i2cData, 1, TRUE);

} // write2Wire


// The End.
