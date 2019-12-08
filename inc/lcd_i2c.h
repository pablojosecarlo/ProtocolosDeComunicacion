//
//  File lcd_i2c.h
//  LCD library with PCF8574 I2C adapter.
//
//  Refurbished para usarla en c con la CIAA
//    c++      ---> c
//    wire.lib ---> No usa wire.lib
//    Arduino  ---> CIAA
//
//  Basado en la siguiente librería:
//
//  file LiquidCrystal_PCF8574.h
//  brief LiquidCrystal library with PCF8574 I2C adapter.
//
//  author Matthias Hertel, http://www.mathertel.de
//  copyright Copyright (c) 2014 by Matthias Hertel.\n
//  This work is licensed under a BSD style license.\n
//  See http://www.mathertel.de/License.aspx
//
// details
// This is a library for driving LiquidCrystal displays (LCD)
// by using the I2C bus and an PCF8574 I2C adapter.
//
// * 19.10.2013 created.
// * 24.05.2015 Arduino Library Manager compatible.

/*=====[Evitar inclusion multiple comienzo]==================================*/

#ifndef _LCD_I2C_PCF8574_H_
#define _LCD_I2C_PCF8574_H_

/*=====[Inclusiones de dependencias de funciones publicas]===================*/

#include "sapi.h"
/*=====[C++ comienzo]========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=======================[Macros de definicion de constantes publicas]=========================*/

/*=======================[Macros estilo funcion publicas]======================================*/

/*=======================[Definiciones de tipos de datos publicos]=============================*/

/*=======================[Tipo de datos enumerado y otros]=====================================*/

/*=======================[Prototipos de funciones del I2C]====================================*/

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

//Aqui guardo las posición en el LCD
typedef struct {
	uint8_t linea;
	uint8_t columna;
} posicion_t;

posicion_t posc;

  void setup_lcd(uint8_t adr);

  void begin(uint8_t cols, uint8_t lines, uint8_t charsize);

  void clear();
  void home();

  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void leftToRight();
  void rightToLeft();
  void autoscroll();
  void noAutoscroll();

  void setCursor( uint8_t, uint8_t );

  void write_lcd(uint8_t);
  void print_lcd(uint8_t *, uint8_t);

  //agregadas
  void clearLinea( uint8_t );
  void writeLinea( uint8_t, uint8_t *, uint8_t);

  // low level functions
  void _command(uint8_t);
  void _send(uint8_t value, uint8_t mode);
  void _sendNibble(uint8_t halfByte, uint8_t mode);
  void _write2Wire(uint8_t halfByte, uint8_t mode, uint8_t enable);

// NEW:
  uint8_t _Addr;        ///< Wire Address of the LCD
  uint8_t _backlight;   ///< the backlight intensity 

  uint8_t _displayfunction; ///< lines and dots mode
  uint8_t _displaycontrol;  ///< cursor, display, blink flags
  uint8_t _displaymode;     ///< left2right, autoscroll

  uint8_t _numLines;        ///< The number of rows the display supports.

 // Mas NEW todavía
  uint8_t _numCols;         ///< El número de columnas que soporta el Display.



/*=====[C++ fin]=============================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Evitar inclusion multiple fin]=======================================*/

#endif /* _LCD_I2C_PCF8574_H_ */
