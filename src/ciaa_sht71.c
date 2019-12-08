/*
 * Copyright 2019, Pablo JC Alonso Castillo.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Date: Dec 2, 2019 */

/*==================[inclusions]=============================================*/

#include "sht71.h"
#include "lcd_i2c.h"
#include "sapi.h"               // <= sAPI header
#include "sapi_i2c.h"           // <= sAPI I2C header
#include "sapi_delay.h"         // <= sAPI Delay header
#include <string.h>
/*==================[macros and definitions]=================================*/
#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232
/*==================[internal data declaration]==============================*/
/*==================[internal functions declaration]=========================*/
/*==================[internal data definition]===============================*/
/*==================[external data definition]===============================*/
extern uint8_t LAST_STATUS_REG_SET;
extern float humedad;
extern float temperatura;
extern float tempDewPoint;
extern uint8_t REG_STATUS_REGISTER;
extern uint8_t TxtBLE[7];
extern uint8_t TxtSTS[24];
/*==================[internal functions definition]==========================*/
/*==================[external functions definition]==========================*/

bool_t hm10bleTest( int32_t uart );
void myTickHook( void * );

/*==================[Main function]==========================================*/

/*
 * Nota:
 * No Eric. No voy a poner una máquina de estados
 * y delays no bloqueantes para este demo.
 * Ya bastantes problemas me dió el SHT71 y el p. BLE HM-10. . . .
 *
 */

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){
   /* ------------- INICIALIZACIONES ------------- */

   boardConfig();

   tickConfig( 50 );

   //************************************
   // Inicializar UART_USB para conectar a la PC
   uartConfig( UART_PC, 9600 );
   uartWriteString( UART_PC, "UART_PC configurada.\r\n" );

   // Inicializar UART_232 para conectar al modulo bluetooth
   uartConfig( UART_BLUETOOTH, 9600 );
   uartWriteString( UART_PC, "UART_BLUETOOTH para modulo Bluetooth configurada.\r\n" );

   uint8_t data = 0;

   uartWriteString( UART_PC, "Testeto si el modulo esta conectado enviando: AT\r\n" );
   if( hm10bleTest( UART_BLUETOOTH ) ){
      uartWriteString( UART_PC, "Modulo conectado correctamente.\r\n" );
   }
   else{
      uartWriteString( UART_PC, "No funciona.\r\n" );
   }

   //************************************

   // Inicializando el i2c
   i2cInit(I2C0, 100000);

   // Inicialización del LCD_I2C
   setup_lcd( 0x27 );
   begin( 4, 20, LCD_5x8DOTS );
   noBlink();
   noCursor();
   home();
   setCursor(0,0);
   print_lcd( "    CIAA - SHT71", 16 );

   // Inicializando el SHT71
   // STSREG_F_N_12_14	-> HEATER=OFF, OPT=NO RELOAD, 12bit RH, 14bit TEMP
   SHT71_SET_STATUS( STSREG_F_N_12_14 );

   //llmada al la lectura y envio de datos al lcd y BLE
   tickCallbackSet( myTickHook, 0 );

   /* ------------- REPETIR POR SIEMPRE ------------- */

   while(1){

	  // Si leo un dato de una UART lo envio a la otra (bridge)
	  if( uartReadByte( UART_PC, &data ) ) {
		 uartWriteByte( UART_BLUETOOTH, data );
	  }
	  if( uartReadByte( UART_BLUETOOTH, &data ) ) {
		 if( data == '0' ) {
			//apagar calefactor
			gpioWrite( LEDB, OFF );
			LAST_STATUS_REG_SET &= ~SHT71_HEAT_ON;
			SHT71_SET_STATUS( LAST_STATUS_REG_SET );
		 }
	     if( data == '1' ) {
			//encender calefactor
			gpioWrite( LEDB, ON );
			LAST_STATUS_REG_SET |= SHT71_HEAT_ON;
			SHT71_SET_STATUS( LAST_STATUS_REG_SET );
		 }
		 if( data == '2' ) {
			//poner en baja resolución
			gpioWrite( LED1, ON );
			LAST_STATUS_REG_SET |= SHT71_LOW_RES;
			SHT71_SET_STATUS( LAST_STATUS_REG_SET );
		 }
		 if( data == '3' ) {
			//poner en alta resolución
			gpioWrite( LED1, OFF );
			LAST_STATUS_REG_SET &= ~SHT71_LOW_RES;
			SHT71_SET_STATUS( LAST_STATUS_REG_SET );
		 }

		 uartWriteByte( UART_PC, data );
	  }
   }

	   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
	      por ningun S.O. */
	   return 0 ;
}

/*==================[definiciones de funciones externas]=====================*/

bool_t hm10bleTest( int32_t uart )
{
   uartWriteString( uart, "AT\r\n" );
   return waitForReceiveStringOrTimeoutBlocking( uart,
                                                 "OK\r\n", strlen("OK\r\n"),
                                                 1000 );
}

void myTickHook( void *ptr )
{
	static uint16_t cont = 0;
	cont++;

	if( cont == 20 ){ //cada 1 segundo
		cont = 0;

	   //ponemos la temperatura en el lcd
	   temperatura = SHT71_READ_TEMPERATURA();

	   setCursor(1,1);
	   print_lcd( "T=", 2 );
	   FLOAT_A_LCD_BLE( temperatura );
	   write_lcd( 'C' );

	   //y en el bluetooth
	   TxtBLE[1] = 'T'; TxtBLE[2] = '=';
	   uartWriteString( UART_BLUETOOTH, TxtBLE );

	   //ponemos la humedad en el lcd
	   humedad = SHT71_READ_HUMEDAD();

	   setCursor(1,11);
	   print_lcd( "H=", 2 );
	   FLOAT_A_LCD_BLE( humedad );
	   write_lcd( '%' );

	   //y en el bluetooth
	   TxtBLE[1] = 'H'; TxtBLE[2] = '=';
	   uartWriteString( UART_BLUETOOTH, TxtBLE );

	   //ponemos la temperatura de rocio en el lcd
	   tempDewPoint =  SHT71_DEW_POINT( temperatura, humedad );

	   setCursor( 2, 3 );
	   print_lcd( "T Rocio=", 8 );
	   FLOAT_A_LCD_BLE( tempDewPoint );
	   write_lcd( 'C' );

	   //y en el bluetooth
	   TxtBLE[0] = 'T'; TxtBLE[1] = 'r'; TxtBLE[2] = '=';
	   uartWriteString( UART_BLUETOOTH, TxtBLE );


	   //leemos el registro de estados
	   REG_STATUS_REGISTER = SHT71_READ_STATUS_REGISTER();

	   for(uint8_t i = 0; i < 16; i++) TxtSTS[1] = ' ';

	   TxtSTS[0] = 'S'; TxtSTS[1] = 't'; TxtSTS[2] = '0'; TxtSTS[3] = '=';

	   //ponemos el estado de la batería en el lcd y el Bluetooth
	   setCursor( 3, 0 );
	   if ( REG_STATUS_REGISTER & SHT71_LOW_BATT ){
		   print_lcd( "B=Lo,", 5);
		   TxtSTS[4] = 'B'; TxtSTS[5] = '='; TxtSTS[6] = 'L';
	   	   TxtSTS[7] = 'o'; TxtSTS[8] = ',';
	   }else{
		   print_lcd( "B=Ok,", 5);
		   TxtSTS[4] = 'B'; TxtSTS[5] = '='; TxtSTS[6] = 'O';
	   	   TxtSTS[7] = 'k'; TxtSTS[8] = ',';
	   }

	   //ponemos el estado del heater en el lcd y el Bluetooth
	   setCursor( 3, 5 );
	   if ( REG_STATUS_REGISTER & SHT71_HEAT_ON ){
		   print_lcd( "He=On, ", 7);
		   TxtSTS[9]  = 'H'; TxtSTS[10] = 'e'; TxtSTS[11] = '=';
	   	   TxtSTS[12] = 'O'; TxtSTS[13] = 'n'; TxtSTS[14] = ',';
	   	   TxtSTS[15] = ' ';
	   }else{
		   print_lcd( "He=Off,", 7);
		   TxtSTS[9]  = 'H'; TxtSTS[10] = 'e'; TxtSTS[11]  = '=';
	   	   TxtSTS[12] = 'O'; TxtSTS[13] = 'f'; TxtSTS[14] = 'f';
	   	   TxtSTS[15] = ',';
	   }
	   uartWriteString( UART_BLUETOOTH, TxtSTS );

	   //ponemos la resolución de la lectura en el lcd y el bluetooth
	   for(uint8_t i = 0; i < 16; i++) TxtSTS[1] = ' ';

	   TxtSTS[0] = 'S'; TxtSTS[1] = 't'; TxtSTS[2] = '1'; TxtSTS[3] = '=';

	   setCursor( 3, 12 );
	   if ( REG_STATUS_REGISTER & SHT71_LOW_RES ){
		   print_lcd( "8RH,12T", 7);
		   TxtSTS[4]  = '8'; TxtSTS[5]  = 'R'; TxtSTS[6] = 'H';
	   	   TxtSTS[7]  = ','; TxtSTS[8]  = '1'; TxtSTS[9] = '2';
	   	   TxtSTS[10] = 'T'; TxtSTS[11] = ' ';
	   }else{
		   print_lcd( "12RH,14T", 8);
		   TxtSTS[4]  = '1'; TxtSTS[5]  = '2'; TxtSTS[6] = 'R';
	   	   TxtSTS[7]  = 'H'; TxtSTS[8]  = ','; TxtSTS[9] = '1';
	   	   TxtSTS[10] = '4'; TxtSTS[11] = 'T';
	   }
	   TxtSTS[12]  = ' '; TxtSTS[13] = ' '; TxtSTS[14] = ' '; TxtSTS[15] = ' ';
	   uartWriteString( UART_BLUETOOTH, TxtSTS );
	}
}

