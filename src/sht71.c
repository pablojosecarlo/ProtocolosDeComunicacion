/*
 *  SHT71.c
 *
 *  Created on: Dec 2, 2019
 *      Author: Pablo JC Alonso Castillo
 */

#include "sht71.h"
#include "sapi.h"
#include "lcd_i2c.h"
#include "math.h"

uint8_t	TX_BUFFER;
uint8_t	RX_BUFFER;

uint8_t LAST_STATUS_REG_SET = 0;
uint8_t	REG_STATUS_REGISTER = 0;
uint8_t	REG_CHECKSUM = 0;
int8_t  NUM[5] = { 0, 0, 0, 0, 0 };
uint8_t TxtBLE[9];
uint8_t TxtSTS[16];

float humedad;
float temperatura;
float tempDewPoint;

typedef struct {
	bool_t ACK;
	uint8_t SCRAP;
} FLAGS_I2C_t;

FLAGS_I2C_t FLAGS_I2C;

#define SHT_VDD   3.5   //valores aceptables: 2.5, 3, 3.3, 3.5, 4, 5,

#define SDA_PIN   GPIO0 //cualquier puerto IO, uso resistencias externas
#define SCL_PIN   GPIO1 //No uso GPIO_INPUT_PULLUP

#define SDA_UP    gpioInit ( SDA_PIN, GPIO_INPUT  )
#define SDA_DOWN  gpioInit ( SDA_PIN, GPIO_OUTPUT )
#define SCL_UP    gpioInit ( SCL_PIN, GPIO_INPUT  )
#define SCL_DOWN  gpioInit ( SCL_PIN, GPIO_OUTPUT )

#define READ_SDA_PIN gpioRead( SDA_PIN )

#define NOP     delay(1)


void SHT71_SET_STATUS( unsigned char elStatus ){

//Setea la constante de status que determina cómo se leerán la temp y humedad

	LAST_STATUS_REG_SET = elStatus;
	SHT71_WRITE_STATUS_REGISTER(elStatus);
}


float SHT71_READ_TEMPERATURA(void){

//LEE LA TEMPERATURA y compensa. entrega el valor real de temperatura

	uint16_t tempCruda = 0;

	do {
		tempCruda = SHT71_READ_TEMPERATURA_CRUDA();
		NOP;
	}
	while( tempCruda > 14000 );  //Esta verificación me ahorra del checksum. . . :)

	//Compensaciones y etc 	ver pag. 5/9 del SHT71.pdf
	float d1, d2;

	if ( SHT_VDD == 2.5 ) d1 = -39.55;
	if ( SHT_VDD == 3   ) d1 = -39.60;
	if ( SHT_VDD == 3.3 ) d1 = -39.63;
	if ( SHT_VDD == 3.5 ) d1 = -39.66;
	if ( SHT_VDD == 4   ) d1 = -39.75;
	if ( SHT_VDD == 5   ) d1 = -40.00;

	if( LAST_STATUS_REG_SET & 0b00000001 ){
		d2 = 0.04;
	}else{
		d2 = 0.01;
	}

	temperatura = d2 * (float)tempCruda + d1;

	return temperatura;
}

float SHT71_READ_HUMEDAD(void){

//LEE LA HUMEDAD y compensa. Entrega el valor final de humedad
//Nota: para compensar en temperatura se debe haber medido ésta previamente

	uint16_t humCruda = 0;

	do{
		humCruda = SHT71_READ_HUMEDAD_CRUDA();
		NOP;
	}
	while( humCruda > 2567 ); //Esta verificación me ahorra del checksum. . . :)

	//Compensaciones y etc 	ver pag. 5/9 del SHT71.pdf
	float c1, c2, c3, t1, t2, SOrh;

	if( LAST_STATUS_REG_SET & SHT71_LOW_RES ){
		//caso de 8  bits HR y 12 bits T
		c1 = -4;   c2 = 0.648;  c3 = -7.2e-4;
		t1 = 0.01; t2 = 0.00008;
	}else{
		//caso de 12 bits HR y 14 bits T
		c1 = -4;   c2 = 0.0405; c3 = -2.8e-6;
		t1 = 0.01; t2 = 0.00128;
	}
	SOrh = (float)humCruda;

	humedad = c1 + c2 * SOrh + c3 * SOrh * SOrh;
	if ( temperatura > 0 ) humedad = (temperatura - 25) * ( t1 + t2 * SOrh ) + humedad;

	return humedad;
}

float SHT71_DEW_POINT( float temp, float hum ){

//Cálculo de la temperatura de rocío, cuanto más me alejo de los 25C más inexacto se vuelve.

	float logEx, dewPoint;

	logEx = 0.66077 +7.5 * temp / ( 237.3 + temp )+( log10( hum ) - 2 );

	dewPoint = ( logEx - 0.66077 ) * 237.3 / ( 0.66077 + 7.5 - logEx );

	return dewPoint;
}


void SETUP_SHT71(void){

//Me reaseguro del estado de los puertos a usar en la interface

	gpioWrite( SDA_PIN, OFF ); //Me aseguro que estén en 0 antes de habilitarlas
	gpioWrite( SCL_PIN, OFF );
	FLAGS_I2C.ACK = FALSE ;
	FLAGS_I2C.SCRAP = 0;

	SDA_DOWN; //Steady State for line & clock
	SCL_DOWN;
	NOP;
	SDA_UP;   //Idle State for line & clock
	SCL_UP;
	delay(5);     //Espero 5 mS
}

void START_CONDITION_SHT71(void){

// 1) TRANSICIÓN DE ALTO A BAJO DEL DATA MIENTRAS ESTA EN ALTO EL CLOCK
// 2) PULSO BAJO DEL CLOCK Y LUEGO NUEVAMENTE EN ALTO, MIENTRAS DATA SIEMPRE EN BAJO
// 3) TRANSICIÓN DE BAJO A ALTO DEL DATA MIENTRAS ESTA EN ALTO EL CLOCK
// Salimos con:
// 4) TRANSICIÓN DE ALTO A BAJO DEL CLOCK  MIENTRAS ESTA EN ALTO EL DATA
// |--\______________/---|-------- DATA
// |------\______/-------|-\______ CLOCK
// RECORDAR QUE EN CNPT LOS DATOS CAMBIAN CUANDO CLOCK ESTA BAJO

	SDA_UP; //Inicio en alto
	SCL_UP;
	NOP;
	SDA_DOWN;
	NOP;
	SCL_DOWN;
	NOP;
	SCL_UP;
	NOP;
	SDA_UP;
	NOP;
	SCL_DOWN;
	NOP;   //AQUI RETORNA CON SCK DOWN Y SDA UP
 }


void SHT71_SOFT_RESET(void){

//RESETEA LA INTERFACE Y COLOCA EL ESTATUS REG EN VALORES POR DEFECTO

	START_CONDITION_SHT71();
	TX_BUFFER = CMD_SHT71_SOFT_RESET;
	I2C_OUTPUT();
	GET_ACK();		//GET ACK BIT
 	TEST_ACK();		//SOLO PARA DEBUGGEO
	//STOP_BIT();	//OUTPUT THE STOP BIT
 	NO_ACK();		//EL STF71 TERMINA CON UN NO_ACK
	//DEBO ESPERAR POR LO MENOS 11MS, PERO ESPERO 50MS POR LAS DUDAS
	delay(50);
	//RETURN TO MAIN PROGRAM
}

void SHT71_CONNECTION_RESET_SECUENCE(void){

// SI SE PIERDE LA COMUNICACIÓN, RESETEA LA INTERFACE SERIE
// 1) DATA EN ALTO DURANTE POR LO MENOS 9 PULSOS DEL CLOCK
// 2) LUEGO UN START CONDITION (NO ESTÁ CLARO SI ESTO PUEDE ESPARAR HASTA QUE DECIDA EL SIGUIENTE COMANDO)
// |-----------------------------------------|--\_________/--|------- DATA
// |__/-\_/-\_/-\_/-\_/-\_/-\_/-\_/-\_/-\_/--|-----\___/-----|------SCK
//    1   2   3   4   5   6   7   8   9     START CONDITION

	SDA_UP;		//CONDICION AL PRINCIPIO DE LA SERIE
	SCL_DOWN;	//CONDICION AL PRINCIPIO DE LA SERIE

	//RESET_SECUENCE: INICIO LA SERIE DE 9 PULSOS
	for( uint8_t i = 0; i < 9; i++){
		NOP;
		SCL_UP;		//INICIO PULSO
		NOP;
		SCL_DOWN;	//FIN PULSO
	}
	START_CONDITION_SHT71();		//OUTPUT LA START CONDITION
}


//****************************************************************************
//	INPUTS TO THIS ROUTINE ARE...
//  EL ARGUMENTO QUE SE MANDA AL STATUS REGISTER
//****************************************************************************
void SHT71_WRITE_STATUS_REGISTER(uint8_t elStausReg){

// WRITE A BYTE TO THE SHT71, SOLO SE PUEDE ESCRIBIR AL STATUS REGISTER

	START_CONDITION_SHT71();		//OUTPUT THE START CONDITION
	TX_BUFFER = CMD_SHT71_WRITE_STATUS_REG;
	I2C_OUTPUT();			//OUTPUT THE DATA ON PORT
	GET_ACK();				//GET ACK BIT
	TEST_ACK();				//SOLO PARA DEBUGGEO
	TX_BUFFER = elStausReg;	//SEND TO THE BUFFER
	I2C_OUTPUT();			//OUTPUT THE DATA ON PORT
	GET_ACK();				//GET ACK BIT
	TEST_ACK();				//ESTO NO TIENE CHISTE, SI EL SLAVE NO RECONOCE (ACK) NO SE TOMA NINGUNA MEDIDA
	NO_ACK();				//EL STF71 TERMINA CON UN NO_ACK
}

//****************************************************************************
//	INPUTS TO THIS ROUTINE ARE... NONE
;//***************************************************************************
uint8_t SHT71_READ_STATUS_REGISTER(void){

//LEE EL STATUS REGISTER DEL SHT71

	START_CONDITION_SHT71();		//OUTPUT THE START CONDITION
	TX_BUFFER = CMD_SHT71_READ_STATUS_REG;
	I2C_OUTPUT();
	GET_ACK();
	TEST_ACK();		//SOLO PARA DEBUGGEO
	NOP;			//SMALL DELAY
	I2C_INPUT();	//GET THE DATA BYTE.
	REG_STATUS_REGISTER = RX_BUFFER;
 	SEND_ACK();		//SEND ACK BIT
	NOP;			//SMALL DELAY
	I2C_INPUT();
	REG_CHECKSUM = 	RX_BUFFER;
	NO_ACK();		//SEND A NOT ACK sht71
	return REG_STATUS_REGISTER; //Si si, ya sé Eric, se copia sobre si mismo. . .
}

uint16_t SHT71_READ_TEMPERATURA_CRUDA(void){

// LEE LOS REGISTROS DE TEMPERATURA SIN PROCESAR

	uint16_t TEMP_CRUDA = 0;
	START_CONDITION_SHT71();	//OUTPUT THE START CONDITION
	TX_BUFFER = CMD_SHT71_READ_TEMPERATURA;
	I2C_OUTPUT();			//OUTPUT THE DATA ON PORT
	GET_ACK();				//GET ACK BIT
	TEST_ACK();				//SOLO PARA DEBUGGEO
	//ESPERAR_MEDICION_DE_TEMPERATURA
	SDA_UP;					//HAGO DE LA LINEA SDA UNA ENTRADA

	gpioWrite(LED2, ON);        //Show de luces

	do{
		NOP;
	}while( READ_SDA_PIN );

	gpioWrite(LED2, OFF);

	I2C_INPUT();			//GET THE DATA BYTE.
	TEMP_CRUDA = RX_BUFFER;	//AND MOVE IT TO DATA IN
	SEND_ACK();				//SEND ACK BIT
	NOP;					//SMALL DELAY
	I2C_INPUT();			//GET THE DATA BYTE.
	TEMP_CRUDA <<= 8;
	TEMP_CRUDA += RX_BUFFER;
//	SEND_ACK();				//SEND ACK BIT
	NO_ACK();				//SI MANDO NOT ACK NO RECIBO EL CHECKSUM Y SE TERMINA AQUI

	//Recibir el CHECSUM aumenta el autocalentamiento del SHT71
	//lo cual implica aumentar el tiempo entre medciones para
	//mantenerlo cerca de la temperatura ambiente

/*
	NOP;					//SMALL DELAY
	I2C_INPUT();			//GET THE DATA BYTE. STORE IN DATA_IN
	REG_CHECKSUM = RX_BUFFER;//GET THE DATA OUT
	//SEND_ACK();			//SEND ACK BIT
	NO_ACK();				//SEND A NOT ACK
	//STOP_BIT;				//OUTPUT THE STOP BIT
*/
	return TEMP_CRUDA;
};

uint16_t SHT71_READ_HUMEDAD_CRUDA(void){

	//LEER HUMEDAD SIN PROCESAR

	uint16_t HUM_CRUDA = 0;
	START_CONDITION_SHT71();	//OUTPUT THE START CONDITION
	TX_BUFFER = CMD_SHT71_READ_HUMEDAD;		//TO OUTPUT BUFFER
	I2C_OUTPUT();				//OUTPUT THE DATA ON PORT
	GET_ACK();					//GET ACK BIT
	TEST_ACK();					//SOLO PARA DEBUGGEO
	//ESPERAR_MEDICION_DE_HUMEDAD
	SDA_UP;						//HAGO DE LA LINEA SDA UNA ENTRADA

	gpioWrite(LED3, ON);        //Show de luces

	do{
		NOP;
	}while( READ_SDA_PIN );

	gpioWrite(LED3, OFF);

	I2C_INPUT();			//GET THE DATA BYTE.
	HUM_CRUDA = RX_BUFFER;
	SEND_ACK();				//SEND ACK BIT
	NOP;					//SMALL DELAY
	I2C_INPUT();			//GET THE DATA BYTE.
	HUM_CRUDA <<= 8;
	HUM_CRUDA += RX_BUFFER;
//	SEND_ACK();				//SEND ACK BIT
	NO_ACK();				//SEND A NOT ACK

	//Recibir el CHECSUM aumenta el autocalentamiento del SHT71
	//lo cual implica aumentar el tiempo entre medciones para
	//mantenerlo cerca de la temperatura ambiente

/*
	NOP;					//SMALL DELAY
	I2C_INPUT();			//GET THE DATA BYTE. STORE IN DATA_IN
	REG_CHECKSUM = RX_BUFFER;//GET THE DATA OUT
	//SEND_ACK();			//SEND ACK BIT
	NO_ACK();				//SEND A NOT ACK
	//STOP_BIT();			//OUTPUT THE STOP BIT
*/
	return HUM_CRUDA;
}

void I2C_OUTPUT(void){
//LOOPEO LOS BITS DESDE LA IZQUIERDA DEL BYTE A ENVIAR
	for(uint8_t i = 0; i < 8; i++ ){
		if (TX_BUFFER & 0b10000000){
			SDA_UP;		//DATA HIGH
		}else{
			SDA_DOWN;	//DATA LOW
		}
		TX_BUFFER = TX_BUFFER << 1;
		NOP;
		//PULSO DEL CLOCK
		SCL_UP;			//CLOCK GOES HIGH
		NOP;
		SCL_DOWN;		//CLOCK GOES ADDLOW AGAIN
		NOP;
	}
	NOP;	//POR CULPA DE NO ESPERAR ESTE TIEMPO EL SSP DEL 16F819 NO ME DETECTABA EL STOP BIT
	//SALGO CON SCL DOWN Y SDA INDEFINIDO
}

void I2C_INPUT(void){
//SI VOY A LEER, EL BIT PASA AL MASTER EN EL FLANCO + DEL CLOCK
	SDA_UP;		//MAKE THE SDA LINE AN INPUT
	NOP;

	for(uint8_t i = 0; i < 8; i++ ){
		SCL_UP;		//CLOCK BACK UP => FLANCO + DEL CLOCK
		NOP;

		if ( READ_SDA_PIN ){
			RX_BUFFER = RX_BUFFER + 1;
		}else{
			RX_BUFFER = RX_BUFFER + 0;
		}

		if ( i < 7 ) RX_BUFFER = RX_BUFFER << 1;

		SCL_DOWN;	//RETURN CLOCK DOWN
		NOP;
	}
	NOP;	//POR CULPA DE NO ESPERAR ESTE TIEMPO EL SSP DEL 16F819 NO ME DETECTABA EL STOP BIT
	//SALGO CON SCL DOWN Y SDA INDEFINIDO
}


void GET_ACK(void){		//ACKNOWLEDGE
//EL MASTER LEE Y GENERA UN PULSO PARA RECIBIR EL BIT DE RECONOCIMIENYO (ACK) DEL SLAVE
//CUANDO LO RECIBE, SETEA EN EL REGISTRO 'FLAGS_I2C' EL BOOLEANO  'ACK'
	SDA_UP;		//MAKE THE SDA LINE AN INPUT
	NOP;
	SCL_UP;		//CLOCK BACK UP
	NOP;

	//TEST FOR LOW ACK (EL ACK DEBE SER BAJO)
	if ( READ_SDA_PIN ){
		//SET THE ACK FLAG=1 -> I2C SI RECONOCIO! :) -> SHT71 NO RECONOCIO :)
		FLAGS_I2C.ACK = TRUE;
	}else{
		//SET THE ACK FLAG=0 -> I2C NO RECONOCIO! :( -> SHT71 SI RECONOCIO :)
		FLAGS_I2C.ACK = FALSE;
	}
	SCL_DOWN;	//RETURN CLOCK ADDLOW
	NOP;
	//SALGO CON SCL DOWN Y SDA UP
}

void SEND_ACK(void){
//CUANDO SE LEE (AL SLAVE), EL MASTER DEBE DAR EL ACK POR CADA BYTE RECIBIDO, EXCEPTO EN EL ÚLTIMO DE TODOS
//EL MASTER AL FINAL DEL ÚLTIMO BYTE RECIBIDO DEBE DAR UN NO-ACK, PARA INDICAR QUE SE ACABO LA TRANSACCIÓN
	SDA_DOWN;	//DATA LINE DOWN
	NOP;
	SCL_UP;		//CLOCK GOES HIGH
	NOP;
	SCL_DOWN;	//CLOCK GOES DOWN
	NOP;
 	//SALGO CON SCL DOWN Y SDA DOWN
}


//
void NO_ACK(void){
//AQUI ESTÁ EL NO-ACK QUE EL MASTER DEBE ENVIAR AL FINAL DEL ÚLTIMO BYTE RECIBIDO, PARA INDICAR QUE SE ACABO LA TRANSACCIÓN
	SDA_UP;		//DATA HIGH
	NOP;
	SCL_UP;		//CLOCK GOES HIGH
	NOP;
	SCL_DOWN;	//CLOCK GOES DOWN
	NOP;
	//SALGO CON SCL DOWN Y SDA UP
}




void TEST_ACK(void){
//MANDA AL LCD UNA N SI NO ESTA EL ACK Y UNA S SI SI ESTA EL ACK
	return;		//PARA QUE ESTA RUTINA FUNCIONE DEBO COMENTAR ESTE RETURN

#ifdef debugueando
	return;
#endif
#ifdef DEMO_LCD
	LCD_WRITE_DATA ("-");
	if ( FLAGS_I2C.ACK ){
		LCD_WRITE_DATA ("S");
	}else{
		LCD_WRITE_DATA ("N");
	};
#endif
	return;
}


void FLOAT_A_LCD_BLE( float x){

// Envia el float con resolución +-###.## al lcd
// Prepara el arreglo TxtBLE para enviar el float como +-###.##
// al Bluetooth LE

	FLOAT_A_DIGITAL5( x );

	if ( x   <  0  ) write_lcd( '-' );
	if ( NUM[4] > 0 ) write_lcd( NUM[4] + 48 );
	if ( NUM[3] > 0 ) write_lcd( NUM[3] + 48 );

	write_lcd( NUM[2] + 48 );
	write_lcd( '.' );
	write_lcd( NUM[1] + 48 );

	if( LAST_STATUS_REG_SET & 1 ){
		write_lcd( 48 );  //0   -> si no tengo resolucion pongo 0 en el segundo decimal
	}else{
		write_lcd( NUM[0] + 48 );
	}

	//preparo el string para mandar al BLE por la usart
	uint8_t i = 0;

	for (; i< 9; i++) TxtBLE[i] = ' ';

	i = 3;
	if (   x    < 0 ){ TxtBLE[i] = '-'        ; i++; }
	if ( NUM[4] > 0 ){ TxtBLE[i] = NUM[4] + 48; i++; }
	if ( NUM[3] > 0 ){ TxtBLE[i] = NUM[3] + 48; i++; }
	                   TxtBLE[i] = NUM[2] + 48; i++;
	                   TxtBLE[i] = '.'        ; i++;
	                   TxtBLE[i] = NUM[1] + 48; i++;

	if( LAST_STATUS_REG_SET & 1 ){
		TxtBLE[i] = 48;  //0   -> si no tengo resolucion pongo 0 en el segundo decimal
	}else{
		TxtBLE[i] = NUM[0] + 48;
	}
}


void FLOAT_A_DIGITAL5(float x){

// Convierte de float a dígitos ascii: Tres enteros y dos decimales
// Asume que el formato del float es ###.## i.e. se considera que solo tien dos decimales válidos
// --> solo se recuperan dos decimales !!
// La salida está en el arreglo NUM[5].

	union
	{
		unsigned char sector[1];
		unsigned int x;
	} unionX;

	unionX.x = (uint16_t) ( 100 * x );
	BIN16_A_DIGITAL5(unionX.sector[1], unionX.sector[0]);	//CONVIERTE HADD Y LADD A NUM[4] -> NUM[0]
}

void BIN16_A_DIGITAL5(uint8_t HADD, uint8_t LADD){
//Si, si. Ya sé Eric. Podria haber hecho directamente que tome un uint16_t pero pienso volver a usar este código y me gusta así

//convierte un uint_16 formando por dos mitades uint8_t en dígitos ascii que guarta en el arreglo NUM[5]

	NUM[0] = 0; NUM[1] = 0; NUM[2] = 0; NUM[3] = 0; NUM[4] = 0;

	//BLOQUE DE PASAR DE BINARIO A GRUPOS DE UNIDADES, DECENAS, CENTENAS, MILES. ETC
	if (LADD & 0b00000001){ NUM[0] +=1; }//1
	if (LADD & 0b00000010){ NUM[0] +=2; }//2
	if (LADD & 0b00000100){ NUM[0] +=4; }//4
	if (LADD & 0b00001000){ NUM[0] +=8; }//8
	if (LADD & 0b00010000){ NUM[0] +=6; NUM[1] += 1; }//16
	if (LADD & 0b00100000){ NUM[0] +=2; NUM[1] += 3; }//32
	if (LADD & 0b01000000){ NUM[0] +=4; NUM[1] += 6; }//64
	if (LADD & 0b10000000){ NUM[0] +=8; NUM[1] += 2; NUM[2] += 1; }//128

	if (HADD & 0b00000001){ NUM[0] +=6; NUM[1] += 5; NUM[2] += 2; }//256
	if (HADD & 0b00000010){ NUM[0] +=2; NUM[1] += 1; NUM[2] += 5; }//512
	if (HADD & 0b00000100){ NUM[0] +=4; NUM[1] += 2; NUM[2] += 0; NUM[3] += 1; }//1024
	if (HADD & 0b00001000){ NUM[0] +=8; NUM[1] += 4; NUM[2] += 0; NUM[3] += 2; }//2048
	if (HADD & 0b00010000){ NUM[0] +=6; NUM[1] += 9; NUM[2] += 0; NUM[3] += 4; }//4096
	if (HADD & 0b00100000){ NUM[0] +=2; NUM[1] += 9; NUM[2] += 1; NUM[3] += 8; }//8192
	if (HADD & 0b01000000){ NUM[0] +=4; NUM[1] += 8; NUM[2] += 3; NUM[3] += 6; NUM[4] += 1; }//16384
	if (HADD & 0b10000000){ NUM[0] +=8; NUM[1] += 6; NUM[2] += 7; NUM[3] += 2; NUM[4] += 3; }//32768
	//BLOQUE DE PASAR DE GRUPOS DE UNIDADES, DECENAS, CENTENAS Y MILES
	//A UNIDADES DE: UNIDAD, DECENA, CENTENA, MIL, DIEZ MIL, ETC.
	while(NUM[0] >= 0) { NUM[0] -= 10; if ( NUM[0] >= 0 ) NUM[1] += 1;} NUM[0] += 10;
	while(NUM[1] >= 0) { NUM[1] -= 10; if ( NUM[1] >= 0 ) NUM[2] += 1;} NUM[1] += 10;
	while(NUM[2] >= 0) { NUM[2] -= 10; if ( NUM[2] >= 0 ) NUM[3] += 1;} NUM[2] += 10;
	while(NUM[3] >= 0) { NUM[3] -= 10; if ( NUM[3] >= 0 ) NUM[4] += 1;} NUM[3] += 10;

}
