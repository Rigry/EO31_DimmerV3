/*
 * macros.h
 *
 * Created: 08.03.2017 7:11:01
 *  Author: dvk
 */ 
#include <stdint.h>

#ifndef MACROS_H_
#define MACROS_H_

	/**
	 * для удобства битовых операций
	 */
	#define	SetBit(reg, bit)		reg |= (1<<bit)
	#define	ClearBit(reg, bit)		reg &= (~(1<<bit))
	#define	InvBit(reg, bit)        reg ^= (1<<bit)
	#define	BitIsSet(reg, bit)      ((reg & (1<<bit)) != 0)
	#define	BitIsClear(reg, bit)    ((reg & (1<<bit)) == 0)	

	#define DEVUNIQNUMBER	1		//для этой прошивки и этого устройства, согласно таблице кодов устройств ЭО-

	#define PASSWORD		0xC2B8	//для смены заводского номера

//	#define F_CPU	14745600UL

	#define CHANQTY	8

	#define QTY_IN_REG	13
	#define QTY_OUT_REG	5

	#define MAINWORKCICLE_PRESCALER	1024 // 0 8 64 256 1024

	#define UART_BUF_SIZE 100

	#define RTS_DDR 	DDRD
	#define RTS_PORT 	PORTD
	#define RTS_PIN 	PD2
	
	//пин включения заводских настроек
	#define FABIN_PORT	PORTD
	#define FABIN_PIN	PD5

	//пин зеленого индикатора
	#define GLED_DDR	DDRD
	#define GLED_PORT	PORTD
	#define GLED_PIN	PD7

	//пин красного индикатора
	#define RLED_DDR	DDRD
	#define RLED_PORT	PORTD
	#define RLED_PIN	PD6

	#define GLED_ON()	SetBit(GLED_DDR, GLED_PIN)
	#define GLED_OFF()	ClearBit(GLED_DDR, GLED_PIN)	
	#define RLED_ON()	SetBit(RLED_DDR, RLED_PIN)
	#define RLED_OFF()	ClearBit(RLED_DDR, RLED_PIN)


	/**
	 * структура данных, хранящейся в еепром
	 */
	#define SIZE_ARRAY_EEPROM	20
	struct EEPROMst{
		uint16_t 	CurD[SIZE_ARRAY_EEPROM];		
		uint16_t 	Cur[SIZE_ARRAY_EEPROM];		
	};

#endif /* MACROS_H_ */
