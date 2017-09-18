
#include <stdint.h>

#ifndef MACROS_H_
#define MACROS_H_

// для удобства битовых операций
#define	SetBit(reg, bit)		reg |= (1<<bit)
#define	ClearBit(reg, bit)		reg &= (~(1<<bit))
#define	InvBit(reg, bit)        reg ^= (1<<bit)
#define	BitIsSet(reg, bit)      ((reg & (1<<bit)) != 0)
#define	BitIsClear(reg, bit)    ((reg & (1<<bit)) == 0)	

#define F_CPU	16000000UL

#define DRIVER_PORT PORTB
#define DRIVER_DDR  DDRB
#define DRIVER_PIN  PB1
// значение тока равное 0 810 - измерено имперически
// см заметки.txt
#define CUR_STAB	1150
#define MIN_CUR		900     // при котором включаеться стабилизация 
#define HYST_CUR_DEL2	10	// гистерезис для тока деленный пополам
#define QTY_CUR_MEAS	4	// количество измерений за раз
#define QTY_PERIOD_MEAS	2	// количество периодов измерений
#define START_D		42*180  // это значение установится при токе менее MIN_CUR
#define DEBUG_D		42*180
#define DELTA_D		10      // на столько изменяеться коэффициент заполнения за 1 итерацию
#define MAX_D		42*260  



// структура данных, хранящейся в еепром
#define SIZE_ARRAY_EEPROM	20
struct EEPROMst{
//    uint16_t 	CurD[SIZE_ARRAY_EEPROM];		
    volatile uint16_t 	Cur[SIZE_ARRAY_EEPROM];		
};


#endif /* MACROS_H_ */
