
#include "macros.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdbool.h>
#include <avr/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h> 
#include "usrlib/eeprom.h"

// запуск таймера на частоте F_CPU/8 с разрешением прерывания при сравнении с OCR1A
#define START_COUNT()	    TCCR1B|=(1<<CS11); TIMSK|=(1<<OCIE1A);
// остановка таймера, запрет прерываний
#define STOP_COUNT()        TCCR1B&=~(1<<CS11);	TIMSK&=~(1<<OCIE1A);
#define START_NEXT_ADC()    ADCSRA|=(1<<ADSC);
#define START_ADC_CURRENT() SetBit(ADMUX, MUX0); ADCSRA|=(1<<ADSC);			
//#define BUT_PUSH()      BitIsSet(PINB,PB6)
//#define BUT_UNPUSH()    BitIsClear(PINB,PB6)


bool bAdcDone = false; 
volatile uint8_t bufADC = 0;
uint16_t curMeas = 0;
uint16_t curMeasSumm = 0;
volatile uint16_t curD = 0;
uint8_t NCurMeas = 0;
uint8_t NPeriodMeas = 0;
bool CurMeasDone = false;
volatile struct EEPROMst EEPROM;
bool butActDone = false;
bool StartDone = false;
volatile uint8_t CurIndEEPROM = 0;


int main(void)
{
	wdt_enable(WDTO_1S);
//	TCCR0|=(1<<CS02)|(1<<CS00); TIMSK|=(1<<TOIE0);

	// включаем внешнее прерывание INT0 по заднему фронту
	MCUCR=MCUCR|(1<<ISC01); // задний фронт INT0
	GICR=GICR|(1<<INT0);	// разрешение прерывания
	// инициализация порта PD0 на выход
    SetBit (DRIVER_DDR, DRIVER_PIN);
	// инициализация таймера вся в прерываниях
	// инициализация АЦП
	// от Vcc, выравнивание по левому краю
	ADMUX|=(1<<REFS0)|(1<<ADLAR);
	// разрешение работы АЦП, разрешение прерывания, частота F_CPU/128
	ADCSRA|=(1<<ADEN)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);

	sei();      // разрешаем прерывания

    while(1)
    {	
		

		UpdEEPROM(&EEPROM);
		
		if (bAdcDone) {
			bAdcDone = false;
			curMeasSumm = curMeasSumm + bufADC;
			NCurMeas++;
			if (NCurMeas >= QTY_CUR_MEAS) { 
				NCurMeas = 0;
				if (NPeriodMeas >= QTY_PERIOD_MEAS) {
					NPeriodMeas = 0;
					curMeas = curMeasSumm;
					curMeasSumm = 0;
					CurMeasDone = true;
				}
			} else {
				START_NEXT_ADC(); 
			}
		}

    }//while(1)
}

// Внешнее прерывание
ISR(INT0_vect)
{
	_delay_us(350);
    __asm__ __volatile__ ("wdr");
	if ((PIND&(1<<PD2))==0)	{
		if (!StartDone) {
			curD = curD + DELTA_D;
			if (curD >= START_D) {
				StartDone = true;
			}
		} else if (curMeas < MIN_CUR) {
		 	curD = START_D;
/*		} else {
			curD = (curD < DEBUG_D) ? (curD + DELTA_D) : DEBUG_D;
		}
*/
		} else if (CurMeasDone) {
			CurMeasDone = false;
/* отладка
            EEPROM.Cur[CurIndEEPROM] = curMeas;
            if (CurIndEEPROM >= SIZE_ARRAY_EEPROM) {
                CurIndEEPROM = 0;
            } else {
                CurIndEEPROM++;
            }
*/
			if (curMeas < CUR_STAB - HYST_CUR_DEL2) {
				curD = (curD < MAX_D) ? (curD + DELTA_D) : MAX_D;
			} else if (curMeas > CUR_STAB + HYST_CUR_DEL2) {
				curD = curD - DELTA_D;
			}
		}

		if (curD != 0) {
			// включаем выход
            SetBit (DRIVER_PORT, DRIVER_PIN);
			// запускаем таймер
			OCR1AH = curD / 256;
			OCR1AL = curD % 256;
			START_COUNT();
		}
	}
} // main

// прерывание таймера
ISR(TIMER1_COMPA_vect)
{
	// выключаем выход
	ClearBit (DRIVER_PORT, DRIVER_PIN);
	STOP_COUNT();
	// обнуляем регистр счета
	TCNT1=0;
	START_ADC_CURRENT();
	NPeriodMeas++;	
}

// прерывание АЦП
ISR(ADC_vect)
{
	bufADC = ADCL;
	bufADC = ADCH;
	bAdcDone = true;
    //сброс флага прерываний
	SetBit(ADCSRA, ADIF);	
}

ISR(__vector_default)
{
 
}