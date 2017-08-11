
#define F_CPU	16000000UL
#define SPI_CS		PB1
#define SPI_LDAC	PB0
#define CUR_STAB	800 //в А * 50 где-то
#define MIN_CUR		25
#define HYST_CUR_DEL2	20	//гистерезис для тока деленный пополам
#define QTY_CUR_MEAS	2	//количество измерений за раз
#define QTY_PERIOD_MEAS	4	//количество периодов измерений
#define K_CUR		1.15	//это еще проверить (близко к правде)
#define START_D		42*100
#define DEBUG_D		42*180
#define DELTA_D		21
#define MAX_D		42*220
#define	SetBit(reg, bit)		reg |= (1<<bit)
#define	ClearBit(reg, bit)		reg &= (~(1<<bit))
#define	InvBit(reg, bit)        reg ^= (1<<bit)
#define	BitIsSet(reg, bit)      ((reg & (1<<bit)) != 0)
#define	BitIsClear(reg, bit)    ((reg & (1<<bit)) == 0)
/*запуск таймера на частоте F_CPU/8 с разрешением прерывания при сравнении с OCR1A*/
#define START_COUNT	TCCR1B|=(1<<CS11); TIMSK|=(1<<OCIE1A);
/*остановка таймера, запрет прерываний*/
#define STOP_COUNT 	TCCR1B&=~(1<<CS11);	TIMSK&=~(1<<OCIE1A);
#define START_NEXT_ADC 	ADCSRA|=(1<<ADSC);		/*запуск АЦП*/
#define START_ADC_CURRENT 	SetBit(ADMUX, MUX0); ADCSRA|=(1<<ADSC);			
#define BUT_PUSH	BitIsSet(PINB,PB6)
#define BUT_UNPUSH	BitIsClear(PINB,PB6)


#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdbool.h>
#include <avr/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h> /*подключаем библиотеку для работы со сторожевым таймером*/
#include "macros.h"
#include "usrlib/eeprom.h"


/*переменные*/
unsigned short int T_work[256];	/*массив со значениями по п.8 алгоритма*/
short int i;					/*буферная переменная, определяет элемент массива*/
bool bAdcDone; 
uint8_t bufADC;
unsigned short cur_SPI_write = 0;
uint16_t curD = 0;
uint8_t NCurMeas = 0;
uint8_t NPeriodMeas = 0;
bool CurMeasDone = false;
struct EEPROMst EEPROM;
bool butActDone = false;
bool StartDone = false;
uint8_t CurIndEEPROM = 0;




/*Внешнее прерывание*/
ISR(INT0_vect)
{
	_delay_us(350);
	if ((PIND&(1<<PD2))==0)
	{
		if (!StartDone) {
			curD = curD + DELTA_D;
			if (curD >= START_D) {
				StartDone = true;
			}
		} else if (cur_SPI_write < MIN_CUR) {
		 	curD = START_D;
/*		} else {
			curD = (curD < DEBUG_D) ? (curD + DELTA_D) : DEBUG_D;
		}
*/
		} else if (CurMeasDone) {
			CurMeasDone = false;
			if (cur_SPI_write < CUR_STAB - HYST_CUR_DEL2) {
				curD = (curD < MAX_D) ? (curD + DELTA_D) : MAX_D;
			} else if (cur_SPI_write > CUR_STAB + HYST_CUR_DEL2) {
				curD = curD - 2*DELTA_D;
			}
		}
		

		if (curD!=0)
		{
			/*включаем выход*/
			PORTD|=(1<<PD0);	
			/*запускаем таймер*/
			/*запись в регистры сравнения*/
			OCR1AH=curD/256;
			OCR1AL=curD%256;
			START_COUNT;
		}
		
	}
}

/*прерывание таймера*/
ISR(TIMER1_COMPA_vect)
{
	/*выключаем выход*/
	PORTD=~(1<<PD0);
	/*останавливаем таймер*/
	STOP_COUNT;
	/*обнуляем регистр счета*/
	TCNT1=0;
	/*запуск АЦП*/
	START_ADC_CURRENT;
	NPeriodMeas++;	
}


/*прерывание таймера*/
ISR(TIMER0_OVF_vect)
{
	__asm__ __volatile__ ("wdr");
}

/*прерывание АЦП*/
ISR(ADC_vect)
{
	/*запоминаем значение с АЦП в буферную переменную*/
	bufADC = ADCL;
	bufADC = ADCH;
	bAdcDone = true;
	SetBit(ADCSRA, ADIF);	//сброс флага прерываний
//	START_NEXT_ADC;		
	
}

int main(void)
{
	 wdt_enable(WDTO_1S);
	 TCCR0|=(1<<CS02)|(1<<CS00); TIMSK|=(1<<TOIE0);
	/*заполнение массива T_work[256] значениями*/
	for (i=0;i<256;i++) {
		T_work[i] = (i < 25) ? 0 : i*40; }
	i=0;
	
	/*включаем внешнее прерывание INT0 по заднему фронту*/
	MCUCR=MCUCR|(1<<ISC01); /*задний фронт INT0*/
	GICR=GICR|(1<<INT0);	/*разрешение прерывания*/
	/*инициализация порта PD0 на выход*/
	DDRD|=(1<<PB0);
	/*инициализация таймера вся в прерываниях*/
	/*инициализация АЦП*/
	/*от внутреннего источника 2,56В, выравнивание по левому краю*/
	ADMUX|=(1<<REFS1)|(1<<REFS0)|(1<<ADLAR);
	/*разрешение работы АЦП, разрешение прерывания, частота F_CPU/128*/
	ADCSRA|=(1<<ADEN)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
	// Инициализации мастера шины SPI
	// Установка выводов SPI на вывод
	DDRB = 0xFF;
	//Включение SPI, режима ведущего, и установка частоты тактирования fclk/128
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
	SetBit(PORTB, SPI_CS);
	SetBit(PORTB, SPI_LDAC);
	/*инициализация спящего режима*/
/*	set_sleep_mode(SLEEP_MODE_IDLE); /*режим сна*/
/*	sleep_enable();		/*разрешаем сон*/
	sei();				/*разрешаем прерывания*/

	
	
    while(1)
    {	
		static uint16_t cur_count = 0;
		static bool new_data_for_SPI_flag = false;
		
		if (bAdcDone) {
			static uint8_t ADC_count = 0;
			bAdcDone = false;

			UpdEEPROM(&EEPROM);

			cur_count = cur_count + bufADC;
			NCurMeas++;
			if (NCurMeas >= QTY_CUR_MEAS) { 
				NCurMeas = 0;
				if (NPeriodMeas >= QTY_PERIOD_MEAS) {
					NPeriodMeas = 0;
					cur_SPI_write = cur_count * K_CUR;
					cur_count = 0;
					new_data_for_SPI_flag = true;
					CurMeasDone = true;
					if (BUT_PUSH && !butActDone) {
						butActDone = true;
						EEPROM.Cur[CurIndEEPROM] = cur_SPI_write;
						EEPROM.CurD[CurIndEEPROM] = curD;
						CurIndEEPROM = 
						(CurIndEEPROM < SIZE_ARRAY_EEPROM-1)
						? CurIndEEPROM++ : 0;
					}
					butActDone = (BUT_UNPUSH) ? false : butActDone;
				}
			} else {
				START_NEXT_ADC; 
			}
		}

		if (new_data_for_SPI_flag) {
			static char	cur_SPI_write_high = 0;
			static char	cur_SPI_write_low = 0;
			static char SPI_step = 0;
			switch (SPI_step) {
				case 0: /*определяем передаваемые байты, СS в 0*/
						cur_SPI_write_high = cur_SPI_write / 256;
						SetBit(cur_SPI_write_high, 4);
						SetBit(cur_SPI_write_high, 5);
						cur_SPI_write_low = cur_SPI_write % 256;
						ClearBit(PORTB, SPI_CS);
						SPI_step = 1;
						break;
				case 1: /*отправляем старший байт*/
						SPDR = cur_SPI_write_high;
						SPI_step = 2;
						break;
				case 2: /*проверяем отправку*/
						if (BitIsSet(SPSR, SPIF)) SPI_step = 3;
						break;
				case 3: /*отправляем младший байт*/
						SPDR = cur_SPI_write_low;
						SPI_step = 4;
						break;
				case 4: /*проверяем отправку, СS в 1*/
						if (BitIsSet(SPSR, SPIF)) {
							SetBit(PORTB, SPI_CS);
							SPI_step = 5; }
						break;
				case 5: /*LDAC в 0, 1мкс задержки LDAC в 1мкс, сброс флага*/
						ClearBit(PORTB, SPI_LDAC);
						_delay_us(1);
						SetBit(PORTB, SPI_LDAC);
						new_data_for_SPI_flag = false;
						SPI_step = 0;
						break;
			} //switch
		} //if (new_data_for_SPI_flag)
						
		/*спящий режим*/
        /*sleep_cpu()*/
    }//while(1)
}

ISR(__vector_default)
{
 
}