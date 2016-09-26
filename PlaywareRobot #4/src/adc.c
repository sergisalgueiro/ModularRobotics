#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "adc.h"

void adc_initialize(void)
{
	DDRA  = 0xFC;
	PORTA = 0xFC;

	ADCSRA = (1 << ADPS2) | (1 << ADPS1);	// Clock 1/64div.
}

int adc_get(int adc_channel)
{
	ADCSRA = (1 << ADEN);	// ADC Enable

	ADMUX = adc_channel;

	switch(adc_channel)
	{
		case 1:
		{
			PORTA &= ~0x80;			// ADC Port 1 ON
			break;
		}
		case 2:
		{
			PORTA &= ~0x40;			// ADC Port 2 ON
			break;
		}
		case 3:
		{
			PORTA &= ~0x20;			// ADC Port 3 ON
			break;
		}
		case 4:
		{
			PORTA &= ~0x10;			// ADC Port 4 ON
			break;
		}
		case 5:
		{
			PORTA &= ~0x08;			// ADC Port 5 ON
			break;
		}
		case 6:
		{
			PORTA &= ~0x04;			// ADC Port 6 ON
			break;
		}
	}

	_delay_us(12);				// Short Delay for rising sensor signal
	ADCSRA |= (1 << ADIF);		// AD-Conversion Interrupt Flag Clear
	ADCSRA |= (1 << ADSC);		// AD-Conversion Start

	while( !(ADCSRA & (1 << ADIF)) );	// Wait until AD-Conversion complete

	PORTA = 0xFC;	// IR-LED off
	ADCSRA = (0 << ADEN); // ADC disable

	return ADC;
}
