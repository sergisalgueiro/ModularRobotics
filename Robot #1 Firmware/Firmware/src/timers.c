/*
 * timers.c
 *
 *  Created on: 11/6/2015
 *      Author: USUARIO
 */
#include <avr/io.h>
#include <stdio.h>

#include "timers.h"

void timer10ms_initialize(void)
{
	TCCR1B=(1<<CS12); // 256 prescaler
	OCR1A = 625; //10 ms
	TIMSK1 = (1<<OCIE1A); //Enable Output Compare Match Interrupt
	TCNT1 = 0; //reset timer/counter 1
}
