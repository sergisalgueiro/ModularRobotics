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

void timer100ms_initialize(void)
{
	TCCR3B=(1<<CS31)|(1<<CS30); // 64 prescaler
	OCR3A = 25000; //100 ms
	//TIMSK3 = (1<<OCIE3A); //Enable Output Compare Match Interrupt
	TCNT3 = 0; //reset timer/counter 3
}

void timer1s_initialize(void)
{
	TCCR3B=(1<<CS32); // 256 prescaler
	OCR3A = 62500; //1000 ms
	//TIMSK3 = (1<<OCIE3A); //Enable Output Compare Match Interrupt
	TCNT3 = 0; //reset timer/counter 3
}

int get_time(int time)
{
	return time;
}
