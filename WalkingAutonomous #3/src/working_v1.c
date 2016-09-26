/*
 * main.c
 *
 *  Created on: 15/6/2015
 *      Author: USUARIO
 */

#define F_CPU 16000000L
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "timers.h"
#include "dynamixel.h"
#include "adc.h"
#include "touch.h"
//#include "walk.h"
#include "serial.h"

#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_MOVING				46
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37

#define RC100_BTN_U		(1)
#define RC100_BTN_D		(2)
#define RC100_BTN_L		(4)
#define RC100_BTN_R		(8)
#define RC100_BTN_1		(16)
#define RC100_BTN_2		(32)
#define RC100_BTN_3		(64)
#define RC100_BTN_4		(128)
#define RC100_BTN_5		(256)
#define RC100_BTN_6		(512)

#define CCW2 880
#define CCW8 880
//#define CCW9 950//1023
#define CCW6 1023
//#define CW2 650//600
#define CW8 600
#define CW9 750
#define CW6 750
#define Base2 750
#define Base8 730
#define Base9 900
#define Base6 900
#define LegDown3 520
#define LegDown4 820
#define LegDown7 830
#define LegDown10 200
#define LegUp3 590
#define LegUp4 750
#define LegUp7 760
#define LegUp10 270

#define		ADC_PORT_1	1
#define		ADC_PORT_2	2
#define		ADC_PORT_3	3
#define		ADC_PORT_4	4
#define		ADC_PORT_5	5
#define		ADC_PORT_6	6

#define LED_BAT 0x01
#define LED_TxD 0x02
#define LED_RxD 0x04
#define LED_AUX 0x08
#define LED_MANAGE 0x10
#define LED_PROGRAM 0x20
#define LED_PLAY 0x40

#define		STRAIGHT  1
#define 	SLEFT      2
#define  	SRIGHT     3
#define 	LEFT      4
#define  	RIGHT     5

#define DEFAULT_BAUDNUM		1 // 1Mbps

void Base(void);
void BaseToA(void);
void Straight (void);
void Right (void);
void Left (void);

int isFinished=1; perform_meas=0, delay=0, selector=1, CCW9, CW2;

int main(void)
{
	int  RcvData,behavior,previousBehavior;
	int  IRfront_val, IRleft_val, DMS_val;

	DDRC  = 0x7F;
	PORTC = 0x7F;

	adc_initialize();
	serial_initialize( 57600 ); // Not using device index
	sei();	// Interrupt Enable
	timer100ms_initialize();
	timer10ms_initialize();
	Base(); //Initialize walking
	selector=1;
	TIMSK3 = (1<<OCIE3A);
	BaseToA();
	previousBehavior=STRAIGHT;
	behavior=STRAIGHT;
	while(1)
	{

	if (perform_meas)
	{
		DMS_val=adc_get(ADC_PORT_1);
		IRfront_val=adc_get(ADC_PORT_5);
		IRleft_val=adc_get(ADC_PORT_2);
		perform_meas=0;
	}
	if (isFinished)
	{
		isFinished=0;
		if (DMS_val > 100 && DMS_val < 200 && IRfront_val < 33)
					{behavior = STRAIGHT;PORTC = ~LED_BAT;}
				else if (DMS_val > 200 && IRfront_val < 33)
					{behavior = SLEFT;PORTC = ~0x02;}
				else if (DMS_val > 75 && DMS_val < 100 && IRfront_val < 33)
					{behavior = SRIGHT;PORTC = ~0x04;}
				else if (DMS_val > 200 && IRfront_val > 33)
					{behavior = LEFT;PORTC = ~0x08;}
				else if (DMS_val < 100 && IRfront_val > 33)
					{behavior = RIGHT;PORTC = ~0x10;}
				else if (DMS_val < 75)
					{behavior = RIGHT;PORTC = ~0x20;}
				else if (IRfront_val > 33 && IRleft_val > 33)
					{behavior = RIGHT;PORTC = ~0x40;}
				else if (IRfront_val < 33 && IRleft_val > 33)
					{behavior = SRIGHT;PORTC = ~0xFF;}
				else
					behavior = STRAIGHT;
	}
		if (previousBehavior != behavior)
		{
			selector=1;
		}
		previousBehavior=behavior;

		switch (behavior)
				{
					case STRAIGHT:
						//CCW9=950;
						//CW2=650;
						CCW9=1000;
						CW2=600;
						Straight();
						break;
					case SLEFT:

						CCW9=900;
						CW2=700;
						Straight();
						break;
					case SRIGHT:

						CCW9=1023;
						CW2=580;
						Straight();
						break;
					case LEFT:

						CCW9=1023;
						CW2=600;
						Left();
						break;
					case RIGHT:

						CCW9=1023;
						CW2=600;
						Right();
						break;
				}

	}

	return 0;
}

ISR(TIMER1_COMPA_vect)
{
	OCR1A += 625;
	perform_meas = 1;
}

ISR(TIMER3_COMPA_vect)
{
	TIMSK3 = (0<<OCIE3A);
	TCNT3 = 0;
	OCR3A = 30000;
	delay=1;
}


//---------------------------Functions-------------------------------------

void Base(void)
{
	//Legs down
	dxl_write_word( 3, P_GOAL_POSITION_L, LegDown3 );
	dxl_write_word( 4, P_GOAL_POSITION_L, LegDown4 );
	dxl_write_word( 7, P_GOAL_POSITION_L, LegDown7 );
	dxl_write_word( 10, P_GOAL_POSITION_L, LegDown10 );
	//Base pose
	dxl_write_word( 2, P_GOAL_POSITION_L, Base2 );
	dxl_write_word( 8, P_GOAL_POSITION_L, Base8 );
	dxl_write_word( 9, P_GOAL_POSITION_L, Base9 );
	dxl_write_word( 6, P_GOAL_POSITION_L, Base6 );
}

void BaseToA (void)
{

	while (delay == 0){};
	delay=0;
	selector++;
	dxl_write_word( 3, P_GOAL_POSITION_L, LegUp3 );
	dxl_write_word( 10, P_GOAL_POSITION_L, LegUp10 );
	TIMSK3 = (1<<OCIE3A);
	//CW
	while (delay == 0){};
	delay=0;
	selector++;
	dxl_write_word( 2, P_GOAL_POSITION_L, CW2 );
	//CCW
	dxl_write_word( 8, P_GOAL_POSITION_L, CCW8 );
	TIMSK3 = (1<<OCIE3A);
	//Legsdown
	while (delay == 0){};
	delay=0;
	selector++;
	dxl_write_word( 10, P_GOAL_POSITION_L, LegDown10 );
	dxl_write_word( 3, P_GOAL_POSITION_L, LegDown3 );
	TIMSK3 = (1<<OCIE3A);
	while (delay == 0){};
	delay=0;
	TIMSK3 = (1<<OCIE3A);
	selector=1;

}

void Straight (void)
{
	if (delay==1 && (selector==1))
	{
	delay=0;
	selector++;
	dxl_write_word( 7, P_GOAL_POSITION_L, LegUp7 );
	dxl_write_word( 4, P_GOAL_POSITION_L, LegUp4 );
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==2))
	{
	delay=0;
	selector++;
	dxl_write_word( 2, P_GOAL_POSITION_L, CCW2 );
	dxl_write_word( 8, P_GOAL_POSITION_L, CW8 );
	dxl_write_word( 6, P_GOAL_POSITION_L, CCW6 );
	dxl_write_word( 9, P_GOAL_POSITION_L, CW9 );
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==3))
	{
	delay=0;
	selector++;
	dxl_write_word( 7, P_GOAL_POSITION_L, LegDown7 );
	dxl_write_word( 4, P_GOAL_POSITION_L, LegDown4 );
	TIMSK3 = (1<<OCIE3A);
	}


	else if (delay==1 && (selector==4))
	{
	dxl_write_word( 3, P_GOAL_POSITION_L, LegUp3 );
	dxl_write_word( 10, P_GOAL_POSITION_L, LegUp10 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==5))
	{
	dxl_write_word( 2, P_GOAL_POSITION_L, CW2 );
	dxl_write_word( 8, P_GOAL_POSITION_L, CCW8 );
	dxl_write_word( 6, P_GOAL_POSITION_L, CW6 );
	dxl_write_word( 9, P_GOAL_POSITION_L, CCW9 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==6))
	{
	dxl_write_word( 3, P_GOAL_POSITION_L, LegDown3 );
	dxl_write_word( 10, P_GOAL_POSITION_L, LegDown10 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}
	else if (delay==1 && (selector==7))
	{
	delay=0;
	selector=1;
	TIMSK3 = (1<<OCIE3A);
	isFinished=1;
	}
}

void Right (void)
{
	if (delay==1 && (selector==1))
	{
	dxl_write_word( 3, P_GOAL_POSITION_L, LegUp3 );
	dxl_write_word( 10, P_GOAL_POSITION_L, LegUp10 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==2))
	{
	dxl_write_word( 2, P_GOAL_POSITION_L, CW2 );
	dxl_write_word( 8, P_GOAL_POSITION_L, CW8 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==3))
	{
	dxl_write_word( 3, P_GOAL_POSITION_L, LegDown3 );
	dxl_write_word( 10, P_GOAL_POSITION_L, LegDown10 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==4))
	{
	dxl_write_word( 7, P_GOAL_POSITION_L, LegUp7 );
	dxl_write_word( 4, P_GOAL_POSITION_L, LegUp4 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==5))
	{
	dxl_write_word( 6, P_GOAL_POSITION_L, CW6 );
	dxl_write_word( 9, P_GOAL_POSITION_L, CW9 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==6))
	{
	dxl_write_word( 7, P_GOAL_POSITION_L, LegDown7 );
	dxl_write_word( 4, P_GOAL_POSITION_L, LegDown4 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==7))
	{
	dxl_write_word( 2, P_GOAL_POSITION_L, Base2 );
	dxl_write_word( 6, P_GOAL_POSITION_L, Base6 );
	dxl_write_word( 8, P_GOAL_POSITION_L, Base8 );
	dxl_write_word( 9, P_GOAL_POSITION_L, Base9 );
	delay=0;
	selector=1;
	TIMSK3 = (1<<OCIE3A);
	isFinished=1;
	}
}

void Left (void)
{
	if (delay==1 && (selector==1))
	{
	dxl_write_word( 3, P_GOAL_POSITION_L, LegUp3 );
	dxl_write_word( 10, P_GOAL_POSITION_L, LegUp10 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==2))
	{
	dxl_write_word( 2, P_GOAL_POSITION_L, CCW2 );
	dxl_write_word( 8, P_GOAL_POSITION_L, CCW8 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==3))
	{
	dxl_write_word( 3, P_GOAL_POSITION_L, LegDown3 );
	dxl_write_word( 10, P_GOAL_POSITION_L, LegDown10 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==4))
	{
	dxl_write_word( 7, P_GOAL_POSITION_L, LegUp7 );
	dxl_write_word( 4, P_GOAL_POSITION_L, LegUp4 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==5))
	{
	dxl_write_word( 6, P_GOAL_POSITION_L, CCW6 );
	dxl_write_word( 9, P_GOAL_POSITION_L, CCW9 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==6))
	{
	dxl_write_word( 7, P_GOAL_POSITION_L, LegDown7 );
	dxl_write_word( 4, P_GOAL_POSITION_L, LegDown4 );
	delay=0;
	selector++;
	TIMSK3 = (1<<OCIE3A);
	}

	else if (delay==1 && (selector==7))
	{
	dxl_write_word( 2, P_GOAL_POSITION_L, Base2 );
	dxl_write_word( 6, P_GOAL_POSITION_L, Base6 );
	dxl_write_word( 8, P_GOAL_POSITION_L, Base8 );
	dxl_write_word( 9, P_GOAL_POSITION_L, Base9 );
	delay=0;
	selector=1;
	TIMSK3 = (1<<OCIE3A);
	isFinished=1;
	}
}

