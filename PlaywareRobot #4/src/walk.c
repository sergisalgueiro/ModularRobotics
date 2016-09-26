//##########################################################
//##                      R O B O T I S                   ##
//## CM-510 (Atmega2561) Example code for Dynamixel.      ##
//##                                           2009.11.10 ##
//##########################################################
#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

#include "dynamixel.h"
#include "walk.h"

/// Control table address
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_MOVING				46
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37

// Defulat setting
#define DEFAULT_BAUDNUM		1 // 1Mbps

#define CCW2 880
#define CCW8 880
#define CCW9 1023
#define CCW6 1023
#define CW2 600
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


/*
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
*/
packet AtoB (struct data packet)
{

	packet.delay=packet.delay+1;
	packet.selector=packet.selector+1;
	/*if (pack.delay==1 && (selector==1))
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
	}*/
	return packet;
}
/*
void Right (struct data)
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
	}
}

void Left (struct data)
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
	}
}*/
