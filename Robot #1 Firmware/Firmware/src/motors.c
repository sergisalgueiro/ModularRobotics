#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "dynamixel.h"
#include "motors.h"

/// Control table address
#define P_MOVING_SPEED_L		32
#define P_MOVING_SPEED_H		33
#define P_PRESENT_SPEED_L		38
#define P_PRESENT_SPEED_H		39
#define P_MOVING				46

void forwardLeftMotor(int speedFL)
{
	int motorspeed = speedFL*10;
	dxl_write_word( 6, P_MOVING_SPEED_L, motorspeed );
}

void forwardRightMotor(int speedFR)
{
	int motorspeed = speedFR*10+1024;
	dxl_write_word( 10, P_MOVING_SPEED_L, motorspeed );
}

void BackLeftMotor(int speedBL)
{
	int motorspeed = (speedBL*10)+1024;
	dxl_write_word( 6, P_MOVING_SPEED_L, motorspeed );
}

void BackRightMotor(int speedBR)
{
	int motorspeed = (speedBR*10);
	dxl_write_word( 10, P_MOVING_SPEED_L, motorspeed );
}

void StopLeftMotor()
{
	dxl_write_word( 6, P_MOVING_SPEED_L, 0 );
}

void StopRightMotor()
{
	dxl_write_word( 10, P_MOVING_SPEED_L, 0 );
}

void TurnRight(int turnL, int turnspeedL)
{
	for (int i=0; i<turnL; i++)
	{
		dxl_write_word( 10, P_MOVING_SPEED_L, turnspeedL*10 );
		dxl_write_word( 6, P_MOVING_SPEED_L, turnspeedL*10 );
	}
}

void TurnLeft(int turnR, int turnspeedR)
{
	for (int i = 0; i<turnR; i++)
	{
		turnspeedR = turnspeedR*10+1024;
		dxl_write_word( 10, P_MOVING_SPEED_L, turnspeedR );
		dxl_write_word( 6, P_MOVING_SPEED_L, turnspeedR );
	}
}
