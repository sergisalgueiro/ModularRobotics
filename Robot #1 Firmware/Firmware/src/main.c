#define F_CPU 16000000L

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "timers.h"
#include "serial.h"
#include "dynamixel.h"
#include "adc.h"
#include "touch.h"
#include "motors.h"

// Define constants
#define		ADC_PORT_1	1
#define		ADC_PORT_2	2
#define		ADC_PORT_3	3
#define		ADC_PORT_4	4
#define		ADC_PORT_5	5
#define		ADC_PORT_6	6

#define		GO_STRAIGHT    1
#define 	TURN_LEFT      2
#define  	TURN_RIGHT     3
#define 	TURN_LEFT_90   4
#define     TURN_RIGHT_90  5
#define 	OTHER          6

#define     LED_OFF        0x7E
#define     LED_STRAIGHT   0x5B
#define     LED_LEFT       0x3E
#define     LED_RIGHT      0x6E
#define     LED_LEFT90     0x36
#define     LED_RIGHT90    0x6C
#define     LED_OTHER      0x24

#define DEFAULT_BAUDNUM		1 // 1Mbps

int perform_meas = 0;

int main(void)
{
	// Define the outputs corresponding to LEDs
	DDRC = 0x7E;
	PORTC = LED_OFF;

	// Define speeds of motors
	int speed_straight = 90;
	int speed_turn_left[2] = {15 , 50};
	int speed_turn_right[2] = {80 , 40};

	// Define the initial behavior of a robot
	int behavior = 1;

	// Define variables corresponding to sensors
	int DMS_val = 0;
	int IRfront_val = 0;
	int IRleft_val = 0;

	// Define variables used for specyfing speeds of motors
	int speed_left = 0;
	int speed_right = 0;
	int speed_turn = 0;
	int rate_turn = 0;

	// Enable global interrupts
	sei();
	// Initialize timer, adc channels, serial port and motors
	timer10ms_initialize();
	adc_initialize();
	serial_initialize(57600);
	dxl_initialize( 0, DEFAULT_BAUDNUM );

	while (1)
	{
		if(perform_meas)
		{
			// Perform measurments
			DMS_val = adc_get(ADC_PORT_1);
			IRfront_val = adc_get(ADC_PORT_2);
			IRleft_val = adc_get(ADC_PORT_3);
			perform_meas = 0;
		}

		// Specify desired action based on measured values
		if (DMS_val > 100 && DMS_val < 200 && IRfront_val < 33)
			behavior = GO_STRAIGHT;
		else if (DMS_val > 200 && IRfront_val < 33)
			behavior = TURN_LEFT;
		else if (DMS_val > 75 && DMS_val < 100 && IRfront_val < 33)
			behavior = TURN_RIGHT;
		else if (DMS_val > 200 && IRfront_val > 33)
			behavior = TURN_LEFT_90;
		else if (DMS_val < 100 && IRfront_val > 33)
			behavior = TURN_RIGHT_90;
		else if (DMS_val < 75)
			behavior = TURN_RIGHT_90;
		else if (IRfront_val > 33 && IRleft_val > 33)
			behavior = TURN_RIGHT_90;
		else if (IRfront_val < 33 && IRleft_val > 33)
			behavior = TURN_RIGHT;
		else
			behavior = OTHER;

		// Apply desired behavior
		switch (behavior)
		{
			case GO_STRAIGHT:
				PORTC = LED_STRAIGHT;
				speed_left = speed_straight;
				speed_right = speed_straight;
				forwardLeftMotor(speed_left);
				forwardRightMotor(speed_right);
				PORTC = LED_OFF;
				break;

			case TURN_LEFT:
				PORTC = LED_LEFT;
				speed_left = speed_turn_left[0];
				speed_right = speed_turn_left[1];
				forwardLeftMotor(speed_left);
				forwardRightMotor(speed_right);
				PORTC = LED_OFF;
				break;

			case TURN_RIGHT:
				PORTC = LED_RIGHT;
				speed_left = speed_turn_right[0];
				speed_right = speed_turn_right[1];
				forwardLeftMotor(speed_left);
				forwardRightMotor(speed_right);
				PORTC = LED_OFF;
				break;

			case TURN_LEFT_90:
				PORTC = LED_LEFT90;
				speed_turn = 50;
				rate_turn = 7;
				TurnLeft(rate_turn , speed_turn);
				PORTC = LED_OFF;
				break;

			case TURN_RIGHT_90:
				PORTC = LED_RIGHT90;
				speed_turn = 50;
				rate_turn = 7;
				TurnRight(rate_turn , speed_turn);
				PORTC = LED_OFF;
				break;

			case OTHER:
				PORTC = LED_OTHER;
				speed_left = speed_straight;
				speed_right = speed_straight;
				forwardLeftMotor(speed_left);
				forwardRightMotor(speed_right);
				PORTC = LED_OFF;
				break;
		}
	}
	return 1;
}

ISR(TIMER1_COMPA_vect)
{
	// Additional 10ms for next interrupt
	OCR1A += 625;
	perform_meas = 1;
}
