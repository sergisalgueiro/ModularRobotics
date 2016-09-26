/*
 * touch.c
 *
 *  Created on: 10/6/2015
 *      Author: USUARIO
 */

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "touch.h"

void touch_initialize(void)
{
	DDRF = 0x00; // SET PORT AS INPUT
}

int touch_get(int touch_channel)
{
	PORTF = 0xFF; // PULL-UPS
	int isTouched;
	int channel=0x01;

	channel = channel << touch_channel;

	if (PINF & channel)
		isTouched=1;
	else
		isTouched=0;

	PORTF = 0x00;

	return isTouched;
}
