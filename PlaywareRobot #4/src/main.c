/*
 * main.c
 *
 *  Created on: 15/6/2015
 *      Author: USUARIO
 */

#define F_CPU 16000000L //DEFINE THE ATMEGA CLOCK
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//SELF-MADE FIRMWARE
#include "timers.h"
#include "dynamixel.h"
#include "adc.h"
#include "touch.h"
#include "serial.h"

//COMMANDS FOR DYNAMIXELS
#define P_GOAL_POSITION_L      30
#define P_GOAL_POSITION_H      31
#define P_MOVING            46
#define P_PRESENT_POSITION_L   36
#define P_PRESENT_POSITION_H   37

//VALUES FOR ZIGBEE RC-100
#define RC100_BTN_U      (1)
#define RC100_BTN_D      (2)
#define RC100_BTN_L      (4)
#define RC100_BTN_R      (8)
#define RC100_BTN_1      (16)
#define RC100_BTN_2      (32)
#define RC100_BTN_3      (64)
#define RC100_BTN_4      (128)
#define RC100_BTN_5      (256)
#define RC100_BTN_6      (512)

//MOTOR POSITIONS FOR WORM MOVEMENT SEQUENCES

//------1UP-----
#define UP1 835
#define UP6 560
#define UP8 157
#define UP21 62

//------2MID-----
#define MID1 810
#define MID6 577
#define MID8 191
#define MID21 285

//------3DOWN-------
#define DOWN1 765
#define DOWN6 577
#define DOWN8 450
#define DOWN21 508

//------4END-------
#define END1 843
#define END6 575
#define END8 690
#define END21 537

//-----5FLAT-----
#define FLAT1 830
#define FLAT6 485
#define FLAT8 610
#define FLAT21 200

//----
#define FLAT22 194
#define RIGHT22 5
#define LEFT22 356

//------TURN---
#define TR11 872
#define TR16 556
#define TR18 579
#define TR121 318

#define TR31 934
#define TR36 564
#define TR38 518
#define TR321 197

//-----ATTACK---
#define CLOSED 424
#define OPEN 510
#define ATT21 10
#define ATT6 400

//-----TAIL-----
#define T1 723
#define T6 601
#define T8 865
#define T21 1


//-----------------------------------
#define      ADC_PORT_1   1
#define      ADC_PORT_2   2
#define      ADC_PORT_3   3
#define      ADC_PORT_4   4
#define      ADC_PORT_5   5
#define      ADC_PORT_6   6

#define LED_BAT 0x01
#define LED_TxD 0x02
#define LED_RxD 0x04
#define LED_AUX 0x08
#define LED_MANAGE 0x10
#define LED_PROGRAM 0x20
#define LED_PLAY 0x40

#define     STRAIGHT  1
#define     LEFT      2
#define     RIGHT     3
#define     ATTACK     4
#define     TAIL     5

#define DEFAULT_BAUDNUM      1 // 1Mbps

void Attack(void);
void Straight(void);
void Right(void);
void Left(void);
void Tail(void);

int isFinished=0, delay=0, selector=1;

int main(void)
{
   int  RcvData,behavior,previousBehavior;

   //PORTS, VARIABLES AND FUNCTIONS INITIALIZATIONS
   DDRC  = 0x7F;
   PORTC = 0x7F;

   PORTD &= ~0x80;   //PORT_LINK_PLUGIN = 0;   // no pull up
   PORTD &= ~0x20;   //PORT_ENABLE_RXD_LINK_PC = 0;
   PORTD |= 0x40;   //PORT_ENABLE_RXD_LINK_ZIGBEE = 1;

   adc_initialize();
   serial_initialize(57600);//Not using device index
   sei();   // Interrupt Enable
   timer1s_initialize();
   selector=1;
   TIMSK3 = (1<<OCIE3A);
   previousBehavior=STRAIGHT;
   behavior=STRAIGHT;


   while(1)
   {


      RcvData=std_getchar();//READING VALUES FROM WI-FI SERIAL COMMUNICATION

      //SETTING THE BEHAVIOR DEPENDING ON KEY PRESSED
            if   (RcvData == RC100_BTN_U)
                  {PORTC= ~LED_MANAGE;
                  behavior=STRAIGHT;}
            else if(RcvData == RC100_BTN_L)
                  {PORTC= ~LED_PROGRAM;behavior=LEFT;}
            else if(RcvData == RC100_BTN_R)
                  {PORTC = ~LED_PLAY;behavior=RIGHT;}
            else if(RcvData == RC100_BTN_D)
                  {PORTC = ~LED_TxD;behavior=ATTACK;}
            else if (RcvData == RC100_BTN_1)
            	  {PORTC = ~LED_TxD;behavior=TAIL;}


      if (previousBehavior != behavior)
      {
         selector=1;
      }
      previousBehavior=behavior;

      //CHOOSING ACTION DEPENDING ON BEHAVIOR
      switch (behavior)
            {
               case STRAIGHT:
                  Straight();
                  break;

               case LEFT:
                  Left();
                  break;

               case RIGHT:
                  Right();
                  break;

               case ATTACK:
                  Attack();
                  break;

               case TAIL:
                  Tail();
                  break;
            }
   }

   return 0;
}


ISR(TIMER3_COMPA_vect)
{
   TIMSK3 = (0<<OCIE3A); //TURN OFF COUNTER
   TCNT3 = 0; // RESET COUNTER
   OCR3A = 19000; // NEW COMPARING VALUE FOR DELAY
   delay=1; //SET DELAY FLAG ON
}


//---------------------------Functions-------------------------------------
/*It's a sequence of order to the motors to move to goal position and then restarting the
 * counter so it has the desired delay time to perform the movement */
void Attack(void)
{
   if (delay==1 && (selector==1))
      {
      delay=0;
      selector++;
      dxl_write_word( 1, P_GOAL_POSITION_L, FLAT1 );
      dxl_write_word( 6, P_GOAL_POSITION_L, FLAT6 );
      dxl_write_word( 8, P_GOAL_POSITION_L, FLAT8 );
      dxl_write_word( 21, P_GOAL_POSITION_L, FLAT21 );
      TIMSK3 = (1<<OCIE3A);
      }
   else if (delay==1 && (selector==2))
      {
            delay=0;
            selector++;
            dxl_write_word( 21, P_GOAL_POSITION_L, ATT21 );
            dxl_write_word( 6, P_GOAL_POSITION_L, ATT6 );
            TIMSK3 = (1<<OCIE3A);
      }
   else if (delay==1 && (selector==3))
      {
            delay=0;
            selector++;
            dxl_write_word( 7, P_GOAL_POSITION_L, OPEN );
            TIMSK3 = (1<<OCIE3A);
      }
   else if (delay==1 && (selector==4))
      {
            delay=0;
            selector++;
            dxl_write_word( 7, P_GOAL_POSITION_L, CLOSED );
            TIMSK3 = (1<<OCIE3A);
      }
   else if (delay==1 && (selector==5))
      {
      delay=0;
      selector++;
      dxl_write_word( 1, P_GOAL_POSITION_L, FLAT1 );
      dxl_write_word( 6, P_GOAL_POSITION_L, FLAT6 );
      dxl_write_word( 8, P_GOAL_POSITION_L, FLAT8 );
      dxl_write_word( 21, P_GOAL_POSITION_L, FLAT21 );
      TIMSK3 = (1<<OCIE3A);
      }
   else if (delay==1 && (selector==6))
   {
   delay=0;
   selector=1;
   TIMSK3 = (1<<OCIE3A);
   isFinished=1;
   }
}

void Tail(void)
{
   if (delay==1 && (selector==1))
      {
      delay=0;
      selector++;
      dxl_write_word( 1, P_GOAL_POSITION_L, FLAT1 );
      dxl_write_word( 6, P_GOAL_POSITION_L, FLAT6 );
      dxl_write_word( 8, P_GOAL_POSITION_L, FLAT8 );
      dxl_write_word( 21, P_GOAL_POSITION_L, FLAT21 );
      TIMSK3 = (1<<OCIE3A);
      }
   else if (delay==1 && (selector==2))
      {
            delay=0;
            selector++;
            dxl_write_word( 1, P_GOAL_POSITION_L, T1 );
            dxl_write_word( 6, P_GOAL_POSITION_L, T6 );
            dxl_write_word( 8, P_GOAL_POSITION_L, T8 );
            dxl_write_word( 21, P_GOAL_POSITION_L, T21 );
            TIMSK3 = (1<<OCIE3A);
      }
   else if (delay==1 && (selector==3))
      {
      delay=0;
      selector++;
      dxl_write_word( 1, P_GOAL_POSITION_L, FLAT1 );
      dxl_write_word( 6, P_GOAL_POSITION_L, FLAT6 );
      dxl_write_word( 8, P_GOAL_POSITION_L, FLAT8 );
      dxl_write_word( 21, P_GOAL_POSITION_L, FLAT21 );
      TIMSK3 = (1<<OCIE3A);
      }
   else if (delay==1 && (selector==4))
   {
   delay=0;
   selector=1;
   TIMSK3 = (1<<OCIE3A);
   isFinished=1;
   }
}


void Straight (void)
{
   if (delay==1 && (selector==1))
   {
   delay=0;
   selector++;
   dxl_write_word( 1, P_GOAL_POSITION_L, UP1 );
   dxl_write_word( 6, P_GOAL_POSITION_L, UP6 );
   dxl_write_word( 8, P_GOAL_POSITION_L, UP8 );
   dxl_write_word( 21, P_GOAL_POSITION_L, UP21 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==2))
   {
   delay=0;
   selector++;
   dxl_write_word( 1, P_GOAL_POSITION_L, MID1 );
   dxl_write_word( 6, P_GOAL_POSITION_L, MID6 );
   dxl_write_word( 8, P_GOAL_POSITION_L, MID8 );
   dxl_write_word( 21, P_GOAL_POSITION_L, MID21 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==3))
   {
   delay=0;
   selector++;
   dxl_write_word( 1, P_GOAL_POSITION_L, DOWN1 );
   dxl_write_word( 6, P_GOAL_POSITION_L, DOWN6 );
   dxl_write_word( 8, P_GOAL_POSITION_L, DOWN8 );
   dxl_write_word( 21, P_GOAL_POSITION_L, DOWN21 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==4))
   {
   delay=0;
   selector++;
   dxl_write_word( 1, P_GOAL_POSITION_L, END1 );
   dxl_write_word( 6, P_GOAL_POSITION_L, END6 );
   dxl_write_word( 8, P_GOAL_POSITION_L, END8 );
   dxl_write_word( 21, P_GOAL_POSITION_L, END21 );
   TIMSK3 = (1<<OCIE3A);
   }
   else if (delay==1 && (selector==5))
   {
      dxl_write_word( 1, P_GOAL_POSITION_L, FLAT1 );
      dxl_write_word( 6, P_GOAL_POSITION_L, FLAT6 );
      dxl_write_word( 8, P_GOAL_POSITION_L, FLAT8 );
      dxl_write_word( 21, P_GOAL_POSITION_L, FLAT21 );
   delay=0;
   selector++;
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==6))
   {
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
   delay=0;
   selector++;
   dxl_write_word( 1, P_GOAL_POSITION_L, TR11 );
   dxl_write_word( 6, P_GOAL_POSITION_L, TR16 );
   dxl_write_word( 8, P_GOAL_POSITION_L, TR18 );
   dxl_write_word( 21, P_GOAL_POSITION_L, TR121 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==2))
   {
   delay=0;
   selector++;
   dxl_write_word( 22, P_GOAL_POSITION_L, RIGHT22 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==3))
   {
   delay=0;
   selector++;
   dxl_write_word( 1, P_GOAL_POSITION_L, TR31 );
   dxl_write_word( 6, P_GOAL_POSITION_L, TR36 );
   dxl_write_word( 8, P_GOAL_POSITION_L, TR38 );
   dxl_write_word( 21, P_GOAL_POSITION_L, TR321 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==4))
   {
   delay=0;
   selector++;
   dxl_write_word( 22, P_GOAL_POSITION_L, FLAT22 );

   TIMSK3 = (1<<OCIE3A);
   }


   else if (delay==1 && (selector==5))
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
   delay=0;
   selector++;
   dxl_write_word( 1, P_GOAL_POSITION_L, TR11 );
   dxl_write_word( 6, P_GOAL_POSITION_L, TR16 );
   dxl_write_word( 8, P_GOAL_POSITION_L, TR18 );
   dxl_write_word( 21, P_GOAL_POSITION_L, TR121 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==2))
   {
   delay=0;
   selector++;
   dxl_write_word( 22, P_GOAL_POSITION_L, LEFT22 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==3))
   {
   delay=0;
   selector++;
   dxl_write_word( 1, P_GOAL_POSITION_L, TR31 );
   dxl_write_word( 6, P_GOAL_POSITION_L, TR36 );
   dxl_write_word( 8, P_GOAL_POSITION_L, TR38 );
   dxl_write_word( 21, P_GOAL_POSITION_L, TR321 );
   TIMSK3 = (1<<OCIE3A);
   }

   else if (delay==1 && (selector==4))
   {
   delay=0;
   selector++;
   dxl_write_word( 22, P_GOAL_POSITION_L, FLAT22 );

   TIMSK3 = (1<<OCIE3A);
   }


   else if (delay==1 && (selector==5))
   {
   delay=0;
   selector=1;
   TIMSK3 = (1<<OCIE3A);
   isFinished=1;
   }
}

