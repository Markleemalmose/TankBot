/*
 * MotorController.c
 *
 * Created: 05-03-2015 12:23:13
 *  Author: T
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
#include <util/delay.h>
#include "lineStateMachine.h"
#include <util/delay.h>
#include "UART.h"

//#define F_CPU 16000000UL
#define INTERVAL_ADJUST_SPEED 10 // In ms
#define PWM_FACTOR 7
#define K_P 3
#define K_I 1

void uart_puts(char *s);

volatile int stopFlag = 0;

unsigned int encoderValueRight = 0;
unsigned int encoderValueLeft = 0;

volatile bool adjustSpeed = false;
volatile uint16_t time1ms = 0;

volatile int IValue;
volatile int errorDiffRight;
volatile int errorDiffLeft;

int backwardsLeft = 0;
int backwardsRight = 0;
int forwardsLeft = 0;
int forwardsRight = 0;


int speedReference = 20*PWM_FACTOR; // 1 RPS = 1032. Max 8 RPS. Measured max 70 per 10 mS
int pwmMax = 511;
volatile int outLeftSide;
volatile int outRightSide;

volatile int turn90degreesLeft;
volatile int turn90degreesRight;
//int firstStop = 0;

void motor_init(void)
{
    // Encoder motor left A and B
    DDRE &= ~(1<<DDE6) & ~(1<<DDE7);
    
    // Encoder motor right A and B
    DDRE  &= ~(1<<DDE4) & ~(1<<DDE5);
    
    cli(); // disable global interrupts
    
    EIFR |= (1<<INT7)|(1<<INT6)|(1<<INT5)|(1<<INT4);
    EIMSK |= (1<<INT7)|(1<<INT6)|(1<<INT5)|(1<<INT4);
    EICRB |= (1<<ISC70)|(1<<ISC60)|(1<<ISC50)|(1<<ISC40);
    //    EICRB |= (1<<ISC71)|(1<<ISC61)|(1<<ISC51)|(1<<ISC41);
    
    
    sei(); //Enable global interrupts. SREG = (1<<I);
    
    // Motor input
    DDRC |= (1<<DDC0)|(1<<DDC1)|(1<<DDC2)|(1<<DDC3); // Outputs
    
    PORTC |= (1<<PORTC0);   // High. Forward motor Left
    PORTC &= ~(1<<PORTC1);  // Low Forward motor Left
    
    PORTC |= (1<<PORTC3);   // High. Forward motor Right
    PORTC &= ~(1<<PORTC2);  // Low Forward motor Right
}

void motor_timer_init(void)
{
    // Timer 0 // adjust timer
    TCCR0A = (1<<WGM01);
    TCCR0B = (1<<CS02); // prescaler 256
    OCR0A = 71;
    TIMSK0 = (1<<OCIE0A);
    
    // Timer 1 // PWM
    TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(0<<WGM10)|(1<<WGM11);
    TCCR1B = (1<<CS10)|(0<<WGM12);
    //ICR1 = 511;
    OCR1B = 1;
    OCR1A = 1;
    
    // PWM
    DDRB |= (1<<DDB5)|(1<<DDB6);    // Output PWM
}


ISR(INT4_vect)
{
    encoderValueRight++;
    
    if (PINE & (1<<PE4)) {
        
        if (!(PINE & (1<<PE5))) {
            forwardsRight = 1;
        } else {
            forwardsRight = 0;
        }
        
    }
    
}

ISR(INT5_vect)
{
    encoderValueRight++;
    
    if (PINE & (1<<PE5)) {
        
        if (!(PINE & (1<<PE4))) {
            forwardsRight = 0;
        } else {
            forwardsRight = 1;
        }
    }
}

ISR(INT6_vect)
{
    encoderValueLeft++;
    
    
    if (PINE & (1<<PE6))
    {
        
        if (!(PINE & (1<<PE7)))
        {
            forwardsLeft = 0;
        }
        else
        {
            forwardsLeft = 1;
        }
        
    }
}

ISR(INT7_vect)
{
    encoderValueLeft++;
    
    if (PINE & (1<<PE7)) {
        
        if (!(PINE & (1<<PE6))) {
            forwardsLeft = 1;
        } else {
            forwardsLeft = 0;
        }
    }
    
    
}



ISR(TIMER0_COMPA_vect) // Comes every 1ms
{
    volatile static uint16_t count10ms = INTERVAL_ADJUST_SPEED;
    time1ms++;
    
    if ((--count10ms) == 0)  // Do if 1 minus count10ms = 0
    {
        
        count10ms = INTERVAL_ADJUST_SPEED; //
        adjustSpeed = true;      // Set adjustSpeed to 1. Activating this function to be run next time
    }
}


void forwardMotorLeft(void)
{
    PORTC |= (1<<PORTC3); // High. Forward motor left
    PORTC &= ~(1<<PORTC2); // Low Forward motor left
}

void forwardMotorRight(void)
{
    PORTC |= (1<<PORTC0); // High. Forward motor right
    PORTC &= ~(1<<PORTC1); // Low Forward motor right
}

void backwardsMotorLeft(void)
{
    PORTC |= (1<<PORTC2); // High backwards motor left
    PORTC &= ~(1<<PORTC3); // Low backwards motor left
}

void backwardsMotorRight(void)
{
    PORTC |= (1<<PORTC1); // High backwards motor right
    PORTC &= ~(1<<PORTC0); // Low backwards motor right
}

char temp1[4];
char temp2[4];
char temp3[4];
char temp4[4];
char temp5[4];
char temp6[4];
char temp7[4];
char temp8[4];


void drive(int direction, int speed)
{
    
    //    itoa(forwardsLeft, temp1,10);
    //    uart_puts("Forward Left: ");
    //    uart_puts(temp1);
    //    uart_puts("        ");
    //    itoa(forwardsRight, temp2,10);
    //    uart_puts("Forward Right: ");
    //    uart_puts(temp2);
    //    uart_puts("\r\n");
    
    //   Encoder values:
//                itoa(encoderValueLeft, temp1, 10);
//                itoa(encoderValueRight, temp2, 10);
//                uart_puts("Encoder Left: ");
//                uart_puts(temp1);
//                uart_puts("     ");
//    
//    
//                uart_puts("Encoder Right: ");
//                uart_puts(temp2);
//                uart_puts("\r\n");
//    
    
    
    // Scaled speed reference
    speedReference = speed*PWM_FACTOR;
    
    // Scaled encoder values
    // Left motor
    unsigned int encoderValueLeftScaled = encoderValueLeft*PWM_FACTOR;
    if (!forwardsLeft) {
        encoderValueLeftScaled = -(encoderValueLeftScaled);
    }
    encoderValueLeft = 0;   // reset variable
    // Right motor
    unsigned int encoderValueRightScaled = encoderValueRight*PWM_FACTOR;
    if (!forwardsRight) {
        encoderValueRightScaled = -(encoderValueRightScaled);
    }
    encoderValueRight = 0;   // reset variable
    
//                    itoa(encoderValueLeftScaled, temp7, 10);
//                    itoa(encoderValueRightScaled, temp8, 10);
//                    uart_puts("Encoder Left: ");
//                    uart_puts(temp7);
//                    uart_puts("     ");
//    
//    
//                    uart_puts("Encoder Right: ");
//                    uart_puts(temp8);
//                    uart_puts("\r\n");
//    
    
    int IValueTemp = (encoderValueRightScaled + (-encoderValueLeftScaled));
    //  int IValueTemp = (encoderValueRightScaled + (-encoderValueLeftScaled) +direction);
    
//    IValueTemp /= 2;    // split value, one for each motor
    IValueTemp *= K_I;    // split value, one for each motor
    
    IValue += IValueTemp;
    
    if (IValue > pwmMax)
    {
        IValue = pwmMax;
    }
    else if (IValue < -pwmMax)
    {
        IValue = -pwmMax;
    }
    
    if(outLeftSide)
    {
        // Turn hard right
        errorDiffRight = ((speedReference - encoderValueRightScaled) - (IValue)); // slowdown right motor
        errorDiffLeft = ((speedReference - encoderValueLeftScaled)); // Keep speed left motor
    }
    else if(outRightSide)
    {
        // Turn hard left
        errorDiffLeft = ((speedReference - encoderValueLeftScaled) + (IValue)); // slowdown left motor
        errorDiffRight = ((speedReference - encoderValueRightScaled)); // Keep speed right motor
    }
    else if(direction == 1) // Nudge left
    {
        errorDiffLeft = (0 - (encoderValueLeftScaled/2)); // stop left motor
        errorDiffRight = ((speedReference - encoderValueRightScaled)); // Keep speed right motor
    }
    else if(direction == 2) // Nudge right
    {
        errorDiffRight = (0 - (encoderValueRightScaled/2)); // stop right motor
        errorDiffLeft = ((speedReference - encoderValueLeftScaled)); // Keep speed left motor
    }
    else if(direction == 3) // medium left
    {
        errorDiffLeft = (0 - encoderValueLeftScaled); // stop left motor
        errorDiffRight = ((speedReference - encoderValueRightScaled)); // Keep speed right motor
    }
    else if(direction == 4) // medium right
    {
        errorDiffRight = (0 - encoderValueRightScaled); // stop right motor
        errorDiffLeft = ((speedReference - encoderValueLeftScaled)); // Keep speed left motor
    }
    else if(direction == 7)  // turn 90 degrees left
    {
        errorDiffRight = (0); //((speedReference - encoderValueRightScaled));
        errorDiffLeft = -300;
        
        backwardsMotorLeft();
    }
    else if(direction == 8) // turn 90 degrees right
    {
        errorDiffLeft = 0; //((speedReference - encoderValueLeftScaled));
        errorDiffRight = -300;
        
        backwardsMotorRight();
    }
    else if(direction == 9) // turn left to line
    {
        errorDiffLeft = -150; //((speedReference - encoderValueLeftScaled));
        errorDiffRight = 150;
    }
    else if(direction == 10) // Turn right to line
    {
        errorDiffLeft = 150; //((speedReference - encoderValueLeftScaled));
        errorDiffRight = -150;
    }
    else if(direction == 11)  // circle left
    {

        errorDiffLeft = -15;
        errorDiffRight = 600;
//        errorDiffLeft = ((0 - (encoderValueLeftScaled))/64); // little forward left motor
//        errorDiffRight = ((speedReference - encoderValueRightScaled)); // Keep speed right motor
        
        backwardsMotorLeft();
    }
    else if(direction == 12) // circle right
    {
        errorDiffLeft = 600;
        errorDiffRight = -15;
//        errorDiffLeft = (speedReference - (encoderValueLeftScaled));
//        errorDiffRight = (0 - (encoderValueRightScaled/64));
        
        backwardsMotorRight();
    }
    else if(direction == 13)  // soft left
    {
        errorDiffLeft = (0); // little forward left motor
        errorDiffRight = ((speedReference - encoderValueRightScaled)); // Keep speed right motor
        
        backwardsMotorLeft();
    }
    else
    {
        errorDiffRight = ((speedReference - encoderValueRightScaled) - (IValue));   // drive straight
        errorDiffLeft = ((speedReference - encoderValueLeftScaled) + (IValue));     // drive straight
        
        
    }
    
    //    itoa(errorDiffLeft, temp5, 10);
    //    itoa(errorDiffRight, temp6, 10);
    //    uart_puts("errorDiffLeft: ");
    //    uart_puts(temp5);
    //    uart_puts("        ");
    //
    //
    //    uart_puts("errorDiffRight: ");
    //    uart_puts(temp6);
    //    uart_puts("\r\n");
    
    
    
    // P block
    
    if (!stopFlag) {
        errorDiffRight *= K_P; // Factor to be calculated
        errorDiffLeft *= K_P;
    }
    
    //    itoa(errorDiffLeft, temp3, 10);
    //    itoa(errorDiffRight, temp4, 10);
    //    uart_puts("errorDiffLeft after: ");
    //    uart_puts(temp3);
    //    uart_puts("        ");
    //
    //
    //    uart_puts("errorDiffRight after: ");
    //    uart_puts(temp4);
    //    uart_puts("\r\n");
    //
    // PWM to motor right
    
    if (errorDiffRight > pwmMax)
    {
        OCR1A = pwmMax;
        forwardMotorRight();
    }
    else if (errorDiffRight < -pwmMax)
    {
        OCR1A = pwmMax;
        backwardsMotorRight();
    }
    else if (errorDiffRight < 0)
    {
        OCR1A = abs(errorDiffRight);
        backwardsMotorRight();
    }
    else
    {
        OCR1A = errorDiffRight;
        forwardMotorRight();
    }
    
    
    // PWM to motor left
    
    if (errorDiffLeft > pwmMax)
    {
        //        uart_puts("den er her 1\r\n");
        OCR1B = pwmMax;
        forwardMotorLeft();
    }
    else if (errorDiffLeft < -pwmMax)
    {
        
        //        uart_puts("den er her 2\r\n");
        OCR1B = pwmMax;
        backwardsMotorLeft();
    }
    else if (errorDiffLeft < 0)
    {
        //        uart_puts("den er her 3\r\n");
        OCR1B = abs(errorDiffLeft);
        backwardsMotorLeft();
    }
    else
    {
        
        //        uart_puts("den er her 4\r\n");
        OCR1B = errorDiffLeft;
        forwardMotorLeft();
    }
    
   
   
    
    // STOP
    if (stopFlag == 1)
    {
        if (errorDiffLeft > -70 && errorDiffLeft < 70)
        {
            OCR1B = 0;
        }
        
        if (errorDiffRight > -70 && errorDiffRight < 70) {
            OCR1A = 0;
        }
        
    }
    
    
//            itoa(OCR1B, temp1, 10);
//            itoa(OCR1A, temp2, 10);
//            uart_puts("OCR1B: ");
//            uart_puts(temp1);
//            uart_puts("        ");
//    
//    
//            uart_puts("OCR1A: ");
//            uart_puts(temp2);
//            uart_puts("\r\n");
}
void stop(void)
{
    stopFlag = 1;
    while(encoderValueLeft != 0 && encoderValueRight != 0)
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            drive(0,0);
        }
    }
    stopFlag = 0;
}
