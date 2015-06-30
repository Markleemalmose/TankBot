//
//  hc-sr04.c
//  Robot_test
//
//  Created by Mark Lee Malmose on 22/02/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//
#include <avr/io.h>
#include <avr/interrupt.h>
#include "HC-SR04.h"

#include <stdlib.h>     // itoa for debugging


// For debugging
void uart_puts(char *s);

// Define ports and pins

// Front 2
#define HC_SR04_FRONT_TRIG_PORT PORTG
#define HC_SR04_FRONT_TRIG_DDR  DDRG
#define	HC_SR04_FRONT_ECHO_PIN  PINB
#define HC_SR04_FRONT_ECHO_DDR  DDRB
#define HC_SR04_FRONT_TRIG	PG4    // Trig pin
#define HC_SR04_FRONT_ECHO	PB4     // Echo pin


// Side 1
#define HC_SR04_SIDE_TRIG_PORT PORTG
#define HC_SR04_SIDE_TRIG_DDR  DDRG
#define	HC_SR04_SIDE_ECHO_PIN  PINB
#define HC_SR04_SIDE_ECHO_DDR  DDRB
#define HC_SR04_SIDE_TRIG	PG3    // Trig pin
#define HC_SR04_SIDE_ECHO	PB7     // Echo pin




void HCSR04_init(void)
{
    HC_SR04_FRONT_TRIG_DDR |= (1 << HC_SR04_FRONT_TRIG);     //output - Trig
    HC_SR04_FRONT_ECHO_DDR &= ~(1 << HC_SR04_FRONT_ECHO);	//input - Echo
    
    HC_SR04_SIDE_TRIG_DDR |= (1 << HC_SR04_SIDE_TRIG);     //output - Trig
    HC_SR04_SIDE_ECHO_DDR &= ~(1 << HC_SR04_SIDE_ECHO);	//input - Echo
}

// microseconds counter
volatile uint16_t microsec = 0;

void timer_start(void)
{
    
    // start 16-bit timer Timer3
    //cli();
    
    // set up timer with no prescaler and CTC mode
    TCCR3B |= (1 << WGM32)|(1 << CS30);
    
    // initialize counter
    TCNT3 = 0;
    
    // initialize compare value 1 pulse is 58 microseconds
    OCR3A = 1068; // Should be 58us/0,05425347 - 1 = 1068, but measured on scope to ??891
    
    // enable compare interrupt
    TIMSK3 |= (1 << OCIE3A);
    
    // enable global interrupts
    //sei();
    
}

void timer_stop(void)
{
    TCCR3B = 0x0;
}


void send_pulse(int sensor)
{
    // 0 = front sensor
    // 1 = side sensor
    
    // send trigger pulse
    DDRA |= (1<<PA0);
    PORTA |= (1<<PA0);
    
    
    // Pin low
    microsec = 0;
    timer_start();
    
    while (microsec <= 1){
        
        if (sensor == 0) {
            HC_SR04_FRONT_TRIG_PORT &= ~(1 << HC_SR04_FRONT_TRIG);   // Pulldown trigger FRONT
        }
        else
        {
            HC_SR04_SIDE_TRIG_PORT &= ~(1 << HC_SR04_SIDE_TRIG);   // Pulldown trigger SIDE
        }
        
    }
    timer_stop();
    
    
    
    // Pin high
    microsec = 0;
    timer_start();
    while (microsec <= 1){
        
        if (sensor == 0) {
            HC_SR04_FRONT_TRIG_PORT |= (1 << HC_SR04_FRONT_TRIG);    // Pullup trigger FRONT
        }
        else
        {
            HC_SR04_SIDE_TRIG_PORT |= (1 << HC_SR04_SIDE_TRIG);    // Pullup trigger SIDE
        }
        
    }
    timer_stop();
    
    
    // Pin low
    microsec = 0;
    timer_start();
    while (microsec <= 1)
    {
        if (sensor == 0) {
            HC_SR04_FRONT_TRIG_PORT &= ~(1 << HC_SR04_FRONT_TRIG);   // Pulldown trigger FRONT
        }
        else
        {
            HC_SR04_SIDE_TRIG_PORT &= ~(1 << HC_SR04_SIDE_TRIG);   // Pulldown trigger SIDE
        }
    }
    timer_stop();
    microsec = 0;
    
    PORTA &= ~(1<<PA0);
}

// 1/0 flag to check if echo is over
volatile char echoDone = 0;

// Length of echo pulse in microseconds
volatile uint8_t result = 0;

uint16_t HCSR04_get_distance_cm(int sensor)
{
    // 0 = front sensor
    // 1 = side sensor
    
    uint8_t distance;
    
    // Enable pin change interrupt
    cli();
    PCICR |= (1 << PCIE0);
    
    if (sensor == 0) {
        PCMSK0 |= (1 << PCINT4);
    }
    else
    {
        PCMSK0 |= (1 << PCINT7);
    }
    
    sei();
    
    // set echo flag
    echoDone = 0;
    
    // send pulse
    send_pulse(sensor);
    
    // loop till echo pin goes low
    while(!echoDone);
    
    // disable pin-change interrupt:
    // disable interrupt
    PCICR &= ~(1 << PCIE0);
    // disable pin
    if (sensor == 0) {
        PCMSK0 &= ~(1 << PCINT4);
    }
    else
    {
        PCMSK0 &= ~(1 << PCINT7);
    }
    
    
    // calculate distance in cm
    distance=result;
    
    return distance;
}


ISR(TIMER3_COMPA_vect)
{
    microsec++;
    TCNT3 = 0x00;
    
    //    DDRA |= (1<<PA0);
    //    PORTA ^= (1<<PA0);
}

// pin-change interrupt handler
ISR(PCINT0_vect)
{
    // read PCINT0
    if((HC_SR04_FRONT_ECHO_PIN & (1 << HC_SR04_FRONT_ECHO)) || (HC_SR04_SIDE_ECHO_PIN & (1 << HC_SR04_SIDE_ECHO))) {
        // rising edge:
        
        // start 8-bit timer
        timer_start();
        
    }
    else {
        // falling edge
        
        
        timer_stop();       // stop timer
        
        result=microsec;    // Save result
        
        microsec=0;         // Reset timer value
        
        echoDone = 1;       // set flag
    }
}