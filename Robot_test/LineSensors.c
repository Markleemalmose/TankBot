//
//  LineSensors.c
//  Robot_test
//
//  Created by Mark Lee Malmose on 28/02/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>     // itoa for debugging
#include "LineSensors.h"
//#include "Globals.h"
#include "ADC.h"
#include <util/delay.h>
#include <stdio.h>

void uart_puts(char *s);    // For debugging
void uart_putc(unsigned char c);
void LCD_write_english_string(unsigned char X,unsigned char Y,char *s);

int writeToTerminal = 0;

// SENSOR
#define SENSOR_1_THRESHOLD 450
#define SENSOR_2_THRESHOLD 600
#define SENSOR_3_THRESHOLD 450
#define SENSOR_4_THRESHOLD 450

#define MCU_DDR DDRC
#define MCU_PIN PC6
#define MCU_PORT PORTC



void LineSensors_init(void)
{
    MCU_DDR = (1<<MCU_PIN);     // Set mcu pin as output
    MCU_PORT &= ~(1<<MCU_PIN);  // Set mcu pin low
}


int convert(void)
{
    ADCSRA |= (1<<ADSC);  // Start conversion
    
    while (ADCSRA &  (1<<ADIF));
    // wait until conversion  completes; ADSC=0 means Complete
    
    uint8_t lowADCL = ADCL;
    uint16_t adcResult = ADCH<<8 | lowADCL;
    
    return adcResult;
}


char adc1[4];
char adc2[4];
char adc3[4];
char adc4[4];

volatile uint8_t binary = 0;

void LineSensors_check_line(int sensorNumber)
{
    uint16_t adcResult = 0;
    
    switch (sensorNumber) {
        case 1:
            ADMUX = 0x41;
            
            adcResult = convert();
            
            if (adcResult>SENSOR_1_THRESHOLD) {
                binary |= 0x01;
            } else {
                binary &= ~0x01;
            }
            
            if (writeToTerminal)
            {
                itoa(adcResult, adc1, 10);
                uart_puts(adc1);
                uart_puts("    ");
            }
            
            break;
            
        case 2:
            ADMUX = 0x42;
            
            adcResult = convert();
            
            if (adcResult>SENSOR_2_THRESHOLD) {
                binary |= 0x02;
            } else {
                binary &= ~0x02;
            }
            
            if (writeToTerminal)
            {
                itoa(adcResult, adc2, 10);
                uart_puts(adc2);
                uart_puts("    ");
            }

            break;
            
        case 3:
            ADMUX = 0x43;
            
            adcResult = convert();
            
            if (adcResult>SENSOR_3_THRESHOLD) {
                binary |= 0x04;
            } else {
                binary &= ~0x04;
            }
            
            if (writeToTerminal)
            {
                itoa(adcResult, adc3, 10);
                uart_puts(adc3);
                uart_puts("    ");
            }
            
            break;
            
        case 4:
            ADMUX = 0x40;
            
            
            adcResult = convert();
            
            if (adcResult>SENSOR_4_THRESHOLD) {
                binary |= 0x08;
            } else {
                binary &= ~0x08;
            }
            
            if (writeToTerminal)
            {
                itoa(adcResult, adc4, 10);
                uart_puts(adc4);
                uart_puts("\r\n");
            }
            
            break;
            
        default:
            break;
    }
    _delay_us(15);
    
}

void LineSensors_check_all(void)
{
    MCU_PORT |= (1<<MCU_PIN);   // Set mcu pin high
    _delay_us(200);
    LineSensors_check_line(1);
    LineSensors_check_line(2);
    LineSensors_check_line(3);
    LineSensors_check_line(4);
    //_delay_us(1000);
    MCU_PORT &= ~(1<<MCU_PIN);  // Set mcu pin low

    
}


int LineSensors_value(void)
{
    return binary;
}


