//
//  ADC.c
//  Robot_test
//
//  Created by Mark Lee Malmose on 16/03/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//

#include "ADC.h"
#include <avr/io.h>

// Initialize adc
void init_ADC(void)
{
    ADCSRA |= (1<<ADPS2);   // Set Prescaler /16
    ADMUX  |= (1<<REFS0);     // Voltage reference Avcc
    ADCSRA |= 1<<ADEN;      // Enable ADC
    ADCSRA |= 1<<ADSC;      // Start conversion
    
}