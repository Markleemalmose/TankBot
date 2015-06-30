//
//  hc-sr04.h
//  Robot_test
//
//  Created by Mark Lee Malmose on 22/02/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//

#ifndef HCSR04_H_
#define HCSR04_H_

void HCSR04_init(void);

void send_pulse(int sensor);

uint16_t HCSR04_get_distance_cm(int sensor);

#endif /* defined(HCSR04_H_) */
