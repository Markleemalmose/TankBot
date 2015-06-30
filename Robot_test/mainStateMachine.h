//
//  mainStateMachine.h
//  Robot_test
//
//  Created by Mark Lee Malmose on 28/03/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//

#ifndef __Robot_test__mainStateMachine__
#define __Robot_test__mainStateMachine__

#include <stdio.h>

void mainState(void);
void driveDistanceEncoder(int distanceInEncoderValues);
void stopTimeMs(uint16_t ms);
void turn90degreesEncoder(int direction);
void turnDegreesGyro(int degrees, int leftright);

#endif /* defined(__Robot_test__mainStateMachine__) */
