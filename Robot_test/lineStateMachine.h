//
//  StateMachine.h
//  Robot_test
//
//  Created by Mark Lee Malmose on 20/03/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//

#ifndef __Robot_test__StateMachine__
#define __Robot_test__StateMachine__

#include <stdio.h>


void lineStateMachine(int newLine, int prevLine);
int getCountNoLine(void);
volatile int newDirection;
extern volatile int outLeftSide;
extern volatile int outRightSide;




#endif /* defined(__Robot_test__StateMachine__) */
