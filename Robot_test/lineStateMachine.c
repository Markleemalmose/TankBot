//
//  StateMachine.c
//  Robot_test
//
//  Created by Mark Lee Malmose on 20/03/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//
#include "lineStateMachine.h"
#include "LineSensors.h"
#include "MotorController.h"
#include <util/delay.h>
#include <stdlib.h>

#define FORWARD 0
#define NUDGE_LEFT 1
#define NUDGE_RIGHT 2
#define MEDIUM_LEFT 3
#define MEDIUM_RIGHT 4

void sendUartToInterface(char command, char *message);

volatile int speed = 140;       // max 95

volatile int outLeftSide = 0;
volatile int outRightSide = 0;

volatile int countNoLine = 0;

void lineStateMachine(int newLine, int prevLine)
{
    
    switch (newLine) {
        case 0:
            countNoLine++;
            
            if (prevLine == 1 || outRightSide == 1) {
                outRightSide = 1;
                drive(MEDIUM_LEFT, 10);
                
            } else if (prevLine == 8 || outLeftSide == 1) {
                outLeftSide = 1;
                drive(MEDIUM_RIGHT, 10);
            }
           
            drive(FORWARD, speed);
            break;
            
        case 1:                                 //  1 0 0 0
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            drive(MEDIUM_LEFT, speed);
            break;
            
        case 2:                                 //  0 1 0 0
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            drive(NUDGE_LEFT, speed);
            break;
            
        case 3:                                 //  1 1 0 0
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            drive(NUDGE_LEFT, speed);
            break;
            
        case 4:                                 //  0 0 1 0
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            drive(NUDGE_RIGHT, speed);
            break;
            
        case 5:                                 //  1 0 1 0
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            //stop();
            sendUartToInterface('H', "Case 5: Sensor 1 and 3");
            break;
            
        case 6:                                 //  0 1 1 0
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            drive(FORWARD, speed);
            break;
            
        case 7:                                 //  1 1 1 0
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            //stop();
            sendUartToInterface('H', "Case 7: Sensor 1, 2 and 4");

            break;
            
        case 8:                                 //  0 0 0 1
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            drive(MEDIUM_RIGHT, speed);
            break;
            
        case 9:                                 //  1 0 0 1
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            //stop();
            sendUartToInterface('H', "Case 9: Sensor 1 and 4");

            break;
            
        case 10:                                 //  0 1 0 1
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            //stop();
            sendUartToInterface('H', "Case 10: Sensor 2 and 4");

            break;
            
        case 11:                                 //  1 1 0 1
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            //stop();
            sendUartToInterface('H', "Case 11: Sensor 1, 2 and 4");

            break;
            
        case 12:                                 //  0 0 1 1
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            drive(NUDGE_RIGHT, speed);
            break;
            
        case 13:                                 //  1 0 1 1
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            //stop();
            sendUartToInterface('H', "Case 13: Sensor 1, 3 and 4");

            break;
            
        case 14:                                 //  0 1 1 1
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            //stop();
            sendUartToInterface('H', "Case 14: Sensor 2, 3 and 4");

            break;
            
        case 15:                                 //  1 1 1 1
            countNoLine = 0;
            outLeftSide = 0;
            outRightSide = 0;
            
            //stop();
            break;
            
            
        default:
            sendUartToInterface('H', "HUH?!");
            break;
    }
}

int getCountNoLine(void)
{
    return countNoLine;
}

