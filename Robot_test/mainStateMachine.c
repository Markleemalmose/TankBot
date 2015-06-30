//
//  mainStateMachine.c
//  Robot_test
//
//  Created by Mark Lee Malmose on 28/03/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//

#include <stdlib.h>
#include "UART.h"
#include "UART1.h"
#include "lineStateMachine.h"
#include "mainStateMachine.h"
#include "MotorController.h"
#include "LineSensors.h"
#include "HC-SR04.h"
#include <util/delay.h>

#define FORWARD 0
#define NUDGE_LEFT 1
#define NUDGE_RIGHT 2
#define MEDIUM_LEFT 3
#define MEDIUM_RIGHT 4
#define HARD_LEFT 5
#define HARD_RIGHT 6
#define TURN_LEFT 7
#define TURN_RIGHT 8
#define TURN_LEFT_TO_LINE 9
#define TURN_RIGHT_TO_LINE 10
#define CIRCLE_LEFT 11
#define CIRCLE_RIGHT 12
#define SOFT_LEFT 13

#define LEFT 0
#define RIGHT 1

#define FRONT_SENSOR 0
#define SIDE_SENSOR 1

// Global variables
extern int speed;
extern int direction;

volatile int linecount = 0;
char noLineChar[4];

// Sensor variables
volatile int newLine = 0;
volatile int prevLine = 0;
char lineToChar[2];

volatile int newDistanceFront = 0;
volatile int frontDistance = 0;
volatile int prevFrontDistance = 0;
char DistanceFrontToChar[2];

volatile int newDistanceSide = 0;
volatile int sideDistance = 0;
volatile int prevSideDistance = 0;
char DistanceSideToChar[2];

volatile int tempDegrees = 0;

// Flags
volatile int lineFoundFlag = 0;
volatile int fullCircleDone = 0;
volatile int highSpeedMark = 0;
volatile int startDistance = 0;

// Statemachine variables
volatile int wallStateMessage = 0;
volatile int stateMessage = 0;

enum states
{
    FIND_LINE,
    BEFORE_LINE_DISRUPTION,
    LINE_DISRUPTION,
    FULL_CIRCLE,
    HIGH_SPEED_CURVE,
    END_OF_LINE,
    WALL_CORNERING,
    FINAL_STOP
};

enum wall_states
{
    FIND_WALL,
    FIRST,
    SECOND,
    THIRD,
    FOURTH,
    STOP
};

// Initial states
enum states state = FIND_LINE;
enum wall_states wall_state = FIND_WALL;




/* ************************************************
 *
 * Function to send information to the PC interface
 *
 * @param command
 * @param message
 *
 **************************************************/
void sendUartToInterface(char command, char *message)
{
    uart_putc(command);
    uart_puts(message);
    uart_puts("\r\n");
}

/* ************************************************
 *
 * Function to send Linesensor value to PC interface
 *
 **************************************************/
void sendDistanceToInterface(int sensor)
{
    if (sensor == 0)
    {
        newDistanceFront = HCSR04_get_distance_cm(sensor);

        frontDistance = newDistanceFront;
        
        if (frontDistance != prevFrontDistance)
        {
            itoa(frontDistance, DistanceFrontToChar, 10);
            
            sendUartToInterface('I', DistanceFrontToChar);
        }
        prevFrontDistance = frontDistance;
    }
    else
    {
        
        newDistanceSide = HCSR04_get_distance_cm(sensor);
         
        sideDistance = newDistanceSide;

        if (sideDistance != prevSideDistance)
        {
            itoa(sideDistance, DistanceFrontToChar, 10);
        
            sendUartToInterface('J', DistanceFrontToChar);
        }
        prevSideDistance = sideDistance;

     }
}

/* ************************************************
 *
 * Function to send Linesensor value to PC interface
 *
 **************************************************/
void sendLineToInterface(void)
{
    newLine = LineSensors_value();
    
    if (newLine != prevLine) {
        itoa(newLine, lineToChar, 10);
        sendUartToInterface('F', lineToChar);
    }
}

/* ************************************************
 *
 * Function to stop robot for specified time in ms
 *
 * @param ms
 *
 **************************************************/
void stopTimeMs(uint16_t ms)
{
    time1ms = 0;
    
    while (time1ms <= ms) {
        stop();
    }
}

/* ************************************************
 *
 * Function to follow the line by checking line
 * sensors
 *
 *
 **************************************************/
void followLine(void)
{
    if (adjustSpeed)
    {
        adjustSpeed = false;
        LineSensors_check_all();
        sendLineToInterface();
        lineStateMachine(newLine, prevLine);
        prevLine = newLine;
    }
}

void driveDistanceEncoder(int distanceInEncoderValues)
{
    // Wheel diameter 37,78 mm. 1032 encoder values pr. revolution
    // distance in encoder values. 0.115 mm pr. encoder value. 100 mm = 869 encoder values

    int encoderCount = 0;
    
    while (encoderCount <= distanceInEncoderValues )
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            encoderCount += (encoderValueRight + encoderValueLeft)/2;
            drive(FORWARD, speed);
        }
    }
}

void driveDistanceInCM(int cm)
{
    int encoderValue = (cm*10)/0.115;
    
    driveDistanceEncoder(encoderValue);
}

void turnDegreesGyro(int degrees, int leftOrRight)
{
    UCSR1B |= (1<<RXCIE1);  // Enable uart1 interrupts
    
    // direction = 0 -> LEFT
    // direction = 1 -> RIGHT
    if (!leftOrRight )
    {
        sendUartToInterface('H', "Turning left some degrees");
    }
    else
    {
        sendUartToInterface('H', "Turning right some degrees");
    }
    
    if (!leftOrRight)   // LEFT
    {
        tempDegrees = direction - degrees;
        
        while (direction >= tempDegrees)
        {
            if (adjustSpeed)
            {
                adjustSpeed = false;
                drive(TURN_LEFT_TO_LINE, speed);
            }
        }
    }
    else   // RIGHT
    {
        tempDegrees = direction + degrees;
        
        while (direction <= tempDegrees)
        {
            if (adjustSpeed)
            {
                adjustSpeed = false;
                drive(TURN_RIGHT_TO_LINE, speed);
            }
        }
    }
    
    if (!leftOrRight )
    {
        sendUartToInterface('H', "Done turning left some degrees");
    }
    else
    {
        sendUartToInterface('H', "Done turning right some degrees");
    }
    
    UCSR1B &= ~(1<<RXCIE1);  // Disable uart1 interrupts
}

void turn90degreesEncoder(int direction)
{
    // direction = 0 -> LEFT
    // direction = 1 -> RIGHT
    if (!direction ) {
        sendUartToInterface('H', "Turning left 90 degrees with encoder values");
    }
    else
    {
        sendUartToInterface('H', "Turning right 90 degrees with encoder values");
    }
    
    int countTurn = 0;
    
    while (countTurn <= 3000) // 1,5 * 1032      2760
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            countTurn += encoderValueLeft;
            
            if (!direction) {
                drive(TURN_LEFT_TO_LINE, speed);
            }
            else
            {
                drive(TURN_RIGHT_TO_LINE, speed);
            }
        }
    }
    
    if (!direction) {
        sendUartToInterface('H', "Done Turning left 90 degrees with encoder values");
    }
    else
    {
        sendUartToInterface('H', "Done Turning right 90 degrees with encoder values");
    }
}

void turnSomeDegreesEncoder(int direction, int degrees)
{
    // direction = 0 -> LEFT
    // direction = 1 -> RIGHT
    if (!direction ) {
        sendUartToInterface('H', "Turning left some degrees with encoder values");
    }
    else
    {
        sendUartToInterface('H', "Turning right some degrees with encoder values");
    }
    
    int countTurn = 0;
    
    while (countTurn <= degrees) // 1,5 * 1032      2760
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            countTurn += encoderValueLeft;
            
            if (!direction) {
                drive(TURN_LEFT_TO_LINE, speed);
            }
            else
            {
                drive(TURN_RIGHT_TO_LINE, speed);
            }
        }
    }
    
    if (!direction) {
        sendUartToInterface('H', "Done Turning left some degrees with encoder values");
    }
    else
    {
        sendUartToInterface('H', "Done Turning right some degrees with encoder values");
    }
}

void turnToLine(int direction)
{
    // direction = 0 -> LEFT
    // direction = 1 -> RIGHT
    
    if (!direction) {
        sendUartToInterface('H', "Turning left to line");
    }
    else
    {
        sendUartToInterface('H', "Turning right to line");
    }
    
    // drive forward 8cm 80/0.115 = 435
    //driveDistanceEncoder(435);
    
    while (true)
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            
            LineSensors_check_all();
            sendLineToInterface();
            
            if (!direction) {
                if (newLine == 0 && prevLine == 2)
                {
                    sendUartToInterface('H', "Done Turning left to line");
                    break;
                }
                else
                {
                    drive(TURN_LEFT_TO_LINE, speed);
                }
            }
            else
            {
                if (newLine == 0 && prevLine == 4)
                {
                    sendUartToInterface('H', "Done Turning right to line");
                    break;
                }
                else
                {
                    drive(TURN_RIGHT_TO_LINE, speed);
                }
            }
            prevLine = newLine;
        }
    }
}

void findLine(void)
{
    while (!adjustSpeed);
    if (adjustSpeed)
    {
        adjustSpeed = false;
        LineSensors_check_all();
        sendLineToInterface();

        if (newLine == 15 || newLine == 7 || newLine == 14)
        {
            sendUartToInterface('H', "Line found");
            lineFoundFlag = 1;
        }
        else
        {
            drive(FORWARD, speed); // Straight outta Compton
        }
        prevLine = newLine;
    }
}

void octagon(void)
{
    int flag = 0;
    
    sendUartToInterface('H', "Octagon");
    
    driveDistanceEncoder(5087);
    
    // turn 90 degrees right
    turn90degreesEncoder(RIGHT);
    
    driveDistanceInCM(35);
    stopTimeMs(300);
    
    turnSomeDegreesEncoder(RIGHT, 1000);
    stopTimeMs(300);
    
    driveDistanceInCM(35);
    stopTimeMs(300);
    
    turnSomeDegreesEncoder(RIGHT, 1000);
    stopTimeMs(300);
    
    driveDistanceInCM(50);
    stopTimeMs(300);
    
    turnSomeDegreesEncoder(RIGHT, 1000);
    stopTimeMs(300);
    
    driveDistanceInCM(36);
    stopTimeMs(300);
    
    turnSomeDegreesEncoder(RIGHT, 1000);
    stopTimeMs(300);
    
    
    
    while (!flag)
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            LineSensors_check_all();
            sendLineToInterface();
            
            if (newLine == 15 || newLine == 7 || newLine == 14)
            {
                
                if (newLine == 7)
                {
//                    driveDistanceInCM(1);
                    
                    while (!(newLine == 15))
                    {
                        if (adjustSpeed)
                        {
                            adjustSpeed = false;
                            LineSensors_check_all();
                            sendLineToInterface();
                            drive(TURN_LEFT_TO_LINE, speed);
                        }
                    }
                    
                }
                
                if (newLine == 14)
                {
//                    driveDistanceInCM(1);
                    
                    while (!(newLine == 15))
                    {
                        if (adjustSpeed)
                        {
                            adjustSpeed = false;
                            LineSensors_check_all();
                            sendLineToInterface();
                            drive(TURN_RIGHT_TO_LINE, speed);
                        }
                    }
                    
                }
                
                sendUartToInterface('H', "Line found");
                flag = 1;
            }
            else
            {
                drive(FORWARD, speed); // Straight outta Compton
            }
            prevLine = newLine;
        }
    }

    flag = 0;
    
    driveDistanceInCM(35);
    stopTimeMs(300);
    
    turnSomeDegreesEncoder(RIGHT, 1000);
    stopTimeMs(300);
    
    driveDistanceInCM(36);
    stopTimeMs(300);

    turnSomeDegreesEncoder(RIGHT, 1000);
    stopTimeMs(300);
    
    driveDistanceInCM(50);
    stopTimeMs(300);
    
    turnSomeDegreesEncoder(RIGHT, 1000);
    stopTimeMs(300);
    
    driveDistanceInCM(36);
    stopTimeMs(300);
    
    turnSomeDegreesEncoder(RIGHT, 1000);
    stopTimeMs(300);
    
//    driveDistanceInCM(25);
//    stopTimeMs(300);
    
    
    while (!flag)
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            LineSensors_check_all();
            sendLineToInterface();
        
            if (newLine == 15 || newLine == 7 || newLine == 14)
            {
                sendUartToInterface('H', "Line found");
                flag = 1;
            }
            else
            {
                drive(FORWARD, speed); // Straight outta Compton
            }
            prevLine = newLine;
        }
    }
    
    turnToLine(LEFT);

//    stopTimeMs(500);
//    
//    uart1_puts("W\r\n");    // send turret to 90 degrees
//    
//    stopTimeMs(4000);

    
    fullCircleDone = 1;

}

void fullCircle(void)
{
    sendUartToInterface('H', "Full circle function");
    volatile int lineCount = 0;

    // drive forward 58.5 cm 585/0.115 = 5087
    driveDistanceEncoder(5087);
//        driveDistanceEncoder(15087);
    
    LineSensors_check_all();
    
    // turn 90 degrees right
    turn90degreesEncoder(RIGHT);
    
    newLine = 0;
    
    driveDistanceEncoder(435); // 5 cm
        
    while (1)
    {
        
        while (!adjustSpeed);
        if (adjustSpeed)
        {
            adjustSpeed = false;
            LineSensors_check_all();
            sendLineToInterface();
            drive(12, 160); // 70
        }
        if ((newLine == 15 || newLine == 14 || newLine == 12 || newLine == 8) && lineCount == 0)
        {
            sendUartToInterface('H', "linecount++");
            driveDistanceEncoder(500); // 5 cm
            newLine = 0;
            lineCount = 1;
        }
        else if ((newLine == 15 || newLine == 14 || newLine == 12 || newLine == 8) && lineCount == 1)
        {
            driveDistanceEncoder(435); // 5 cm
            sendUartToInterface('H', "break");
            break;
        }
        
        prevLine = newLine;
    }

    sendUartToInterface('H', "Full circle done");
    turnToLine(LEFT);
    
    fullCircleDone = 1;
}

void followWall(int distanceToWall)
{
    sendDistanceToInterface(SIDE_SENSOR);
    
    if (sideDistance < distanceToWall)
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            drive(NUDGE_RIGHT, speed);
            sendDistanceToInterface(SIDE_SENSOR);
        }
    }
    else if (sideDistance > distanceToWall)
    {
        if (adjustSpeed)
        {
            adjustSpeed = false;
            drive(NUDGE_LEFT, speed);
            sendDistanceToInterface(SIDE_SENSOR);
        }
    }
    else
    {
        while (!adjustSpeed);
        adjustSpeed = false;
        drive(FORWARD, speed);
    }
}

void followWallStateMachine(void)
{
    speed = 120;
    
    switch (wall_state)
    {
        case FIND_WALL:
            if (wallStateMessage == 0)
            {
                wallStateMessage = 1;
            }
            
            driveDistanceInCM(102);
            stopTimeMs(500);
            
            // turn 90 degrees right
            turnDegreesGyro(3, RIGHT);
            
            
            stopTimeMs(500);
            
            wallStateMessage = 0;
            wall_state = FIRST;
            
        case FIRST:
            if (wallStateMessage == 0)
            {
                sendUartToInterface('C', "SW2");
                wallStateMessage = 1;
            }
            
            driveDistanceInCM(60);
            
            stopTimeMs(500);
            
            // turn 90 degrees left
            turnDegreesGyro(23, LEFT);
            //            turn90degreesEncoder(LEFT);
            
            wallStateMessage = 0;
            wall_state = SECOND;
            
            //            break;
            
        case SECOND:
            if (wallStateMessage == 0)
            {
                sendUartToInterface('C', "SW3");
                wallStateMessage = 1;
            }
            
            
            driveDistanceInCM(80);
            
            stopTimeMs(500);
            
            // turn 90 degrees left
            turnDegreesGyro(50, LEFT);
            //            turn90degreesEncoder(LEFT);
            
            wallStateMessage = 0;
            wall_state = THIRD;
            
            //            break;
            
        case THIRD:
            if (wallStateMessage == 0)
            {
                sendUartToInterface('C', "SW4");
                wallStateMessage = 1;
            }
            
            // drive at bit forward to align to wall
            driveDistanceInCM(65);
            
            
            stopTimeMs(500);
            
            // turn 90 degrees right
            //            turnDegreesGyro(0, RIGHT);
            turnSomeDegreesEncoder(RIGHT, 2500);
            //            turn90degreesEncoder(RIGHT);
            
            wallStateMessage = 0;
            wall_state = FOURTH;
            
            //            break;
            
        case FOURTH:
            if (wallStateMessage == 0)
            {
                sendUartToInterface('C', "SW5");
                wallStateMessage = 1;
            }
            
            driveDistanceInCM(70);
            
            stopTimeMs(500);
            
            wallStateMessage = 0;
            wall_state = STOP;
            
            //            break;
            
        case STOP:
            if (wallStateMessage == 0)
            {
                sendUartToInterface('C', "SW6");
                wallStateMessage = 1;
            }
            
            // the end
            stop();
            wallStateMessage = 0;
            
            //            break;
            
        default:
            break;
    }
}


void mainState(void)
{
    
    switch (state)
    {
        case FIND_LINE:
            if (stateMessage == 0)
            {
                sendUartToInterface('C', "S1");
                stateMessage = 1;
            }
            
            if (startDistance == 0) {
                driveDistanceInCM(20);
                startDistance = 1;
            }
            
            findLine();
            
            if (lineFoundFlag)
            {
                stopTimeMs(500);
                turnToLine(RIGHT);
                stopTimeMs(500);
                
                stateMessage = 0;
                state = BEFORE_LINE_DISRUPTION;
            }
            lineFoundFlag = 0;

            break;
        
        case BEFORE_LINE_DISRUPTION:
            if (stateMessage == 0)
            {
                sendUartToInterface('C', "S2");
                stateMessage = 1;
            }

            followLine();
            
            if (newLine == 1 || newLine == 3) {
                // drive forward 4cm 40/0.115 = 348
                driveDistanceEncoder(348);
                while(!adjustSpeed);
                followLine();
                stateMessage = 0;
                state = LINE_DISRUPTION;
            }
            
            break;
            
        case LINE_DISRUPTION:
            if (stateMessage == 0)
            {
                sendUartToInterface('C', "S3");
                stateMessage = 1;
            }
            
            
            
            linecount = getCountNoLine();
            itoa(linecount, noLineChar, 10);
            uart_puts(noLineChar);
            uart_puts("\r\n");

            followLine();
            
            if (linecount >= 20)
            {
                uart_puts("Off line\r\n");
                drive(SOFT_LEFT, speed); // drive left
                linecount = 0;
            }
            
            if (newLine == 1 || newLine == 3)
            {
                uart_puts("found line\r\n");
                followLine();
                linecount = 0;
                stateMessage = 0;
                state = FULL_CIRCLE;
            }
            

            
            break;
        
            
        case FULL_CIRCLE:
            if (stateMessage == 0)
            {
                sendUartToInterface('C', "S4");
                stateMessage = 1;
            }
            
            followLine();
            
            if (newLine == 15 || newLine == 7 || newLine == 14)
            {
                stopTimeMs(1000);
                octagon();
                //                fullCircle();
            }
            
            if (fullCircleDone)
            {
                stateMessage = 0;
                state = HIGH_SPEED_CURVE;
            }
            
            break;
            
        
        case HIGH_SPEED_CURVE:
            
            if (stateMessage == 0)
            {
                sendUartToInterface('C', "S5");
                stateMessage = 1;
            }
            
            followLine();
            
            if (newLine == 3 || newLine == 7)
            {
                
                if (!highSpeedMark)
                {
                    sendUartToInterface('C', "S5.1");
                    driveDistanceEncoder(435);
                    speed = 180;
                    highSpeedMark = 1;
                }

            }
            else if (newLine == 15 || newLine == 7 || newLine == 14)
            {
                stateMessage = 0;
                state = END_OF_LINE;
            }
            
            break;
            
        
        case END_OF_LINE:
            
            if (stateMessage == 0)
            {
                sendUartToInterface('C', "S6");
                stateMessage = 1;
            }
            
            
            // Reset Atmega328p
            DDRD |= (1<<PD4);
            PORTD &= ~(1<<PD4);
            _delay_us(5);
            PORTD |= (1<<PD4);
            stopTimeMs(1000);
            
            uart1_puts("W\r\n");    // send turret to 90 degrees
            
            stopTimeMs(2000);
            
            stateMessage = 0;
            state = WALL_CORNERING;
            
            break;
        
            
        case WALL_CORNERING:
            if (stateMessage == 0)
            {
                sendUartToInterface('C', "S7");
                stateMessage = 1;
            }
            
            followWallStateMachine();
            
            stateMessage = 0;
            state = FINAL_STOP;
            
            break;
            
        case FINAL_STOP:
            if (stateMessage == 0)
            {
//                sendUartToInterface('C', "S7");
                stateMessage = 1;
            }
            
            stop();
            stateMessage = 0;
            
        default:
            break;
    }
    
    
}
