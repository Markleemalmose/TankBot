/*
 * MotorController.h
 */


#ifndef MotorController_H_
#define MotorController_H_

#include <stdbool.h>

extern void stop(void);
extern void drive(int direction, int speed);
extern void motor_init(void);
extern void motor_timer_init(void);
extern volatile bool adjustSpeed;
extern volatile uint16_t time1ms;
extern int firstStop;
extern unsigned int encoderValueRight;
extern unsigned int encoderValueLeft;

#endif /* MotorController_H_ */