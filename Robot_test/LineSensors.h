//
//  LineSensors.h
//  Robot_test
//
//  Created by Mark Lee Malmose on 28/02/15.
//  Copyright (c) 2015 Mark Lee Malmose. All rights reserved.
//

#ifndef __Robot_test__LineSensors__
#define __Robot_test__LineSensors__

void LineSensors_init(void);
int convert(void);
void LineSensors_check_line(int sensorNumber);
void LineSensors_check_all(void);
void LineSensors_check_noise(int sensorNumber);
int LineSensors_value(void);

#endif /* defined(__Robot_test__LineSensors__) */
