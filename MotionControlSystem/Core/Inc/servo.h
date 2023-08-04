#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include "usart.h"

#define PI 3.141592653589793
#define SERVO1_RADIAN_PRESICION (PI/180.0*220.0/1024.0)
#define SERVO2_RADIAN_PRESICION (PI/180.0*360.0/4096.0)


void SetServoPos(uint8_t id, uint16_t pos, uint16_t speed);
uint16_t GetServoPos(uint8_t id);
void UpdateServoPos();


#endif