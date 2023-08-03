#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include "usart.h"

void SetServoPos(uint8_t id, uint16_t pos, uint16_t speed);
uint16_t GetServoPos(uint8_t id);
void UpdateServoPos();


#endif