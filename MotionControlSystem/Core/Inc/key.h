#ifndef KEY_H
#define KEY_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

extern int8_t bitMode;
extern int8_t bit0;
extern int8_t bit1;


void ReadKeyBit();

int IsModeChange();


#endif