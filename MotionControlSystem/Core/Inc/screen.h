#ifndef SCREEN_H
#define SCREEN_H

#include "main.h"
#include "servo.h"

extern int servoXPos;
extern int servoYPos;

struct Point
{
    int x;
    int y;
};

extern struct Point screenAPoint;
extern struct Point screenCPoint;


extern int screenWidth;
extern int screenHeight;
extern int screenWidthHalf;
extern int screenHeightHalf;


void UpdateScreen();
void CalcScreenSize();

#endif