#ifndef SCREEN_H
#define SCREEN_H

#include "main.h"
#include "servo.h"
#include "math.h"


#define SCREEN_SERVO_ORIGIN_X 512
#define SCREEN_SERVO_ORIGIN_Y 512

#define SCREEN_SERVO_MIN_X 300
#define SCREEN_SERVO_MAX_X 700
#define SCREEN_SERVO_MIN_Y 300
#define SCREEN_SERVO_MAX_Y 700



extern int servoXPos;
extern int servoYPos;

struct Point
{
    int x;
    int y;
};

struct BallCoord
{
    double theta;
    double phi;
};

struct ServoPos
{
    double x;
    double y;
};


struct BallCoord CalcScreenServoCoord(double x, double y);
struct ServoPos CalcServoPos(struct BallCoord ballCoord);



void UpdateScreen();

#endif