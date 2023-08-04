#ifndef SCREEN_H
#define SCREEN_H

#include "main.h"
#include "servo.h"
#include "math.h"


#define SCREEN_DISTANCE 1

#define SCREEN_SERVO_ORIGIN_X 546
#define SCREEN_SERVO_ORIGIN_Y 2150

#define SCREEN_SERVO_MIN_X 300
#define SCREEN_SERVO_MAX_X 700
#define SCREEN_SERVO_MIN_Y 1800
#define SCREEN_SERVO_MAX_Y 2500


struct Point
{
    int x;
    int y;
};

struct Point2d
{
    double x;
    double y;
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

struct ScreenPos
{
    double x;
    double y;
};

extern int servoXPos;
extern int servoYPos;

extern struct ScreenPos screen_origin_pos;
extern struct ServoPos screen_servo_origin_pos;
extern struct Point2d cursor_pos;

extern struct Point2d mv_o_pos;
extern struct Point2d mv_a_pos;
extern struct Point2d mv_b_pos;
extern struct Point2d mv_c_pos;
extern struct Point2d mv_d_pos;

extern struct Point mv_o_pos_cali;
extern struct Point mv_a_pos_cali;
extern struct Point mv_b_pos_cali;
extern struct Point mv_c_pos_cali;
extern struct Point mv_d_pos_cali;




void CheckScreenServoRange();
struct BallCoord CalcScreenServoCoord(double x, double y);
struct ServoPos CalcServoPos(struct BallCoord ballCoord);
struct ScreenPos CalcScreenPosFromServoPos(struct ServoPos servoPos);
struct Point2d CalcScreenPosFromOpenMVPos(struct Point point);



void UpdateScreen();

#endif