#include "screen.h"


int servoXPos = SCREEN_SERVO_ORIGIN_X;
int servoYPos = SCREEN_SERVO_ORIGIN_Y;


void CheckScreenServoRange()
{
    if (servoXPos < SCREEN_SERVO_MIN_X) servoXPos = SCREEN_SERVO_MIN_X;
    if (servoXPos > SCREEN_SERVO_MAX_X) servoXPos = SCREEN_SERVO_MAX_X;
    if (servoYPos < SCREEN_SERVO_MIN_Y) servoYPos = SCREEN_SERVO_MIN_Y;
    if (servoYPos > SCREEN_SERVO_MAX_Y) servoYPos = SCREEN_SERVO_MAX_Y;
}

void UpdateScreen()
{
    CheckScreenServoRange();
    SetServoPos(1, servoXPos, 2000);
    SetServoPos(2, 2 * SCREEN_SERVO_ORIGIN_Y - servoYPos, 2000);
    UpdateServoPos();
}


struct BallCoord CalcScreenServoCoord(double x, double y)
{
    struct BallCoord ballCoord;
    double p = sqrt(pow(x, 2) + pow(y, 2) + 1);
    ballCoord.phi = acos(y / p);
    ballCoord.theta = acos(x / (p * sin(ballCoord.phi)));
    return ballCoord;
}

struct ServoPos CalcServoPos(struct BallCoord ballCoord)
{
    struct ServoPos servoPos;
    ballCoord.theta -= PI / 2;
    servoPos.x = SCREEN_SERVO_ORIGIN_X - (ballCoord.theta / SERVO_RADIAN_PRESICION);
    ballCoord.phi = PI / 2 - ballCoord.phi;
    servoPos.y = SCREEN_SERVO_ORIGIN_Y + ballCoord.phi / SERVO_RADIAN_PRESICION;
    return servoPos;
}