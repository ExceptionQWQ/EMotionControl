#include "screen.h"


int servoXPos = SCREEN_SERVO_ORIGIN_X;
int servoYPos = SCREEN_SERVO_ORIGIN_Y;

struct ScreenPos screen_origin_pos = {0, 0};
struct ServoPos screen_servo_origin_pos = {SCREEN_SERVO_ORIGIN_X, SCREEN_SERVO_ORIGIN_Y};
struct Point2d cursor_pos = {0, 0};


struct Point2d mv_o_pos = {0.00001, 0.00001};
struct Point2d mv_a_pos = {-0.25, 0.25};
struct Point2d mv_b_pos = {0.25, 0.25};
struct Point2d mv_c_pos = {0.25, -0.25};
struct Point2d mv_d_pos = {-0.25, -0.25};

struct Point mv_o_pos_cali;
struct Point mv_a_pos_cali;
struct Point mv_b_pos_cali;
struct Point mv_c_pos_cali;
struct Point mv_d_pos_cali;

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
    SetServoPos(1, servoXPos, 500);
    SetServoPos(2, 2 * SCREEN_SERVO_ORIGIN_Y - servoYPos, 500);
    UpdateServoPos();
}


struct BallCoord CalcScreenServoCoord(double x, double y)
{
    struct BallCoord ballCoord;
    double p = sqrt(pow(x, 2) + pow(y, 2) + pow(SCREEN_DISTANCE, 2));
    ballCoord.phi = acos(y / p);
    ballCoord.theta = acos(x / (p * sin(ballCoord.phi)));
    return ballCoord;
}

struct ServoPos CalcServoPos(struct BallCoord ballCoord)
{
    struct ServoPos servoPos;
    ballCoord.theta -= PI / 2;
    servoPos.x = SCREEN_SERVO_ORIGIN_X - (ballCoord.theta / SERVO1_RADIAN_PRESICION);
    ballCoord.phi = PI / 2 - ballCoord.phi;
    servoPos.y = SCREEN_SERVO_ORIGIN_Y + ballCoord.phi / SERVO2_RADIAN_PRESICION;
    return servoPos;
}

struct ScreenPos CalcScreenPosFromServoPos(struct ServoPos servoPos)
{
    double theta = (SCREEN_SERVO_ORIGIN_X - servoPos.x) * SERVO1_RADIAN_PRESICION + PI / 2;
    double phi = PI / 2 - (servoPos.y - SCREEN_SERVO_ORIGIN_Y) * SERVO2_RADIAN_PRESICION;
    struct ScreenPos screenPos;
    screenPos.x = SCREEN_DISTANCE * cos(theta) / sin(theta);
    screenPos.y = SCREEN_DISTANCE * cos(phi) / (sin(theta) * sin(phi));
    return screenPos;
}

struct Point2d CalcScreenPosFromOpenMVPos(struct Point point)
{
    struct Point2d screenPos;

    double dx = (mv_c_pos.x - mv_o_pos.x) / (double)(mv_c_pos_cali.x - mv_o_pos_cali.x);
    double dy = (mv_c_pos.y - mv_o_pos.y) / (double)(mv_c_pos_cali.y - mv_o_pos_cali.y);

    screenPos.x = mv_o_pos.x + (point.x - mv_o_pos_cali.x) * dx;
    screenPos.y = mv_o_pos.y + (point.y - mv_o_pos_cali.y) * dy;

    return screenPos;
}