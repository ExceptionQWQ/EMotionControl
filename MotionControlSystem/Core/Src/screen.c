#include "screen.h"


int servoXPos = 512;
int servoYPos = 512;

struct Point screenAPoint = { 400, 600};
struct Point screenCPoint = {512, 512};

int screenWidth = 0;
int screenHeight = 0;
int screenWidthHalf = 0;
int screenHeightHalf = 0;


void UpdateScreen()
{
    if (servoXPos < 300) servoXPos = 300;
    if (servoXPos > 700) servoXPos = 700;
    if (servoYPos < 300) servoYPos = 300;
    if (servoYPos > 700) servoYPos = 700;
    SetServoPos(1, servoXPos, 2000);
    SetServoPos(2, 1024 - servoYPos, 2000);
    UpdateServoPos();
}

void CalcScreenSize()
{
    screenWidthHalf = screenCPoint.x - screenAPoint.x;
    screenHeightHalf = screenAPoint.y - screenCPoint.y;
    screenWidth = 2 * screenWidthHalf;
    screenHeight = 2 * screenHeightHalf;


}