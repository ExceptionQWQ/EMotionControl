#include "servo.h"


uint8_t CheckSum(uint8_t startNum, uint8_t endNum, uint8_t *inData)
{
    uint8_t count;
    uint16_t checkSum = 0;
    for (count = startNum; count < endNum + 1; ++count)
    {
        checkSum += inData[count];
    }
    return ((~checkSum) & 0xFF);
}

void SetServoPos(uint8_t id, uint16_t pos, uint16_t speed)
{
    if (id == 1) {
        if (pos < 300) pos = 300;
        if (pos > 700) pos = 700;
    } else if (id == 2) {
        if (pos < 300) pos = 300;
        if (pos > 700) pos = 700;
    }
    uint8_t writeTempCmd[] = {0xFF, 0xFF, 0x00, 0x09, 0x04, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    writeTempCmd[2] = id;
    writeTempCmd[6] = (pos >> 8) & 0xff;
    writeTempCmd[7] = pos & 0xff;
    writeTempCmd[10] = (speed >> 8) & 0xff;
    writeTempCmd[11] = speed & 0xff;
    writeTempCmd[12] = CheckSum(2, 11, writeTempCmd);

    HAL_UART_Transmit(&huart3, writeTempCmd, 13, 10);
    HAL_Delay(1);
}

uint16_t GetServoPos(uint8_t id)
{
    //TODO
    return 0;
}

void UpdateServoPos()
{
    uint8_t writeTempCmd[] = {0xff, 0xff, 0xfe, 0x02, 0x05, 0xfa};
    HAL_UART_Transmit(&huart3, writeTempCmd, 6, 10);
    HAL_Delay(1);
}