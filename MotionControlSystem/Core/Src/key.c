#include "key.h"

int8_t bitMode = 0;
int8_t bitModeLast = -1;
int8_t bit0 = 0;
int8_t bit1 = 0;



void ReadKeyBit()
{
    bitMode = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    bit0 = !HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
    bit1 = !HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
    if (bitModeLast == -1) bitModeLast = bitMode;
}


int IsModeChange()
{
    if (bitModeLast != bitMode) {
        bitModeLast = bitMode;
        osDelay(200);
        return 1;
    }
    return 0;
}