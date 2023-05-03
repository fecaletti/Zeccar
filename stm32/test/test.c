#include <stdio.h>

static unsigned char WheelMotorSpeed = 30;
static unsigned char TurnMotorQuantity = 30;
static unsigned char CurrentMotorCommand = 0;
static unsigned char CurrentTurnCommand = 0;

int main()
{
    char usbBuff[255];
    sprintf(usbBuff, "WS -> %i; WP -> %i; TS -> %i; TP -> %i;\n\r", CurrentMotorCommand, WheelMotorSpeed, CurrentTurnCommand, TurnMotorQuantity);
    printf("%s", usbBuff);

    return 0;
}