#pragma once

#include "main.h"

typedef struct Pin {
    GPIO_TypeDef* GPIO_Port;
    uint16_t      GPIO_Pin;
} Pin;


class MotorController
{
public:
    MotorController(Pin _ena, Pin _enb,
                    Pin _in1, Pin _in2, Pin _in3, Pin _in4,
                    double _motorAConst, double _motorBConst);
    void move(int leftSpeed, int rightSpeed, int minAbsSpeed);
    void move(int speed);
    void move(int speed, int minAbsSpeed);
    void turnLeft(int speed, bool kick);
    void turnRight(int speed, bool kick);
    void stopMoving();

protected:
    void pwmInit();
    int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

protected:
    Pin ena, in1, in2;
    Pin enb, in3, in4;
    int currentSpeed;
    double motorAConst, motorBConst;
};

