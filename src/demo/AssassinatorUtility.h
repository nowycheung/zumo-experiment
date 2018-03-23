#pragma once

#include <stdint.h>
#include <Zumo32U4.h>

class AssassinatorUtility
{
  public:
    /*
        returns 0 ... 12
        0 means nothing found, 12 means very close.
    */
    int16_t getOpponentDistance(Zumo32U4ProximitySensors proxSensors);

    /*
        Minus means left, plus means right.
    */
    int16_t getOpponentDirection(Zumo32U4ProximitySensors proxSensors);

    /*
        Speed : 0 ... 400
        Direction : Minus will be left, plus will be right
    */
    void setRotate(Zumo32U4Motors motors);

    /*
        Speed : 0 ... 400
        Direction : Minus will be left, plus will be right
    */
    void setRotate(Zumo32U4Motors motors, int16_t speed);

    /*
        Speed : 0 ... 400
        Direction : Minus will be left, plus will be right
    */
    void setRotate(Zumo32U4Motors motors, int16_t speed, int16_t direction);
};