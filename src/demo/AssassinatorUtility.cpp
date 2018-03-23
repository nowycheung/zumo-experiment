#include <Zumo32U4.h>
#include "AssassinatorUtility.h"

int16_t AssassinatorUtility::getOpponentDistance(Zumo32U4ProximitySensors proxSensors) {
    return proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
}

int16_t AssassinatorUtility::getOpponentDirection(Zumo32U4ProximitySensors proxSensors) {
    return proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();
}

void AssassinatorUtility::setRotate(Zumo32U4Motors motors, int16_t speed, int16_t direction) {
    if (direction <= 0) {
        motors.setSpeeds(-speed, speed);
    } else {
        motors.setSpeeds(speed, -speed);
    }
}

void AssassinatorUtility::setRotate(Zumo32U4Motors motors, int16_t speed) {
    this->setRotate(motors, speed, -1);
}

void AssassinatorUtility::setRotate(Zumo32U4Motors motors) {
    this->setRotate(motors, -400, 400);
}
