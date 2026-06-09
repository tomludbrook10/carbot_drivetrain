#include "ServoContext.h"

void ServoContext::servoDriver() {
    if (servoValue < minAngle)
        servoValue = minAngle;
    if (servoValue > maxAngle)
        servoValue = maxAngle;
    // https://www.handsontec.com/dataspecs/motor_fan/MG996R.pdf servo datasheet.
    // note that this datasheet seems to be wrong, the physical limitations of my servo is 
    // 45-135 degrees. 
    int value = map((long)servoValue, -45, 45, 205, 409);
    ledcWrite(pwmChannel, value);
}


