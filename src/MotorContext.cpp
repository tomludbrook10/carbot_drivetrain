#include "MotorContext.h"
#include <Arduino.h>

void MotorContext::update(const double dt) {
    theta = ((double)encoderCount)/PULSES_PER_REVO;
    double current_rps = (theta - thetaPrev) / (dt / 1000.0); // derivative of the theta. 
    thetaPrev = theta;  
    rps = rpsFilter.filter(current_rps);
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int deadzone = 2;
int deadzone_bump = 12;

// motor driver function. 
void MotorContext::motorDriver() {
    int pwmValue = (int)mapf(abs(motorValue), 0.0, maxValue, 0.0, 255.0);
    if (pwmValue > 255) {
        pwmValue = 255;
    }

    if (pwmValue <= deadzone) {
        pwmValue = 0; // deadzone for motor, remove noise.
    } else if (pwmValue <= deadzone_bump) {
        pwmValue = deadzone_bump; // to overcome the motor bump.
    }
    
    if (motorValue < 0) {
        digitalWrite(digitalPin1, HIGH);
        digitalWrite(digitalPin2, LOW);
    } else if (motorValue > 0) {
        digitalWrite(digitalPin1, LOW);
        digitalWrite(digitalPin2, HIGH);
    } else {
        digitalWrite(digitalPin1, LOW);
        digitalWrite(digitalPin2, LOW);
    }
    ledcWrite(motorChannel, pwmValue);
}

void MotorContext::PIDControl(double targetRPS, const double dt){

    if (targetRPS > 0 && targetRPS < MIN_RPS) {
        targetRPS = MIN_RPS;
    } else if (targetRPS < 0 && targetRPS > -MIN_RPS) {
        targetRPS = -MIN_RPS;
    }

    e = targetRPS - rps;

    inte = intePrev + (dt * ((e+ePrev)/2));
    // final term is the derivative term.
    motorValue = kp * e + ki * inte + kd * ((e - ePrev) / dt);

    // very basic anti-windup solution.
    if (motorValue > maxValue) {
        motorValue = maxValue;
        inte = intePrev;
    }
    if (motorValue < minValue) {
        motorValue = minValue;
        inte = intePrev;
    }

    // prevent sudden direction change.
    // intutively, the motor should never go the reverse direction
    // of the rps, the intergation term adds this into the system 
    // if there is a sudden change in the load of the motor.
    if (targetRPS > 0 && motorValue < 0) {
        motorValue = 0;
    }
    if (targetRPS < 0 && motorValue > 0) {
        motorValue = 0;
    }

    ePrev = e;
    intePrev = inte;
}