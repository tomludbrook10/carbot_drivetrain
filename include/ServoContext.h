#pragma once 

#include <Arduino.h>

struct ServoContext {
  const int servoFrequncy; // in Hz.
  const int resolution;
  const int servoPin;
  const int pwmChannel;
  const float maxAngle;
  const float minAngle;

  float servoValue;

  ServoContext(const int servoPin_, 
              const int servoFrequncy_, 
              const int pwmChannel_, 
              const int minAngle_, 
              const int maxAngle_,
              const int resolution_) :
  servoPin(servoPin_),
  servoFrequncy(servoFrequncy_),
  pwmChannel(pwmChannel_),
  resolution(resolution_),
  minAngle(minAngle_),
  maxAngle(maxAngle_) { }

  void servoDriver();
};