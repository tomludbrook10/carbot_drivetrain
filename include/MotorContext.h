#pragma once

// roughly choose these values to be around the scale of the rps which 
// the PID controller will be working with.
const double maxValue = 10.0;
const double minValue = -10.0;

// max rps is roughly ~9rps
const double MAX_RPS = 6.0f;
const double MIN_RPS = 0.25f;

const double PULSES_PER_REVO = 413.0;

const int ema_window_size = 4;
const double ema_alpha =  2.0 / (ema_window_size + 1.0);

struct EMA {
    double prevFilteredValue;

    EMA() {
      prevFilteredValue = 0.0;  
    }
    double filter(double newValue) {
        double filteredValue = (newValue - prevFilteredValue) * ema_alpha + prevFilteredValue;
        prevFilteredValue = filteredValue;
        return filteredValue;
    }
};

struct MotorContext {
  const int digitalPin1;
  const int digitalPin2;
  const int controlPin;
  const int motorChannel;

  // counter for the encoder
  const int encoderAPin;
  const int encoderBPin;

  // this is the input into the motor. 
  // It represents the voltage the h-bridge (motor-driver) will apply to the motor. 
  double motorValue;

  // encoder value.
  volatile long encoderCount;

  // theta is the current angle.
  double theta;
  double thetaPrev;
  double rps;

  // PID values.
  double e, ePrev, inte, intePrev;
  const double kp;
  const double ki;
  const double kd;

  EMA rpsFilter;

  MotorContext(const int digitalPin1_, 
                const int digitalPin2_, 
                const int controlPin_,
                const int motorChannel_, 
                const int encoderAPin_,
                const int encoderBPin_,
                const double kp_,
                const double ki_,
                const double kd_
              ) : 
              digitalPin1(digitalPin1_), 
              digitalPin2(digitalPin2_),
              controlPin(controlPin_),
              motorChannel(motorChannel_),
              encoderAPin(encoderAPin_),
              encoderBPin(encoderBPin_),
              kp(kp_),
              ki(ki_),
              kd(kd_),
              motorValue(0),
              encoderCount(0),
              e(0.0),
              ePrev(0.0),
              inte(0.0),
              intePrev(0.0) {}

  void update(const double dt);

  // motor driver function. 
  void motorDriver();

  void PIDControl(const double targetRPS, const double dt);
};