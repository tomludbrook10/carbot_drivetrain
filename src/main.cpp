#include <Arduino.h>
#include <atomic>
#include <string>
#include <cstdlib>
#include "ServoContext.h"
#include "MotorContext.h"
#include "SerialCom.h"

/// radio control

const int channel1 =  18;
const int channel2 =  16;

int readPulse(int channel, int minValue, int maxValue, int defaultValue) {
  int input = pulseIn(channel, HIGH, 30000);
  if (input < 100) return defaultValue;
  return map(input, 1000, 2000, minValue, maxValue);
}  

const float pi = 3.141592;

// time for encoder. 
unsigned long t, tPrev, tPrev_ =  0;

// PWM setup
const int freq = 20000;
const int resolution = 8; // 8 bit resolution (0-255)

volatile unsigned long count; // volatile tells the compiler that the value can be changed externally. 
unsigned long countPrev;
unsigned long countForTransmitter;

// put function definitions here:
MotorContext rightMotorContext(14, 27, 13, 0, 34, 35, 1, 0.002, 0.02);
MotorContext leftMotorContext(25, 33, 32, 1, 36, 39, 1, 0.002, 0.02);
ServoContext servoContext(17, 50, 2, -25, 25, 12); // 50Hz.

#define STAND_BY_PIN 26

#define CLOCK_SYNC_PIN 5

// timer
hw_timer_t *timer = NULL;

// interrupt functions.

// interrupt timer function. 
void IRAM_ATTR onTimer() {
  count++;
}

void IRAM_ATTR countLeftPulses() {
  bool encoderA = digitalRead(leftMotorContext.encoderAPin);
  bool encoderB = digitalRead(leftMotorContext.encoderBPin);

  if (encoderA != encoderB) {
    leftMotorContext.encoderCount--;
  }
  else {
    leftMotorContext.encoderCount++;
  }
}

void IRAM_ATTR countRightPulses() {
  bool encoderA = digitalRead(rightMotorContext.encoderAPin);
  bool encoderB = digitalRead(rightMotorContext.encoderBPin);

  if (encoderA != encoderB) {
    rightMotorContext.encoderCount++;
  }
  else {
    rightMotorContext.encoderCount--;
  }
}

hw_timer_t *timer_time_stamp = NULL;
volatile int64_t ros_time_offset_us = 0;

void IRAM_ATTR syncTime() {
  ros_time_offset_us = timerRead(timer_time_stamp);
}

float sign(float x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  }
  return 0;
}

void get_nums() {
  while(Serial.available()) {
    char c = Serial.read();
    if (c == '\n')
      break;
  }
}

void rc_control_setup() {
  pinMode(channel1, INPUT_PULLUP);
  pinMode(channel2, INPUT_PULLUP);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // set the clockrate here for our timer. 
  setCpuFrequencyMhz(240);

  // timer set up. 
  // NOTE: these timers are based off of the APB clock, which is running at 80MHz.
  // set the preset to 80, so we get 1MHz of precision.
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true); // 10ms.
  timerAlarmEnable(timer);

  // timer set up. 
  // set the preset to 80, so we get 1MHz of precision.
  timer_time_stamp = timerBegin(1, 80, true);

  // interrupts. 
  pinMode(leftMotorContext.encoderAPin, INPUT_PULLDOWN);
  pinMode(rightMotorContext.encoderAPin, INPUT_PULLDOWN);
  pinMode(leftMotorContext.encoderBPin, INPUT_PULLDOWN);
  pinMode(rightMotorContext.encoderBPin, INPUT_PULLDOWN);
  pinMode(CLOCK_SYNC_PIN, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(leftMotorContext.encoderAPin), countLeftPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightMotorContext.encoderAPin), countRightPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLOCK_SYNC_PIN), syncTime, HIGH);

  pinMode(STAND_BY_PIN, OUTPUT);

  // left Motor. 
  pinMode(leftMotorContext.digitalPin1, OUTPUT);
  pinMode(leftMotorContext.digitalPin2, OUTPUT);

  // pwm pin for leftMotor.
  ledcSetup(leftMotorContext.motorChannel, freq, resolution);
  ledcAttachPin(leftMotorContext.controlPin, leftMotorContext.motorChannel);

  // right Motor. 
  pinMode(rightMotorContext.digitalPin1, OUTPUT);
  pinMode(rightMotorContext.digitalPin2, OUTPUT);

  // pwm pin for right Motor.
  ledcSetup(rightMotorContext.motorChannel, freq, resolution);
  ledcAttachPin(rightMotorContext.controlPin, rightMotorContext.motorChannel);

  // servo. 
  ledcSetup(servoContext.pwmChannel, servoContext.servoFrequncy, servoContext.resolution);
  ledcAttachPin(servoContext.servoPin, servoContext.pwmChannel);

  Serial.println("Setup Complete");

  digitalWrite(STAND_BY_PIN, HIGH);

  // creating a thread for command reading .
  xTaskCreatePinnedToCore(getCurrentCommand, "Command Reader", 10000, NULL, 1, NULL, 1);
}

void profile_motor(double rpsD);
void run_servo();
void run_motor(float dt);
void rc_run_motor();
void rc_run_servo();
void run_motor_tune_pid(double dt, double rpsD);

uint64_t get_offset_time() {
  return timerRead(timer_time_stamp) - ros_time_offset_us;
}

long long timer_count; 

double test_rps(double t) {
  return MAX_RPS * std::sin(2 * PI *0.005 * t / 1000.0 ) * sign(std::sin(2 * PI * 0.05 * t / 1000.0 ));
}

void loop() {

  // runs every ~10ms. 
  if (count > countPrev) {
    // first run controller for steering and motors. 
    t = millis();
    double dt = static_cast<double>(t - tPrev);

    if (std::isnan(dt)) {
      Serial.println("L, NaN dt");
    }

    if (dt >= 11.0) {
      Serial.print("L, Large dt: ");
      Serial.println(dt);
    }

    leftMotorContext.update(dt);
    rightMotorContext.update(dt);

    run_motor(dt);
    run_servo();
    tPrev = t;
    countPrev = count;
  }

  // // run every ~50ms 
  if (count > countForTransmitter + 1) {
    Serial.print("D,");
    Serial.print(get_offset_time()); 
    Serial.print(",");
    Serial.println((leftMotorContext.rps + rightMotorContext.rps) / 2);
    countForTransmitter = count;
  }
}

void rc_run_motor() {
  int motorValue = readPulse(channel1, -minValue, maxValue, 0);
  leftMotorContext.motorValue = -motorValue; rightMotorContext.motorValue = -motorValue;
  leftMotorContext.motorDriver(); rightMotorContext.motorDriver();
}

void rc_run_servo() {
  int servoValue = readPulse(channel2, -25, 25, 0);
  servoContext.servoValue = servoValue;
  servoContext.servoDriver();
}

void run_motor(float dt) {
    float rpsD = desiredRPS.load(std::memory_order_relaxed);
    leftMotorContext.PIDControl(rpsD, dt);
    rightMotorContext.PIDControl(rpsD, dt);
    leftMotorContext.motorDriver();
    rightMotorContext.motorDriver();
}

void run_motor_tune_pid(double dt, double rpsD) {
    leftMotorContext.PIDControl(rpsD, dt);
    rightMotorContext.PIDControl(rpsD, dt);
    leftMotorContext.motorDriver();
    rightMotorContext.motorDriver();
}

void run_servo() {
  int servoD = desiredSA.load(std::memory_order_relaxed);
  servoContext.servoValue = servoD;
  servoContext.servoDriver();
}

void profile_motor(double rpsD) {
  Serial.print(rpsD); Serial.print(" \t");
  Serial.print(leftMotorContext.rps); Serial.print(" \t");
  Serial.print(rightMotorContext.rps); Serial.print(" \t");
  Serial.println();
}
