#pragma once
#include "Arduino.h"
#include "FunctionalInterrupt.h"

 class QuadratureEncoder {
  public:
  const int pin_sensor_a; // sensor that triggers first for forward
  const int pin_sensor_b; 
  int last_a = -1;
  int last_b = -1;
  int last_forward = false;

  volatile long odometer_a = 0;
  volatile long odometer_b = 0;
  volatile long direction_change_count = 0;
  volatile long extra_interrupts_count = 0;

  void reset() {
    odometer_a = 0;
    odometer_b = 0;
    direction_change_count = 0;
  }

  QuadratureEncoder(int _pin_sensor_a, int _pin_sensor_b):
    pin_sensor_a(_pin_sensor_a),
    pin_sensor_b(_pin_sensor_b)
  {
  }

  void init() {
    pinMode(pin_sensor_a, INPUT);
    pinMode(pin_sensor_b, INPUT);
    attachInterrupt(pin_sensor_a, std::bind(&QuadratureEncoder::sensor_a_changed, this), CHANGE);
    attachInterrupt(pin_sensor_b, std::bind(&QuadratureEncoder::sensor_b_changed, this), CHANGE);
  }
  
  void IRAM_ATTR sensor_a_changed() {
    int a=digitalRead(pin_sensor_a);
    int b=digitalRead(pin_sensor_b);
    bool forward;
    if(a==last_a) {
      ++extra_interrupts_count;
      return;
    }
    if(a==b){
      forward = false;
      --odometer_a;
    } else {
      forward = true;
      ++odometer_a;
    }
    if(forward!=last_forward) {
      ++direction_change_count;
      last_forward=forward;
    }
    last_a=a;
    last_b=b;
  }
  
  void IRAM_ATTR sensor_b_changed() {
    int a=digitalRead(pin_sensor_a);
    int b=digitalRead(pin_sensor_b);
    bool forward;
    if(b==last_b) {
      ++extra_interrupts_count;
    }
    if(a==b){
      forward=true;
      ++odometer_b;
    } else {
      forward=false;
      --odometer_b;
    }
    if(forward!=last_forward) {
      ++direction_change_count;
      last_forward=forward;
    }
    last_a=a;
    last_b=b;
  }
};