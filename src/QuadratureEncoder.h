#pragma once
#include "Arduino.h"
#include "FunctionalInterrupt.h"


class QuadratureEncoder {
  public:
  const int pin_sensor_a; // sensor that triggers first for forward
  const int pin_sensor_b; 
  volatile int last_a = -1;
  volatile int last_b = -1;
  volatile int last_forward = false;

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
  }

};


void IRAM_ATTR sensor_a_changed(QuadratureEncoder & e ) {
    int a=digitalRead(e.pin_sensor_a);
    int b=digitalRead(e.pin_sensor_b);
    bool forward;
    if(a==e.last_a) {
      ++e.extra_interrupts_count;
      return;
    }
    if(a==b){
      forward = false;
      --e.odometer_a;
    } else {
      forward = true;
      ++e.odometer_a;
    }
    if(forward!=e.last_forward) {
      ++e.direction_change_count;
      e.last_forward=forward;
    }
    e.last_a=a;
    e.last_b=b;
}

void IRAM_ATTR sensor_b_changed(QuadratureEncoder & e) {
    int a=digitalRead(e.pin_sensor_a);
    int b=digitalRead(e.pin_sensor_b);
    bool forward;
    if(b==e.last_b) {
      ++e.extra_interrupts_count;
    }
    if(a==b){
      forward=true;
      ++e.odometer_b;
    } else {
      forward=false;
      --e.odometer_b;
    }
    if(forward!=e.last_forward) {
      ++e.direction_change_count;
      e.last_forward=forward;
    }
    e.last_a=a;
    e.last_b=b;
}
