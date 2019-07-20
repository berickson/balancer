#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <Arduino.h>
#include "math.h"
#include "kalman.h"

class Speedometer
{
public:
  Speedometer(double meters_per_tick);

  double meters_per_tick = 0; // set by client
  int last_odo = 0;
  uint32_t last_change_us = 0;

  double velocity = 0.0;
  double meters_travelled = 0.0;

  int get_ticks() const;
  double get_velocity() const;
  double get_smooth_velocity() const;
  double get_smooth_acceleration() const;
  double get_meters_travelled() const;
  

  void set_ticks(uint32_t clock_us, int32_t total_ticks);

  // updates internal state and returns meters just moved
  double add_ticks(uint32_t clock_us, int32_t odo);

  KalmanFilter kalman_v;
  KalmanFilter kalman_a;
};

#endif // SPEEDOMETER_H
