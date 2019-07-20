#include <Arduino.h>
#include "speedometer.h"
#include "kalman.h"

using namespace std;

/*
template<class T> T max(T x1,T x2) {
  return x1>x2?x1:x2;
}
*/

Speedometer::Speedometer(double meters_per_tick): meters_per_tick(meters_per_tick)  {
}

void Speedometer::set_ticks(uint32_t clock_us, int32_t total_ticks) {
  add_ticks(clock_us, total_ticks - last_odo );
}

int Speedometer::get_ticks() const {
  return last_odo;
}

double Speedometer::get_velocity() const {
  return velocity;
}

double Speedometer::get_smooth_velocity() const {
  return kalman_v.mean;
}

double Speedometer::get_smooth_acceleration() const {
  return kalman_a.mean;
}

double Speedometer::get_meters_travelled() const {
  return meters_travelled;
}


double Speedometer::add_ticks(uint32_t clock_us, int32_t ticks_moved) {
  double last_v = velocity;

  double meters_moved = ticks_moved*meters_per_tick;
  double elapsed_seconds = (clock_us - last_change_us) / 1000000.;

  last_odo += ticks_moved;

  velocity = meters_moved / elapsed_seconds;
  if (ticks_moved == 0) {
    // no tick this time, how long has it been?
    elapsed_seconds= ( clock_us - last_change_us) / 1000000.;
    if (elapsed_seconds > 0.1){
      // it's been a long time, let's call velocity zero
      velocity = 0.0;
    } else {
      // we've had a tick recently, fastest possible speed is when a tick is about to happen
      // do nothing unless smaller than previously certain velocity
      double  max_possible = meters_per_tick / elapsed_seconds;
      if(max_possible < fabs(velocity)){
        if(velocity > 0)
          velocity = max_possible;
        else
          velocity = -max_possible;
      }
    }
  }
  meters_travelled += meters_moved;
  if(last_change_us > 0) {
    auto dt = (clock_us - last_change_us) * 1E-6;
    kalman_v.predict(0,100.0 *dt*dt); // sigma = 10m/s^2 * dt, variance = sigma^2 = 100 * dt^2
    kalman_a.predict(0,900.0*dt*dt);  // sigma = 30m/s^3 * dt, variance = 900 m/s^2 * dt^2
  }
  kalman_v.update(velocity,0.01);
  if(elapsed_seconds > 0) {
    auto a = (velocity - last_v) / elapsed_seconds;
    kalman_a.update(a,1.0*1.0);
  }


  if(ticks_moved) {
    last_change_us = clock_us;
  }

  return meters_moved;
}
