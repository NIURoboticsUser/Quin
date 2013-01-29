
/*

This file is based off of code that is licensed by the GNU Public License
version 3 (GPLv3). As such, the code in this file (and only in this file,
unless stated otherwise) is also licensed under GPLv3. The full text
of the license may be found at http://www.gnu.org/licenses/gpl-3.0.html

*/

#include <math.h>
#include <Arduino.h>

#ifndef __PID_H__
#define __PID_H__

#ifndef PID_ROLLING_INTEGRAL_TIME
  #define PID_ROLLING_INTEGRAL_TIME 5000
#endif

// Approximate time between measurements. Determines rolling avg array size
#ifndef PID_APPROX_SAMPLE_RATE
  #define PID_APPROX_SAMPLE_DT 20
#endif

#define PID_ROLLING_INTEGRAL_ARRAY_SIZE (PID_ROLLING_INTEGRAL_TIME / PID_APPROX_SAMPLE_DT + 2)

int _pid_wrap(int, int, int);

class PID {
public:

  PID(const float &initial_p = 0.0,
      const float &initial_i = 0.0,
      const float &initial_d = 0.0,
      const int16_t &initial_imax = 0) {
    
    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _imax = initial_imax;

    // set _last_derivative as invalid when we startup
    _last_derivative = NAN;
  }

  /**
   * Iterate the PID, return the new control value
   * Positive error produces positive output.
   *
   * @param error The measured error value
   * @param now Time of measurement (in milliseconds)
   * @param scaler An arbitrary scale factor. Optional
   *
   * @return The updated control output.
   */
  int32_t getPid(int32_t error, unsigned long now = 0, float scaler = 1.0);

  /**
   * Reset the PID integrator
   */
  void resetI();

  /** 
   * Overload the function call operator to permit relatively easy initialisation
   */
  void operator () (const float    p,
      const float    i,
      const float    d,
      const int16_t  imaxval) {
    _kp = p; _ki = i; _kd = d; _imax = imaxval;
  }

  float getkP() const {
    return _kp;
  }
  
  float getkI() const {
    return _ki;
  }
  
  float getkD() const {
    return _kd;
  }
  
  int16_t getimax() const {
    return _imax;
  }

  void setkP(const float v) {
    _kp = v;
  }
  
  void setkI(const float v) {
    _ki = v;
  }
  
  void setkD(const float v) {
    _kd = v;
  }
  
  void setimax(const int16_t v)   {
    _imax = abs(v);
  }

  float get_integrator() const {
    return _integrator;
  }

private:
  float _kp;
  float _ki;
  float _kd;
  int16_t _imax;

  float _integrator; //integrator value
  int32_t _last_error; // last error for derivative
  float _last_derivative; // last derivative for low-pass filter
  uint32_t _last_time; // last time get_pid() was called in millis
  
  // Rolling integral stuff
  unsigned long _first_sample_time;
  uint16_t _rolling_integral_dt[PID_ROLLING_INTEGRAL_ARRAY_SIZE];
  float _rolling_integral_values[PID_ROLLING_INTEGRAL_ARRAY_SIZE];
  uint16_t _rolling_integral_start;
  uint16_t _rolling_integral_end;

  /// Low pass filter cut frequency for derivative calculation.
  ///
  /// 20 Hz becasue anything over that is probably noise, see
  /// http://en.wikipedia.org/wiki/Low-pass_filter.
  ///
  static const uint8_t _fCut = 20;
};

int32_t PID::getPid(int32_t error, unsigned long now, float scaler) {
  uint32_t dt = now - _last_time;
  float output = 0;
  float delta_time;

  if (_last_time == 0 || dt > 1000) {
    dt = 0;

    // if this PID hasn't been used for a full second then zero
    // the intergator term. This prevents I buildup from a
    // previous fight mode from causing a massive return before
    // the integrator gets a chance to correct itself
    resetI();
  }
  _last_time = now;

  delta_time = (float)dt / 1000.0;

  // Compute proportional component
  output += error * _kp;

  // Compute derivative component if time has elapsed
  if ((fabs(_kd) > 0) && (dt > 0)) {
    float derivative;

    if (isnan(_last_derivative)) {
      // we've just done a reset, suppress the first derivative
      // term as we don't want a sudden change in input to cause
      // a large D output change
      derivative = 0;
      _last_derivative = 0;
    } else {
      derivative = (error - _last_error) / delta_time;
    }

    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    float RC = 1/(2*M_PI*_fCut);
    derivative = _last_derivative +
          ((delta_time / (RC + delta_time)) *
          (derivative - _last_derivative));

    // update state
    _last_error             = error;
    _last_derivative    = derivative;

    // add in derivative component
    output += _kd * derivative;
  }

  // scale the P and D components
  output *= scaler;
  
  if (_first_sample_time == 0) {
    _first_sample_time = now;
  }

  // Compute integral component if time has elapsed
  if (/*(fabs(_ki) > 0) &&*/ (dt > 0)) {
    /*_integrator += (error * _ki) * scaler * delta_time;
    if (_integrator < -_imax) {
      _integrator = -_imax;
    } else if (_integrator > _imax) {
      _integrator = _imax;
    }*/
    
    if (now - _first_sample_time > PID_ROLLING_INTEGRAL_TIME ||
        _pid_wrap(_rolling_integral_end + 1, 0, PID_ROLLING_INTEGRAL_ARRAY_SIZE) == _rolling_integral_start) {
      // Either oldest value has expired, or we ran out of space
      
      // Remove start value
      _integrator -= _rolling_integral_values[_rolling_integral_start];
      _rolling_integral_start = _pid_wrap(_rolling_integral_start + 1, 0, PID_ROLLING_INTEGRAL_ARRAY_SIZE);
      _first_sample_time += _rolling_integral_dt[_rolling_integral_start];
    }
    
    
    // Add end value
    float newValue = error * delta_time;
    _integrator += newValue;
    _rolling_integral_values[_rolling_integral_end] = newValue;
    _rolling_integral_dt[_rolling_integral_end] = dt;
    _rolling_integral_end = _pid_wrap(_rolling_integral_end + 1, 0, PID_ROLLING_INTEGRAL_ARRAY_SIZE);
    
    output += constrain(_integrator * _ki, -_imax, _imax) * scaler;
  }

  return output;
}

void PID::resetI() {
  _integrator = 0;
  // we use NAN (Not A Number) to indicate that the last 
  // derivative value is not valid
  _last_derivative = NAN;
  _first_sample_time = 0;
}

inline int _pid_wrap(int value, int min, int max) {
  int range = max - min + 1;

  if (value < min)
    value += range * ((min - value) / range + 1);

  return min + (value - min) % range;
}

#endif
