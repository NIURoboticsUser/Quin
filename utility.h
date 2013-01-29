#ifndef _util_h
#define _util_h

#ifdef __AVR_ATmega2560__
  #define ARDUINO_MEGA
#else
  #define ARDUINO_UNO
#endif


inline int wrap(int value, int min, int max) {
  int range = max - min + 1;

  if (value < min)
    value += range * ((min - value) / range + 1);

  return min + (value - min) % range;
}

#endif
