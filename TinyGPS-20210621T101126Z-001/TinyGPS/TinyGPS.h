
#ifndef TinyGPS_h
#define TinyGPS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class TinyGPS
{
public:
  enum {
    GPS_INVALID_AGE = 0xFFFFFFFF,      GPS_INVALID_ANGLE = 999999999, 
     GPS_INVALID_TIME = 0xFFFFFFFF,	
    GPS_INVALID_FIX_TIME = 0xFFFFFFFF, GPS_INVALID_SATELLITES = 0xFF,
    GPS_INVALID_HDOP = 0xFFFFFFFF
  };

  static const float GPS_INVALID_F_ANGLE;

  TinyGPS();
  bool encode(char c); // process one character received from GPS
  TinyGPS &operator << (char c) {encode(c); return *this;}

  // lat/long in hundred thousandths of a degree and age of fix in milliseconds
  void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0);

 
  // satellites used in last full GPGGA sentence
  inline unsigned short satellites() { return _numsats; }

  // horizontal dilution of precision in 100ths
  inline unsigned long hdop() { return _hdop; }

  void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0);
  

#ifndef _GPS_NO_STATS
  void stats(unsigned long *chars, unsigned short *good_sentences, unsigned short *failed_cs);
#endif

private:
  enum {_GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_OTHER};

  // properties
  unsigned long _time, _new_time;
  
  long _latitude, _new_latitude;
  long _longitude, _new_longitude;
 
  unsigned long  _hdop, _new_hdop;
  unsigned short _numsats, _new_numsats;

  unsigned long _last_time_fix, _new_time_fix;
  unsigned long _last_position_fix, _new_position_fix;

  // parsing state variables
  byte _parity;
  bool _is_checksum_term;
  char _term[15];
  byte _sentence_type;
  byte _term_number;
  byte _term_offset;
  bool _gps_data_good;

#ifndef _GPS_NO_STATS
  // statistics
  unsigned long _encoded_characters;
  unsigned short _good_sentences;
  unsigned short _failed_checksum;
  unsigned short _passed_checksum;
#endif

  // internal utilities
  int from_hex(char a);
  unsigned long parse_decimal();
  unsigned long parse_degrees();
  bool term_complete();
  bool gpsisdigit(char c) { return c >= '0' && c <= '9'; }
  long gpsatol(const char *str);
  int gpsstrcmp(const char *str1, const char *str2);
};

#if !defined(ARDUINO) 
// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round 
#endif

#endif
