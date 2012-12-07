//Board = Arduino Mega 2560 or Mega ADK
#define ARDUINO 10
#define __AVR_ATmega2560__
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
#define __attribute__(x)
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__
#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define prog_void
#define PGM_VOID_P int
#define NOINLINE __attribute__((noinline))

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {}

void GPS_calc_longitude_scaling(int32_t lat);
void GPS_set_next_wp(uint8_t wp_number);
static bool check_missed_wp();
uint32_t GPS_distance_cm(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
int32_t GPS_bearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
static void GPS_calc_velocity( int32_t gps_latitude, int32_t gps_longitude);
static void GPS_calc_location_error( int32_t target_lat, int32_t target_lng, int32_t gps_lat, int32_t gps_lng );
static void GPS_calc_poshold(int x_error, int y_error);
static void GPS_calc_nav_rate(int max_speed);
static void GPS_update_crosstrack(void);
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow);
void GPS_reset_nav();
void GPS_update_i2c_dataset();
int32_t wrap_18000(int32_t error);
int32_t wrap_36000(int32_t angle);
uint32_t GPS_coord_to_degrees(char* s);
bool GPS_NMEA_newFrame(char c);
void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b);
bool GPS_UBLOX_newFrame(uint8_t data);
bool UBLOX_parse_gps(void);
inline long _swapl(const void *bytes);
bool GPS_MTK_newFrame(uint8_t data);
void requestEvent();
void receiveEvent(int bytesReceived);
void blink_update();
//already defined in arduno.h
//already defined in arduno.h
//already defined in arduno.h

#include "Z:\arduino-1.0\hardware\arduino\variants\mega\pins_arduino.h" 
#include "Z:\arduino-1.0\hardware\arduino\cores\arduino\arduino.h"
#include "Z:\I2C_GPS_NAV\I2C_GPS_NAV.ino"
#include "Z:\I2C_GPS_NAV\LeadFilter.cpp"
#include "Z:\I2C_GPS_NAV\LeadFilter.h"
#include "Z:\I2C_GPS_NAV\PICtrl.cpp"
#include "Z:\I2C_GPS_NAV\PICtrl.h"
#include "Z:\I2C_GPS_NAV\PIDCtrl.cpp"
#include "Z:\I2C_GPS_NAV\PIDCtrl.h"
#include "Z:\I2C_GPS_NAV\WireMW.cpp"
#include "Z:\I2C_GPS_NAV\WireMW.h"
#include "Z:\I2C_GPS_NAV\config.h"
#include "Z:\I2C_GPS_NAV\registers.h"
#include "Z:\I2C_GPS_NAV\twiMW.c"
#include "Z:\I2C_GPS_NAV\twiMW.h"
