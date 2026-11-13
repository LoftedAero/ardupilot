#pragma once

/*
  "Gyro" behaviours that make ArduPlane act like a consumer RC gyro
  (AS3X / SAFE style) for pilots who never arm and never open a ground
  station:

   - a transmitter master gain knob that scales the roll, pitch and yaw
     rate controllers (P and D, and the I build-up rate; feedforward is
     untouched)
   - a throttle based logging gate for vehicles with ARMING_REQUIRE 0,
     which otherwise log continuously from boot
   - a calibration request parameter (GYRO_CAL) so gyro and level
     calibration run on the main thread when requested from a menu
   - an assumed airspeed for the attitude estimate's turn correction
     when the model flies with no GPS and no airspeed sensor
   - the transmitter setup menu over CRSF (gyro_menu.cpp)

  Everything is off by default; stock ArduPlane behaviour is unchanged
  until a GYRO_ parameter enables it.
 */

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#ifndef AP_PLANE_GYRO_ENABLED
#define AP_PLANE_GYRO_ENABLED 1
#endif

#if AP_PLANE_GYRO_ENABLED

#include "gyro_menu.h"

class PlaneGyro {
public:
    PlaneGyro();

    /* Do not allow copies */
    CLASS_NO_COPY(PlaneGyro);

    static const struct AP_Param::GroupInfo var_info[];

    // scheduler task, 10Hz
    void update();

    // true when the logger should treat the vehicle as armed
    bool logging_allowed() const { return log_allowed; }

    float get_gain_scale() const { return gain_scale; }

    // calibration requests (GYRO_CAL): 1 gyros, 2 level. The state reads
    // back the request while pending, 0 when done and -1 when it failed
    void request_calibration(int8_t type) { cal_request.set(type); }
    int8_t calibration_state() const { return cal_request.get(); }

#if AP_PLANE_GYRO_MENU_ENABLED
    // scheduler task, 50Hz: transmitter setup menu over CRSF
    void menu_update() { menu.update(); }

    // true while a menu command that must not be interrupted by arming
    // is open on the radio (calibration, learning, trim copy, reset)
    bool menu_busy() const { return menu.busy(); }
#endif

private:
    AP_Int8 gain_ch;
    AP_Float gain_min;
    AP_Float gain_max;
    AP_Int8 log_thr;
    AP_Int8 cal_request;
    AP_Int8 turn_corr;

    float gain_scale = 1.0f;
    bool log_allowed = false;   // never let the logger see "armed" before the first update()
    uint32_t last_thr_high_ms;
    bool cal_request_cleared;

    // assumed airspeed: the model counts as airborne from a second of
    // armed throttle until it is disarmed and still
    bool airborne;
    uint32_t thr_airborne_ms;
    uint32_t still_ms;

#if AP_PLANE_GYRO_MENU_ENABLED
    PlaneGyroMenu menu;
#endif

    void update_gain_scale();
    void update_logging_gate(uint32_t now_ms);
    void update_calibration();
    void update_assumed_airspeed(uint32_t now_ms);
    void write_log();
};

#endif  // AP_PLANE_GYRO_ENABLED
