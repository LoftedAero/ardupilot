#pragma once

/*
  "Gyro" behaviours that make ArduPlane act like a consumer RC gyro
  (AS3X / SAFE style) for pilots who set it up from the transmitter
  and never open a ground station:

   - a transmitter master gain knob that scales the roll, pitch and yaw
     rate controllers (P and D, and the I build-up rate; feedforward is
     untouched)
   - the level calibration the menu requests, run on the next update
     so the menu can answer the radio first
   - an assumed airspeed for the attitude estimate's turn correction
     when the model flies with no GPS and no airspeed sensor
   - the transmitter setup menu over CRSF (stabrx_menu.cpp)

  The knob and the turn correction are off until their STABRX_ parameter
  is set. The menu runs when a CRSF receiver is connected.
 */

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#ifndef AP_PLANE_STABRX_ENABLED
#define AP_PLANE_STABRX_ENABLED 1
#endif

#if AP_PLANE_STABRX_ENABLED

#include "stabrx_menu.h"

class PlaneStabRX {
public:
    PlaneStabRX();

    /* Do not allow copies */
    CLASS_NO_COPY(PlaneStabRX);

    static const struct AP_Param::GroupInfo var_info[];

    // scheduler task, 10Hz
    void update();

    float get_gain_scale() const { return gain_scale; }

    // level calibration (AHRS trim) requested by the menu
    enum LevelCalState : int8_t {
        LEVEL_CAL_FAILED = -1,
        LEVEL_CAL_DONE = 0,
        LEVEL_CAL_PENDING = 1,
    };
    void request_level_calibration() { level_cal_state = LEVEL_CAL_PENDING; }
    LevelCalState level_calibration_state() const { return level_cal_state; }

#if AP_PLANE_STABRX_MENU_ENABLED
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
    AP_Int8 turn_corr;

    float gain_scale = 1.0f;
    LevelCalState level_cal_state;

    // assumed airspeed: the model counts as airborne from a second of
    // armed throttle until it is disarmed and still
    bool airborne;
    uint32_t thr_airborne_ms;
    uint32_t still_ms;

#if AP_PLANE_STABRX_MENU_ENABLED
    PlaneStabRXMenu menu;
#endif

    void update_gain_scale();
    void update_calibration();
    void update_assumed_airspeed(uint32_t now_ms);
    void write_log();
};

#endif  // AP_PLANE_STABRX_ENABLED
