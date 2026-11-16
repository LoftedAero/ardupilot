#include "Plane.h"
#include "gyro.h"

#if AP_PLANE_GYRO_ENABLED

const AP_Param::GroupInfo PlaneGyro::var_info[] = {

    // @Param: GAIN_CH
    // @DisplayName: Gyro master gain channel
    // @Description: RC input channel used as a master gain knob for the roll, pitch and yaw rate controllers, like the gain setting on a consumer RC gyro. The knob scales the P and D terms of the rate controllers and how fast the I term builds up, so turning it never steps the I output. Feedforward is not scaled, so stick authority in ACRO is unchanged. Knob centre gives a scale of 1.0 (the tuned gains), the low end gives GYRO_GAIN_MIN and the high end gives GYRO_GAIN_MAX. The scale keeps its last value during RC failsafe, and is held at 1.0 while autotune is running. Set to 0 to disable.
    // @Values: 0:Disabled,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("GAIN_CH", 1, PlaneGyro, gain_ch, 0),

    // @Param: GAIN_MIN
    // @DisplayName: Gyro master gain at knob minimum
    // @Description: Rate controller gain scale when the GYRO_GAIN_CH knob is at its low end. 0 removes the P and D correction and holds the I term at its current value, leaving feedforward.
    // @Range: 0 1
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("GAIN_MIN", 2, PlaneGyro, gain_min, 0.0f),

    // @Param: GAIN_MAX
    // @DisplayName: Gyro master gain at knob maximum
    // @Description: Rate controller gain scale when the GYRO_GAIN_CH knob is at its high end.
    // @Range: 1 3
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("GAIN_MAX", 3, PlaneGyro, gain_max, 1.5f),

    // index 4 was TRIM_CAP (trim capture, removed): do not reuse

    // @Param: LOG_THR
    // @DisplayName: Gyro logging throttle threshold
    // @Description: Only used when ARMING_REQUIRE is 0, where the vehicle counts as armed from boot and would otherwise log continuously. Logging is treated as armed while the throttle output is at or above this percentage, for 60 seconds after it last was, or while the vehicle is detected as flying. 0 disables the gate and logs from boot.
    // @Range: 0 100
    // @Units: %
    // @User: Standard
    AP_GROUPINFO("LOG_THR", 5, PlaneGyro, log_thr, 0),

    // @Param: CAL
    // @DisplayName: Gyro calibration request
    // @Description: Used by the transmitter setup menu. Set to 1 to calibrate the gyros, or 2 to calibrate level (simple accelerometer calibration with the model held at its level flight attitude). The calibration runs on the main thread, only while disarmed, like a ground station calibration. The value reads back 0 when the calibration succeeded and -1 when it failed or was refused. It is reset to 0 at boot.
    // @Values: -1:Last calibration failed,0:Idle,1:Calibrate gyros,2:Calibrate level
    // @User: Advanced
    AP_GROUPINFO("CAL", 6, PlaneGyro, cal_request, 0),

    // @Param: TURN_CORR
    // @DisplayName: Gyro turn correction without GPS
    // @Description: Keeps the attitude estimate correct in turns when there is no GPS and no airspeed sensor. Without a speed the estimate cannot correct for the acceleration of a turn, and in a held FBWA turn the real bank and dive angles grow while the estimate reads steady. When enabled, an assumed airspeed is used for that correction: SCALING_SPEED at 45% throttle, lower at low throttle and higher at high throttle, the same model that scales the gains by throttle. Set SCALING_SPEED to the model's cruise speed. The assumed airspeed is used from one second of armed throttle above 30% until the model is disarmed and still, and never when a GPS or airspeed sensor gives a speed.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("TURN_CORR", 7, PlaneGyro, turn_corr, 0),

    AP_GROUPEND
};

PlaneGyro::PlaneGyro()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void PlaneGyro::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    update_gain_scale();
    update_logging_gate(now_ms);
    update_calibration();
    update_assumed_airspeed(now_ms);

    write_log();
}

/*
  map the gain knob to a scale factor and push it into the rate
  controllers. Knob centre is always 1.0 so the autotuned gains are
  reachable regardless of GYRO_GAIN_MIN/MAX.
 */
void PlaneGyro::update_gain_scale()
{
    float scale = 1.0f;
    const int8_t ch = gain_ch.get();
    if (ch > 0 && ch <= NUM_RC_CHANNELS &&
        (plane.failsafe.rc_failsafe || !rc().has_valid_input())) {
        // no knob to read: keep the last gain. The pilot may have
        // turned it down because the model oscillates at 1.0
        scale = gain_scale;
    } else if (ch > 0 && ch <= NUM_RC_CHANNELS) {
        const RC_Channel *c = rc().channel(ch - 1);
        if (c != nullptr) {
            const float n = constrain_float(c->norm_input_ignore_trim(), -1.0f, 1.0f);
            if (n < 0) {
                scale = 1.0f + n * (1.0f - gain_min.get());
            } else {
                scale = 1.0f + n * (gain_max.get() - 1.0f);
            }
            scale = constrain_float(scale, 0.0f, 5.0f);
        }
    }
    gain_scale = scale;
    plane.rollController.set_gain_scale(scale);
    plane.pitchController.set_gain_scale(scale);
    plane.yawController.set_gain_scale(scale);
}

/*
  throttle based logging gate for vehicles that never arm
 */
void PlaneGyro::update_logging_gate(uint32_t now_ms)
{
    if (log_thr.get() <= 0 ||
        plane.arming.arming_required() != AP_Arming::Required::NO) {
        // stock behaviour
        log_allowed = true;
        return;
    }
    const float thr = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (thr >= log_thr.get()) {
        last_thr_high_ms = now_ms;
    }
    const bool recent_throttle = (last_thr_high_ms != 0) && (now_ms - last_thr_high_ms < 60000U);
    log_allowed = recent_throttle || plane.is_flying();
}

/*
  run a calibration requested through GYRO_CAL. Calling the calibration
  from a Lua script directly (gcs:run_command_int) blocks the main loop
  while the calibration waits for IMU samples, so the script only sets
  the request and the calibration runs here, on the main thread.
 */
void PlaneGyro::update_calibration()
{
    if (!cal_request_cleared) {
        // a request must never survive a reboot (for example if a
        // ground station saved the parameter)
        cal_request_cleared = true;
        if (cal_request.get() != 0) {
            cal_request.set_and_save(0);
        }
        return;
    }

    const int8_t req = cal_request.get();
    if (req != 1 && req != 2) {
        return;
    }

    bool ok = false;
    if (hal.util->get_soft_armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "StabRX: disarm to calibrate");
    } else {
        EXPECT_DELAY_MS(30000);
        plane.in_calibration = true;
        if (req == 1) {
            ok = AP::ins().calibrate_gyros();
        } else {
            ok = AP::ins().simple_accel_cal() == MAV_RESULT_ACCEPTED;
        }
        plane.in_calibration = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "StabRX: %s calibration %s",
                      req == 1 ? "gyro" : "level", ok ? "done" : "failed");
    }
    cal_request.set(ok ? 0 : -1);
}

/*
  give the attitude estimate an assumed airspeed while the model flies
  with no GPS and no airspeed sensor. DCM uses it only for its
  centripetal correction and only when it has no other speed.
 */
void PlaneGyro::update_assumed_airspeed(uint32_t now_ms)
{
    // throttle that counts as a takeoff or a launch, and for how long
    const float airborne_throttle_pct = 30;
    const uint32_t airborne_throttle_ms = 1000;
    // a disarmed model that is still this long has landed
    const uint32_t landed_still_ms = 3000;

    const bool armed = hal.util->get_soft_armed();
    if (armed && SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) >= airborne_throttle_pct) {
        if (thr_airborne_ms == 0) {
            thr_airborne_ms = now_ms;
        } else if (now_ms - thr_airborne_ms >= airborne_throttle_ms) {
            airborne = true;
        }
    } else {
        thr_airborne_ms = 0;
    }
    // a disarm in flight is a throttle cut: the model still flies
    if (!armed && AP::ins().is_still()) {
        if (still_ms == 0) {
            still_ms = now_ms;
        } else if (now_ms - still_ms >= landed_still_ms) {
            airborne = false;
        }
    } else {
        still_ms = 0;
    }

    float airspeed = 0;
    if (turn_corr.get() > 0 && airborne) {
        // the speed the throttle based gain scaling stands for
        airspeed = plane.g.scaling_speed / MAX(plane.get_speed_scaler(), 0.1f);
    }
    plane.ahrs.set_assumed_airspeed_EAS(airspeed);
}

void PlaneGyro::write_log()
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: GYRO
    // @Description: Gyro-style helper state
    // @Field: TimeUS: Time since system startup
    // @Field: Gain: master gain scale applied to the rate controllers
    // @Field: LogOK: logging gate state
    // @Field: ASpd: airspeed assumed for the attitude estimate's turn correction, 0 when none
    AP::logger().WriteStreaming("GYRO", "TimeUS,Gain,LogOK,ASpd",
                                "s--n", "F--0", "QfBf",
                                AP_HAL::micros64(),
                                gain_scale,
                                (uint8_t)log_allowed,
                                plane.ahrs.get_assumed_airspeed_EAS());
#endif
}

#endif  // AP_PLANE_GYRO_ENABLED
