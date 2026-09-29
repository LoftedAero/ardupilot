#include "Plane.h"
#include "gyro.h"

#if AP_PLANE_GYRO_ENABLED

const AP_Param::GroupInfo PlaneGyro::var_info[] = {

    // @Param: GAIN_CH
    // @DisplayName: Gyro master gain channel
    // @Description: RC input channel used as a master gain knob for the roll, pitch and yaw rate controllers, like the gain setting on a consumer RC gyro. The knob scales the P, I and D terms of the rate controllers. Feedforward is not scaled, so stick authority in ACRO is unchanged. Knob centre gives a scale of 1.0 (the tuned gains), the low end gives GYRO_GAIN_MIN and the high end gives GYRO_GAIN_MAX. The scale is held at 1.0 during RC failsafe and while autotune is running. Set to 0 to disable.
    // @Values: 0:Disabled,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("GAIN_CH", 1, PlaneGyro, gain_ch, 0),

    // @Param: GAIN_MIN
    // @DisplayName: Gyro master gain at knob minimum
    // @Description: Rate controller gain scale when the GYRO_GAIN_CH knob is at its low end. 0 removes all rate correction (feedforward only).
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

    // @Param: TRIM_CAP
    // @DisplayName: Gyro trim capture on mode change
    // @Description: When enabled, arming in MANUAL and switching from MANUAL to ACRO or FBWA capture the current transmitter trims: the servo trims are set to the current surface positions and the RC trims to the current stick centre, exactly as the TRIM_TO_CURRENT_SERVO_RC aux function does. This lets transmitter trims applied in MANUAL carry into the stabilised modes without becoming a rate or attitude demand. The capture is skipped silently if roll or pitch stick is more than 20 percent from centre, the airframe is rotating faster than 30 deg/s, or RC is in failsafe.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("TRIM_CAP", 4, PlaneGyro, trim_capture, 0),

    // @Param: LOG_THR
    // @DisplayName: Gyro logging throttle threshold
    // @Description: Only used when ARMING_REQUIRE is 0, where the vehicle counts as armed from boot and would otherwise log continuously. Logging is treated as armed while the throttle output is at or above this percentage, for 60 seconds after it last was, or while the vehicle is detected as flying. 0 disables the gate and logs from boot.
    // @Range: 0 100
    // @Units: %
    // @User: Standard
    AP_GROUPINFO("LOG_THR", 5, PlaneGyro, log_thr, 0),

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

    // trim capture at the moment of arming, like a gyro learning stick
    // centre at power-up
    const bool armed = plane.arming.is_armed();
    if (armed && !was_armed && trim_capture.get() > 0) {
        capture_trims();
    }
    was_armed = armed;

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
        !plane.failsafe.rc_failsafe &&
        rc().has_valid_input()) {
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
  gyro style trim capture: called from Plane::set_mode() while
  control_mode is still the old mode
 */
void PlaneGyro::mode_change_hook(const Mode &new_mode)
{
    if (trim_capture.get() <= 0) {
        return;
    }
    if (plane.control_mode != &plane.mode_manual) {
        return;
    }
    if (&new_mode != &plane.mode_acro && &new_mode != &plane.mode_fbwa) {
        return;
    }
    capture_trims();
}

/*
  run Plane::trim_radio() if it can succeed. The preconditions are the
  same as trim_radio()'s own, checked here so a capture that cannot
  happen is skipped silently instead of producing an error message.
 */
bool PlaneGyro::capture_trims()
{
    if (plane.control_mode != &plane.mode_manual) {
        return false;
    }
    if (plane.failsafe.rc_failsafe || !rc().has_valid_input()) {
        return false;
    }
    if (labs(plane.channel_roll->get_control_in()) > (plane.channel_roll->get_range() * 0.2) ||
        labs(plane.channel_pitch->get_control_in()) > (plane.channel_pitch->get_range() * 0.2)) {
        return false;
    }
    if (degrees(plane.ahrs.get_gyro().length()) > 30.0f) {
        return false;
    }
    plane.trim_radio();
    trim_captures++;
    return true;
}

void PlaneGyro::write_log()
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: GYRO
    // @Description: Gyro-style helper state
    // @Field: TimeUS: Time since system startup
    // @Field: Gain: master gain scale applied to the rate controllers
    // @Field: LogOK: logging gate state
    // @Field: TrimN: number of automatic trim captures since boot
    AP::logger().WriteStreaming("GYRO", "TimeUS,Gain,LogOK,TrimN",
                                "s---", "F---", "QfBH",
                                AP_HAL::micros64(),
                                gain_scale,
                                (uint8_t)log_allowed,
                                trim_captures);
#endif
}

#endif  // AP_PLANE_GYRO_ENABLED
