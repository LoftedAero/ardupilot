#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQStol::_enter()
{
    quadplane.throttle_wait = false;
    return true;
}

void ModeQStol::update()
{
    // set nav_roll and nav_pitch using sticks
    // Beware that QuadPlane::tailsitter_check_input (called from Plane::read_radio)
    // may alter the control_in values for roll and yaw, but not the corresponding
    // radio_in values. This means that the results for norm_input would not necessarily
    // be correct for tailsitters, so get_control_in() must be used instead.
    // normalize control_input to [-1,1]
    const float roll_input = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    // use dedicated parameters for pitch and roll limits
    plane.nav_roll_cd = roll_input * plane.quadplane.roll_stol;
    plane.nav_pitch_cd = pitch_input * plane.quadplane.pitch_stol + plane.quadplane.trim_stol * 100.0f;
}

// quadplane stabilize mode
void ModeQStol::run()
{
    float pilot_throttle_scaled = quadplane.get_pilot_throttle();
    quadplane.hold_stabilize(pilot_throttle_scaled);
}

#endif
