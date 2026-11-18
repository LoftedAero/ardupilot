#include "Plane.h"

#if AP_PLANE_GYRO_MENU_ENABLED

#include <stdarg.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>

extern const AP_HAL::HAL& hal;

namespace {

// CRSF parameter entry types used by the menu
constexpr uint8_t CRSF_FLOAT = 8;
constexpr uint8_t CRSF_TEXT_SELECTION = 9;
constexpr uint8_t CRSF_INFO = 12;
constexpr uint8_t CRSF_COMMAND = 13;
constexpr uint8_t CRSF_HIDDEN = 0x80;

// CRSF command states, sent to and from the radio
constexpr uint8_t CMD_READY = 0;
constexpr uint8_t CMD_START = 1;
constexpr uint8_t CMD_PROGRESS = 2;
constexpr uint8_t CMD_CONFIRMATION_NEEDED = 3;
constexpr uint8_t CMD_CONFIRM = 4;
constexpr uint8_t CMD_CANCEL = 5;
constexpr uint8_t CMD_POLL = 6;

// raw input (us) of an RC channel, numbered from 0; 0 when it does not exist
uint16_t radio_in(uint8_t chan)
{
    const RC_Channel *c = rc().channel(chan);
    return c == nullptr ? 0 : c->get_radio_in();
}

// how often the radio polls a running command, in 10 ms units
constexpr uint8_t CMD_POLL_INTERVAL = 25;

// how far (us) a channel must move to count while learning
// stick learning, as the Loft GCS radio calibration: a stick counts once it
// is this far from its centre and this much further than any other stick,
// and a step completes once it is held there
constexpr int16_t STICK_DETECT_US = 180;
constexpr int16_t STICK_MARGIN_US = 90;
constexpr int16_t STICK_HOLD_US = 10;
constexpr uint32_t STICK_HOLD_MS = 500;
constexpr int16_t LEARN_SWITCH_DELTA = 300;
// learned stick and knob endpoints must be at least this far apart (us)
constexpr int16_t LEARN_MIN_SPAN = 600;
// a learned stick centre may be this far (percent of travel) from the middle
constexpr int16_t STICK_CENTRE_TOL_PCT = 5;
// trim copy: stick offsets above this (percent of throw) are a held stick
constexpr int16_t TRIM_COPY_MAX_PCT = 15;
// trim copy: TX trims count as centred within this
constexpr int16_t TRIM_CENTRED_PCT = 3;
// a command's reply to a write goes out in one frame with no chunks:
// its name and popup text together must stay within this
constexpr uint8_t CMD_TEXT_MAX = 50;
constexpr uint32_t LEARN_TIMEOUT_MS = 30000;
// a command is abandoned when the radio stops asking about it: it polls
// a running command several times a second, and is silent while the
// pilot reads a confirmation
constexpr uint32_t ABANDON_PROGRESS_MS = 10000;
constexpr uint32_t ABANDON_CONFIRM_MS = 120000;

struct Choice {
    const char *label;
    int16_t value;
};

const Choice mode_choices[] = {{"MANUAL", 0}, {"ACRO", 4}, {"FBWA", 5}, {"AUTOTUNE", 8}};
const Choice function_choices[] = {
    {"None", 0}, {"Aileron", 4}, {"Elevator", 19}, {"Rudder", 21}, {"Throttle", 70},
    {"Elevon L", 77}, {"Elevon R", 78}, {"V-tail L", 79}, {"V-tail R", 80},
    {"Flaperon L", 24}, {"Flaperon R", 25}, {"Flap", 2},
};
const Choice orientation_choices[] = {
    {"None", ROTATION_NONE}, {"Yaw 90", ROTATION_YAW_90}, {"Yaw 180", ROTATION_YAW_180},
    {"Yaw 270", ROTATION_YAW_270}, {"Roll 180", ROTATION_ROLL_180},
    {"Roll180 Yaw90", ROTATION_ROLL_180_YAW_90}, {"Pitch 180", ROTATION_PITCH_180},
    {"Roll180 Yaw270", ROTATION_ROLL_180_YAW_270},
};
const Choice link_loss_choices[] = {{"FBWA glide", 2}, {"RTL if GPS", 1}};
const Choice gps_choices[] = {{"Off", 0}, {"On", 1}};
// ARMING_SKIPCHK bits for the GPS and GPS_CONFIG checks: skipped while
// the GPS is off, checked while it is on
constexpr int32_t GPS_ARMING_CHECKS = (1U << 3) | (1U << 12);
const Choice reversed_choices[] = {{"Normal", 0}, {"Reversed", 1}};
const Choice channel_choices[] = {
    {"Off", 0}, {"Ch5", 5}, {"Ch6", 6}, {"Ch7", 7}, {"Ch8", 8}, {"Ch9", 9}, {"Ch10", 10},
    {"Ch11", 11}, {"Ch12", 12}, {"Ch13", 13}, {"Ch14", 14}, {"Ch15", 15}, {"Ch16", 16},
};

const char *const wing_labels[] = {"Normal", "Flying wing", "V-tail"};
constexpr uint8_t NUM_WINGS = ARRAY_SIZE(wing_labels);
const int16_t wing_functions[NUM_WINGS][4] = {
    {4, 19, 70, 21},    // aileron, elevator, throttle, rudder
    {77, 78, 70, 21},   // elevon left, elevon right, throttle, rudder
    {4, 79, 70, 80},    // aileron, V-tail left, throttle, V-tail right
};

// mountings made of right angles: the candidates for orientation detection
const uint8_t right_angle_rotations[] = {
    ROTATION_NONE, ROTATION_YAW_90, ROTATION_YAW_180, ROTATION_YAW_270,
    ROTATION_ROLL_180, ROTATION_ROLL_180_YAW_90, ROTATION_PITCH_180, ROTATION_ROLL_180_YAW_270,
    ROTATION_ROLL_90, ROTATION_ROLL_90_YAW_90, ROTATION_ROLL_270, ROTATION_ROLL_270_YAW_90,
    ROTATION_PITCH_90, ROTATION_PITCH_270, ROTATION_ROLL_90_PITCH_90, ROTATION_ROLL_180_PITCH_90,
    ROTATION_ROLL_270_PITCH_90, ROTATION_ROLL_90_PITCH_180, ROTATION_ROLL_270_PITCH_180,
    ROTATION_ROLL_90_PITCH_270, ROTATION_ROLL_180_PITCH_270, ROTATION_ROLL_270_PITCH_270,
    ROTATION_ROLL_90_PITCH_180_YAW_90, ROTATION_ROLL_90_YAW_270,
};

struct NumberSpec {
    const char *param;
    float min, max, step;
    uint8_t decimals;
    const char *unit;
};
enum : uint8_t {
    // gains and limits: these may change in flight
    NUM_GAIN_MIN, NUM_GAIN_MAX, NUM_ACRO_ROLL, NUM_ACRO_PITCH, NUM_ROLL_LIMIT,
    NUM_PITCH_MAX, NUM_PITCH_MIN, NUM_AUTOTUNE_LEVEL,
    NUM_ROLL_FF, NUM_ROLL_P, NUM_ROLL_I, NUM_ROLL_D,
    NUM_PITCH_FF, NUM_PITCH_P, NUM_PITCH_I, NUM_PITCH_D,
    NUM_YAW_DAMP, NUM_RUDDER_MIX,
    // settings: locked while armed
    NUM_RTL_ALT, NUM_CRUISE_SPEED, NUM_MIN_SPEED, NUM_MAX_SPEED,
};
// numbers that may change in flight: the gains and limits in 6 Tuning, not
// its speeds (with a GPS a new cruise speed rescales every gain)
bool number_in_flight(uint8_t arg)
{
    return arg <= NUM_RUDDER_MIX;
}
// the rate gain limits are wider than their documented ranges so that any
// value autotune or the stock defaults set (PTCH_RATE_P is 0.04) shows and keeps.
// No entry has more than 3 decimals: the ExpressLRS script caps a float's
// precision at 3 but still divides by its own 10^3, so a value sent with 4
// decimals would show 10 times too large.
const NumberSpec number_specs[] = {
    {"GYRO_GAIN_MIN", 0, 1, 0.05, 2, ""},
    {"GYRO_GAIN_MAX", 1, 3, 0.05, 2, ""},
    {"ACRO_ROLL_RATE", 30, 500, 10, 0, "d/s"},
    {"ACRO_PITCH_RATE", 30, 500, 10, 0, "d/s"},
    {"ROLL_LIMIT_DEG", 10, 85, 5, 0, "deg"},
    {"PTCH_LIM_MAX_DEG", 5, 60, 5, 0, "deg"},
    {"PTCH_LIM_MIN_DEG", -60, -5, 5, 0, "deg"},
    {"AUTOTUNE_LEVEL", 1, 10, 1, 0, ""},
    {"RLL_RATE_FF", 0, 3, 0.005, 3, ""},
    {"RLL_RATE_P", 0, 1, 0.005, 3, ""},
    {"RLL_RATE_I", 0, 1, 0.005, 3, ""},
    {"RLL_RATE_D", 0, 0.1, 0.001, 3, ""},
    {"PTCH_RATE_FF", 0, 3, 0.005, 3, ""},
    {"PTCH_RATE_P", 0, 1, 0.005, 3, ""},
    {"PTCH_RATE_I", 0, 1, 0.005, 3, ""},
    {"PTCH_RATE_D", 0, 0.1, 0.001, 3, ""},
    // the yaw damper; 0 turns it off, and the gain knob scales it
    {"YAW2SRV_DAMP", 0, 2, 0.05, 2, ""},
    {"KFF_RDDRMIX", 0, 1, 0.05, 2, ""},
    {"RTL_ALTITUDE", 20, 400, 10, 0, "m"},
    // the cruise speed is also written to AIRSPEED_CRUISE
    {"SCALING_SPEED", 6, 50, 1, 0, "m/s"},
    {"AIRSPEED_MIN", 5, 50, 1, 0, "m/s"},
    {"AIRSPEED_MAX", 6, 100, 1, 0, "m/s"},
};
static_assert(ARRAY_SIZE(number_specs) == NUM_MAX_SPEED + 1, "number_specs must follow the NUM_ order");

struct AuxSpec {
    const char *label;      // menu entry and messages
    const char *learn;      // learn command name
    const char *prompt;     // what to do while learning
    int16_t option;         // RCn_OPTION, or -1 for functions with their own parameter
};
const AuxSpec aux_specs[] = {
    {"Mode switch", "Learn mode switch", "Move the mode switch", -1},
    {"Arm switch", "Learn arm switch", "Flip the arm switch", 153},
    {"RTL switch", "Learn RTL switch", "Flip the RTL switch", 4},
    {"Autotune switch", "Learn autotune sw", "Flip the autotune switch", 107},
    {"Gain knob", "Learn gain knob", "Turn the gain knob", -1},
};

// sticks: 0 roll, 1 pitch, 2 yaw, 3 throttle
const char *const rcmap_params[] = {"RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_YAW", "RCMAP_THROTTLE"};
// stick learning steps, in the Loft GCS order: the first move of a stick
// finds its channel and direction (right, back and full throttle raise
// the value unless reversed), the second confirms it. The throttle stays
// up for the rudder steps.
struct StickStep {
    uint8_t stick;
    bool first;
    const char *prompt;
};
const StickStep stick_steps[] = {
    {3, true, "THROTTLE all the way up, hold"},
    {2, true, "Throttle up. RUDDER right, hold"},
    {2, false, "Throttle up. RUDDER left, hold"},
    {3, false, "THROTTLE all the way down, hold"},
    {0, true, "AILERON right, hold"},
    {0, false, "AILERON left, hold"},
    {1, true, "ELEVATOR back (toward you), hold"},
    {1, false, "ELEVATOR forward, hold"},
};
const char stick_letters[] = {'A', 'E', 'R', 'T'};

#if HAL_INS_ACCELCAL_ENABLED
// the accel calibration poses, in AP_AccelCal's order
const char *const accel_cal_poses[] = {
    "level", "LEFT wing down", "RIGHT wing down", "nose DOWN", "nose UP", "upside down",
};

// receives the accel calibration's result; it calibrates nothing itself
class AccelCalResult : public AP_AccelCal_Client {
public:
    enum class Result : uint8_t { NONE, SUCCESS, FAILED, CANCELLED };
    Result result;
private:
    AccelCalibrator *_acal_get_calibrator(uint8_t) override { return nullptr; }
    void _acal_save_calibrations() override {}
    void _acal_event_success() override { result = Result::SUCCESS; }
    void _acal_event_cancellation() override { result = Result::CANCELLED; }
    void _acal_event_failure() override { result = Result::FAILED; }
};
AccelCalResult accel_cal_result;
#endif

// control surface outputs: the ones a trim copy sets
bool is_surface(SRV_Channel::Function f)
{
    switch (f) {
    case SRV_Channel::k_aileron:
    case SRV_Channel::k_elevator:
    case SRV_Channel::k_rudder:
    case SRV_Channel::k_elevon_left:
    case SRV_Channel::k_elevon_right:
    case SRV_Channel::k_vtail_left:
    case SRV_Channel::k_vtail_right:
    case SRV_Channel::k_flaperon_left:
    case SRV_Channel::k_flaperon_right:
        return true;
    default:
        return false;
    }
}

// big-endian writer for CRSF parameter entries
class Packer {
public:
    Packer(uint8_t *buf, uint8_t size) : _buf(buf), _size(size) {}
    void u8(uint8_t v) {
        if (_len < _size) {
            _buf[_len++] = v;
        }
    }
    void i32(int32_t v) {
        const uint32_t u = uint32_t(v);
        u8(u >> 24);
        u8(u >> 16);
        u8(u >> 8);
        u8(u);
    }
    void str(const char *s) {
        while (*s) {
            u8(uint8_t(*s++));
        }
        u8(0);
    }
    uint8_t len() const { return _len; }
private:
    uint8_t *_buf;
    uint8_t _size;
    uint8_t _len = 0;
};

uint8_t choice_index(const Choice *choices, uint8_t count, int16_t value)
{
    for (uint8_t i = 0; i < count; i++) {
        if (choices[i].value == value) {
            return i;
        }
    }
    return count;   // "Other"
}

// selection entry; a value not in the list shows as an extra "Other" option
void pack_select(Packer &p, uint8_t flags, const char *name, const Choice *choices, uint8_t count, int16_t value)
{
    const uint8_t idx = choice_index(choices, count, value);
    p.u8(CRSF_TEXT_SELECTION | flags);
    p.str(name);
    for (uint8_t i = 0; i < count; i++) {
        if (i > 0) {
            p.u8(';');
        }
        for (const char *c = choices[i].label; *c; c++) {
            p.u8(uint8_t(*c));
        }
    }
    if (idx == count) {
        for (const char *c = ";Other"; *c; c++) {
            p.u8(uint8_t(*c));
        }
    }
    p.u8(0);
    p.u8(idx);                                  // value
    p.u8(0);                                    // min
    p.u8(idx == count ? count : count - 1);     // max
    p.u8(idx);                                  // default
    p.str("");                                  // unit
}

void pack_number(Packer &p, uint8_t flags, const char *name, float value, float min, float max,
                 float step, uint8_t decimals, const char *unit)
{
    float scale = 1;
    for (uint8_t i = 0; i < decimals; i++) {
        scale *= 10;
    }
    const int32_t v = int32_t(lroundf(value * scale));
    p.u8(CRSF_FLOAT | flags);
    p.str(name);
    p.i32(v);
    p.i32(int32_t(lroundf(min * scale)));
    p.i32(int32_t(lroundf(max * scale)));
    p.i32(v);                                   // default
    p.u8(decimals);
    p.i32(int32_t(lroundf(step * scale)));
    p.str(unit);
}

void pack_info(Packer &p, uint8_t flags, const char *name, const char *text)
{
    p.u8(CRSF_INFO | flags);
    p.str(name);
    p.str(text);
}

int32_t payload_i32(const uint8_t *payload)
{
    return int32_t((uint32_t(payload[0]) << 24) | (uint32_t(payload[1]) << 16) |
                   (uint32_t(payload[2]) << 8) | uint32_t(payload[3]));
}

}  // namespace

/*
  parameter helpers
 */
float PlaneGyroMenu::param_get(const char *name)
{
    float value = 0;
    AP_Param::get(name, value);
    return value;
}

void PlaneGyroMenu::param_set(const char *name, float value)
{
    AP_Param::set_and_save_by_name(name, value);
}

float PlaneGyroMenu::output_param(uint8_t output, const char *field)
{
    char name[17];
    hal.util->snprintf(name, sizeof(name), "SERVO%u_%s", unsigned(output), field);
    return param_get(name);
}

void PlaneGyroMenu::set_output_param(uint8_t output, const char *field, float value, bool save)
{
    char name[17];
    hal.util->snprintf(name, sizeof(name), "SERVO%u_%s", unsigned(output), field);
    if (save) {
        param_set(name, value);
    } else {
        AP_Param::set_by_name(name, value);
    }
}

bool PlaneGyroMenu::gps_present()
{
    return AP::gps().status() != AP_GPS_FixType::NO_GPS;
}

/*
  GPS off: no GPS driver and DCM, which flies without one. GPS on: the
  stock ArduPlane setup, EKF3 and the GPS arming checks, so the model
  arms only once EKF3 has started with a fix: EKF3 never starts in the
  air from a turn's tilted accelerations.
 */
bool PlaneGyroMenu::gps_fitted()
{
    return int8_t(param_get("GPS1_TYPE")) != 0;
}

bool PlaneGyroMenu::set_gps_fitted(bool on)
{
    if (on == gps_fitted()) {
        return false;
    }
    param_set("GPS1_TYPE", on ? 1 : 0);
    param_set("EK3_ENABLE", on ? 1 : 0);
    param_set("AHRS_EKF_TYPE", on ? 3 : 0);
    const int32_t skip = int32_t(param_get("ARMING_SKIPCHK"));
    param_set("ARMING_SKIPCHK", on ? (skip & ~GPS_ARMING_CHECKS) : (skip | GPS_ARMING_CHECKS));
    return true;
}

bool PlaneGyroMenu::armed()
{
    return plane.arming.is_armed();
}

void PlaneGyroMenu::mode_name(const Mode &mode, char *buf, uint8_t size)
{
    const char *name = mode.name();
    if (strlen(name) > 8) {
        name = mode.name4();
    }
    uint8_t i = 0;
    for (; name[i] != 0 && i + 1 < size; i++) {
        const char c = name[i];
        buf[i] = (c >= 'a' && c <= 'z') ? char(c - 'a' + 'A') : c;
    }
    buf[i] = 0;
}

/*
  switch functions
 */
uint8_t PlaneGyroMenu::aux_channel(uint8_t func)
{
    switch (func) {
    case AUX_MODE:
        return uint8_t(param_get("FLTMODE_CH"));
    case AUX_GAIN:
        return uint8_t(param_get("GYRO_GAIN_CH"));
    default:
        break;
    }
    for (uint8_t ch = 5; ch <= 16; ch++) {
        char name[17];
        hal.util->snprintf(name, sizeof(name), "RC%u_OPTION", unsigned(ch));
        if (int16_t(param_get(name)) == aux_specs[func].option) {
            return ch;
        }
    }
    return 0;
}

void PlaneGyroMenu::set_aux_channel(uint8_t func, uint8_t ch)
{
    switch (func) {
    case AUX_MODE:
        param_set("FLTMODE_CH", ch);
        return;
    case AUX_GAIN:
        param_set("GYRO_GAIN_CH", ch);
        return;
    default:
        break;
    }
    // only one channel per function
    for (uint8_t c = 5; c <= 16; c++) {
        char name[17];
        hal.util->snprintf(name, sizeof(name), "RC%u_OPTION", unsigned(c));
        if (c != ch && int16_t(param_get(name)) == aux_specs[func].option) {
            param_set(name, 0);
        }
    }
    if (ch != 0) {
        char name[17];
        hal.util->snprintf(name, sizeof(name), "RC%u_OPTION", unsigned(ch));
        param_set(name, aux_specs[func].option);
    }
}

// which of our switch functions uses a channel, or -1
int8_t PlaneGyroMenu::channel_owner(uint8_t ch, uint8_t except_func)
{
    if (ch == 0) {
        return -1;
    }
    for (uint8_t f = 0; f < AUX_COUNT; f++) {
        if (f != except_func && aux_channel(f) == ch) {
            return int8_t(f);
        }
    }
    return -1;
}

// true when a channel's switch is in its low (off) position
static bool switch_low(uint8_t ch)
{
    const RC_Channel *c = rc().channel(ch - 1);
    return c != nullptr && c->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::LOW;
}

/*
  wing templates
 */
uint8_t PlaneGyroMenu::detect_wing()
{
    for (uint8_t w = 0; w < NUM_WINGS; w++) {
        bool match = true;
        for (uint8_t i = 0; i < 4; i++) {
            if (int16_t(output_param(i + 1, "FUNCTION")) != wing_functions[w][i]) {
                match = false;
            }
        }
        if (match) {
            return w;
        }
    }
    return NUM_WINGS;
}

void PlaneGyroMenu::apply_wing(uint8_t wing)
{
    for (uint8_t i = 0; i < 4; i++) {
        set_output_param(i + 1, "FUNCTION", wing_functions[wing][i]);
    }
}

/*
  find the right-angle mounting under which the level sample points down
  and the nose-up sample has gravity moving towards the tail
 */
bool PlaneGyroMenu::orientation_from(const Vector3f &level, const Vector3f &nose_up, uint8_t &rotation) const
{
    const float g = GRAVITY_MSS;
    for (const uint8_t r : right_angle_rotations) {
        Vector3f a = level;
        Vector3f b = nose_up;
        a.rotate(Rotation(r));
        b.rotate(Rotation(r));
        // level: specific force points up (-z); nose up: it gains a +x component
        if (a.z < -0.8f * g && fabsf(a.x) < 0.3f * g && fabsf(a.y) < 0.3f * g &&
            b.x > 0.3f * g && fabsf(b.y) < 0.3f * g) {
            rotation = r;
            return true;
        }
    }
    return false;
}

/*
  menu construction
 */
void PlaneGyroMenu::add(AP_CRSF_Telem::ScriptedMenu *menu, Handler handler, uint8_t arg, const char *name)
{
    if (menu == nullptr || num_items >= MAX_ITEMS) {
        return;
    }
    Item &item = items[num_items];
    item.handler = handler;
    item.arg = arg;
    item.name = name;
    uint8_t buf[200];
    const uint8_t len = pack(item, buf, sizeof(buf));
    AP_CRSF_Telem::ScriptedParameter *param = menu->add_parameter(len, (const char *)buf);
    if (param == nullptr) {
        return;
    }
    item.id = param->id;
    num_items++;
}

AP_CRSF_Telem::ScriptedMenu *PlaneGyroMenu::submenu(AP_CRSF_Telem::ScriptedMenu *parent, const char *name)
{
    if (parent == nullptr) {
        return nullptr;
    }
    return parent->add_menu(name, 2, parent->id);
}

bool PlaneGyroMenu::build()
{
    AP_CRSF_Telem *crsf = AP::crsf_telem();
    if (crsf == nullptr) {
        return false;
    }
    // the ExpressLRS script returns to the top level on Back from any
    // folder, so the folders sit at the top level rather than in one menu
    AP_CRSF_Telem::ScriptedMenu *root = &crsf->scripted_menus;

    add(root, Handler::STATUS, 0, "Status");

    // the sensors fitted and their calibration. The gyros calibrate at
    // each start (INS_GYR_CAL 1)
    AP_CRSF_Telem::ScriptedMenu *m = submenu(root, "1 Sensors");
    add(m, Handler::ORIENT, 0, "Orientation");
    add(m, Handler::ORIENT_DETECT, 0, "Detect orientation");
#if HAL_INS_ACCELCAL_ENABLED
    add(m, Handler::CAL_ACCEL, 0, "Calibrate accel");
#endif
    add(m, Handler::CAL_LEVEL, 0, "Calibrate level");
    add(m, Handler::GPS_FITTED, 0, "GPS");

    m = submenu(root, "2 Radio");
    add(m, Handler::LEARN_STICKS, 0, "Learn sticks");
    for (const uint8_t f : {AUX_ARM, AUX_MODE, AUX_RTL, AUX_AUTOTUNE, AUX_GAIN}) {
        add(m, Handler::AUX_CH, f, aux_specs[f].label);
        add(m, Handler::AUX_LEARN, f, aux_specs[f].learn);
    }

    m = submenu(root, "3 Flight modes");
    add(m, Handler::MODE_CHECK, 0, "Check mode switch");
    add(m, Handler::MODE_POS, 0, "Low");
    add(m, Handler::MODE_POS, 1, "Middle");
    add(m, Handler::MODE_POS, 2, "High");

    // the wing type sits above the channels it sets
    m = submenu(root, "4 Outputs");
    add(m, Handler::WING_TYPE, 0, "Wing type");
    add(m, Handler::TRIM_COPY, 0, "Copy stick trims");
    static const char *const output_names[NUM_OUTPUTS] = {
        "Channel 1", "Channel 2", "Channel 3", "Channel 4", "Channel 5", "Channel 6", "Channel 7", "Channel 8",
    };
    for (uint8_t n = 1; n <= NUM_OUTPUTS; n++) {
        AP_CRSF_Telem::ScriptedMenu *o = submenu(m, output_names[n - 1]);
        add(o, Handler::OUT_FUNC, n, "Function");
        add(o, Handler::OUT_MIN, n, "Min");
        add(o, Handler::OUT_TRIM, n, "Trim");
        add(o, Handler::OUT_MAX, n, "Max");
        add(o, Handler::OUT_REV, n, "Reversed");
    }

    m = submenu(root, "5 Failsafe");
    add(m, Handler::LINK_LOSS, 0, "Link loss");
    add(m, Handler::NUMBER, NUM_RTL_ALT, "RTL altitude");
    add(m, Handler::NO_GPS_INFO, 0, "RTL");

    m = submenu(root, "6 Tuning");
    add(m, Handler::NUMBER, NUM_GAIN_MIN, "Gain knob min");
    add(m, Handler::NUMBER, NUM_GAIN_MAX, "Gain knob max");
    add(m, Handler::NUMBER, NUM_ROLL_LIMIT, "FBWA bank limit");
    add(m, Handler::NUMBER, NUM_PITCH_MAX, "FBWA pitch up");
    add(m, Handler::NUMBER, NUM_PITCH_MIN, "FBWA pitch down");
    add(m, Handler::NUMBER, NUM_AUTOTUNE_LEVEL, "Autotune level");
    // the speeds are locked while armed (see number_in_flight)
    AP_CRSF_Telem::ScriptedMenu *g = submenu(m, "Speeds");
    add(g, Handler::NUMBER, NUM_CRUISE_SPEED, "Cruise speed");
    add(g, Handler::NUMBER, NUM_MIN_SPEED, "Min speed");
    add(g, Handler::NUMBER, NUM_MAX_SPEED, "Max speed");
    // the rate gains autotune learns (FF, P, D) and sets (I), and the ACRO rate
    static const char *const gain_names[] = {"FF", "P", "I", "D"};
    g = submenu(m, "Roll gains");
    for (uint8_t i = 0; i < 4; i++) {
        add(g, Handler::NUMBER, NUM_ROLL_FF + i, gain_names[i]);
    }
    add(g, Handler::NUMBER, NUM_ACRO_ROLL, "ACRO roll rate");
    g = submenu(m, "Pitch gains");
    for (uint8_t i = 0; i < 4; i++) {
        add(g, Handler::NUMBER, NUM_PITCH_FF + i, gain_names[i]);
    }
    add(g, Handler::NUMBER, NUM_ACRO_PITCH, "ACRO pitch rate");
    g = submenu(m, "Yaw gains");
    add(g, Handler::NUMBER, NUM_YAW_DAMP, "Yaw damper");
    add(g, Handler::NUMBER, NUM_RUDDER_MIX, "Rudder mix");

    m = submenu(root, "Tools");
    add(m, Handler::FIRMWARE, 0, "Firmware");
    add(m, Handler::RESET, 0, "Reset to defaults");
    // last, so adding it numbered no other entry differently
    add(m, Handler::MENU_VERSION, 0, "Menu version");

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "StabRX: setup menu ready (%u entries)", unsigned(num_items));
    return true;
}

PlaneGyroMenu::Item *PlaneGyroMenu::find_item(uint8_t id)
{
    for (uint8_t i = 0; i < num_items; i++) {
        if (items[i].id == id) {
            return &items[i];
        }
    }
    return nullptr;
}

/*
  entries as the radio sees them
 */
uint8_t PlaneGyroMenu::pack(const Item &item, uint8_t *buf, uint8_t size)
{
    Packer p(buf, size);
    const uint8_t index = uint8_t(&item - items);
    const uint8_t no_gps_hidden = gps_present() ? 0 : CRSF_HIDDEN;

    switch (item.handler) {
    case Handler::STATUS: {
        char text[40];
        char gps[16];
        switch (AP::gps().status()) {
        case AP_GPS_FixType::NO_GPS:
            strncpy(gps, "none", sizeof(gps));
            break;
        case AP_GPS_FixType::NONE:
            strncpy(gps, "no fix", sizeof(gps));
            break;
        case AP_GPS_FixType::FIX_2D:
            strncpy(gps, "2D", sizeof(gps));
            break;
        default:
            hal.util->snprintf(gps, sizeof(gps), "%u sats", unsigned(AP::gps().num_sats()));
            break;
        }
        char mode[10];
        mode_name(*plane.control_mode, mode, sizeof(mode));
        hal.util->snprintf(text, sizeof(text), "%s %s GPS %s", armed() ? "ARMED" : "DISARMED", mode, gps);
        pack_info(p, 0, item.name, text);
        break;
    }
    case Handler::FIRMWARE:
        pack_info(p, 0, item.name, AP::fwversion().fw_short_string);
        break;
    case Handler::MENU_VERSION:
        // Hidden from the ExpressLRS script. A radio app with screens built
        // on this menu (Lofty PFD's Gyro Setup) uses them only for a version
        // it knows; raise it when an entry it relies on changes name, kind,
        // place or meaning.
        pack_info(p, CRSF_HIDDEN, item.name, "4");
        break;
    case Handler::NO_GPS_INFO:
        pack_info(p, gps_present() ? CRSF_HIDDEN : 0, item.name, "needs a GPS");
        break;
    case Handler::WING_TYPE: {
        // read from channels 1-4, so a change made channel by channel shows
        // as "Other"
        static const Choice wings[] = {{"Normal", 0}, {"Flying wing", 1}, {"V-tail", 2}};
        pack_select(p, 0, item.name, wings, NUM_WINGS, detect_wing());
        break;
    }
    case Handler::ORIENT:
        pack_select(p, 0, item.name, orientation_choices, ARRAY_SIZE(orientation_choices),
                    int16_t(param_get("AHRS_ORIENTATION")));
        break;
    case Handler::AUX_CH:
        pack_select(p, item.arg == AUX_RTL ? no_gps_hidden : 0, item.name, channel_choices,
                    ARRAY_SIZE(channel_choices), aux_channel(item.arg));
        break;
    case Handler::MODE_POS: {
        static const char *const fltmode[] = {"FLTMODE1", "FLTMODE4", "FLTMODE6"};
        pack_select(p, 0, item.name, mode_choices, ARRAY_SIZE(mode_choices),
                    int16_t(param_get(fltmode[item.arg])));
        break;
    }
    case Handler::OUT_FUNC:
        pack_select(p, 0, item.name, function_choices, ARRAY_SIZE(function_choices),
                    int16_t(output_param(item.arg, "FUNCTION")));
        break;
    case Handler::OUT_MIN:
        pack_number(p, 0, item.name, output_param(item.arg, "MIN"), 500, 2500, 10, 0, "us");
        break;
    case Handler::OUT_TRIM:
        pack_number(p, 0, item.name, output_param(item.arg, "TRIM"), 500, 2500, 5, 0, "us");
        break;
    case Handler::OUT_MAX:
        pack_number(p, 0, item.name, output_param(item.arg, "MAX"), 500, 2500, 10, 0, "us");
        break;
    case Handler::OUT_REV:
        pack_select(p, 0, item.name, reversed_choices, ARRAY_SIZE(reversed_choices),
                    int16_t(output_param(item.arg, "REVERSED")));
        break;
    case Handler::LINK_LOSS:
        pack_select(p, 0, item.name, link_loss_choices, ARRAY_SIZE(link_loss_choices),
                    int16_t(param_get("FS_LONG_ACTN")));
        break;
    case Handler::GPS_FITTED:
        pack_select(p, 0, item.name, gps_choices, ARRAY_SIZE(gps_choices), gps_fitted() ? 1 : 0);
        break;
    case Handler::NUMBER: {
        const NumberSpec &n = number_specs[item.arg];
        const uint8_t flags = item.arg == NUM_RTL_ALT ? no_gps_hidden : 0;
        pack_number(p, flags, item.name, param_get(n.param), n.min, n.max, n.step, n.decimals, n.unit);
        break;
    }
    case Handler::TRIM_COPY:
    case Handler::ORIENT_DETECT:
    case Handler::CAL_ACCEL:
    case Handler::CAL_LEVEL:
    case Handler::LEARN_STICKS:
    case Handler::AUX_LEARN:
    case Handler::MODE_CHECK:
    case Handler::RESET: {
        const bool active = cmd.item == index;
        uint8_t flags = 0;
        if (item.handler == Handler::AUX_LEARN && item.arg == AUX_RTL) {
            flags = no_gps_hidden;
        }
        p.u8(CRSF_COMMAND | flags);
        p.str(item.name);
        p.u8(active ? cmd.status : CMD_READY);
        p.u8(CMD_POLL_INTERVAL);
        // clip the text rather than lose the reply
        char info[sizeof(cmd.info)] {};
        if (active) {
            const size_t name_len = strlen(item.name);
            const size_t room = name_len < CMD_TEXT_MAX ? CMD_TEXT_MAX - name_len : 0;
            memcpy(info, cmd.info, MIN(strnlen(cmd.info, sizeof(cmd.info)), MIN(room, sizeof(info) - 1)));
        }
        p.str(info);
        break;
    }
    }
    return p.len();
}

/*
  writes from the radio
 */
uint8_t PlaneGyroMenu::write(const Item &item, const uint8_t *payload, uint8_t length, uint8_t *buf, uint8_t size)
{
    const uint8_t index = uint8_t(&item - items);
    Packer p(buf, size);

    switch (item.handler) {
    case Handler::TRIM_COPY:
    case Handler::ORIENT_DETECT:
    case Handler::CAL_ACCEL:
    case Handler::CAL_LEVEL:
    case Handler::LEARN_STICKS:
    case Handler::AUX_LEARN:
    case Handler::MODE_CHECK:
    case Handler::RESET:
        // commands answer with their full entry: status and text
        if (length >= 1) {
            command(index, payload[0]);
        }
        return pack(item, buf, size);
    default:
        break;
    }

    // in flight only the tuning numbers can change: a wrong output or
    // radio setting takes effect at once. Everything else answers with
    // its value unchanged.
    const bool tuning = item.handler == Handler::NUMBER && number_in_flight(item.arg);
    const bool locked = armed() && !tuning;
    if (locked) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "StabRX: disarm to change %s", item.name);
    }

    // number entries carry a 4 byte value
    switch (item.handler) {
    case Handler::OUT_MIN:
    case Handler::OUT_TRIM:
    case Handler::OUT_MAX:
    case Handler::NUMBER: {
        if (length >= 4 && !locked) {
            const int32_t raw = payload_i32(payload);
            if (item.handler == Handler::NUMBER) {
                const NumberSpec &n = number_specs[item.arg];
                float scale = 1;
                for (uint8_t i = 0; i < n.decimals; i++) {
                    scale *= 10;
                }
                float value = constrain_float(raw / scale, n.min, n.max);
                // the speeds stay in order: min below cruise below max
                const float cruise = param_get(number_specs[NUM_CRUISE_SPEED].param);
                if (item.arg == NUM_MIN_SPEED) {
                    value = MIN(value, cruise - 1);
                } else if (item.arg == NUM_MAX_SPEED) {
                    value = MAX(value, cruise + 1);
                } else if (item.arg == NUM_CRUISE_SPEED) {
                    value = MAX(MIN(value, param_get(number_specs[NUM_MAX_SPEED].param) - 1),
                                param_get(number_specs[NUM_MIN_SPEED].param) + 1);
                }
                param_set(n.param, value);
                if (item.arg == NUM_CRUISE_SPEED) {
                    param_set("AIRSPEED_CRUISE", value);
                }
            } else {
                const char *field = item.handler == Handler::OUT_MIN ? "MIN" :
                                    item.handler == Handler::OUT_TRIM ? "TRIM" : "MAX";
                set_output_param(item.arg, field, constrain_int32(raw, 500, 2500));
            }
        }
        // answer with the value now in effect
        float value;
        uint8_t decimals = 0;
        if (item.handler == Handler::NUMBER) {
            value = param_get(number_specs[item.arg].param);
            decimals = number_specs[item.arg].decimals;
        } else {
            const char *field = item.handler == Handler::OUT_MIN ? "MIN" :
                                item.handler == Handler::OUT_TRIM ? "TRIM" : "MAX";
            value = output_param(item.arg, field);
        }
        float scale = 1;
        for (uint8_t i = 0; i < decimals; i++) {
            scale *= 10;
        }
        p.i32(int32_t(lroundf(value * scale)));
        return p.len();
    }
    default:
        break;
    }

    // everything else is a selection carrying a 1 byte index
    const uint8_t idx = (length >= 1 && !locked) ? payload[0] : 0xFF;
    switch (item.handler) {
    case Handler::WING_TYPE: {
        // takes effect at once, as every output entry does
        if (idx < NUM_WINGS) {
            apply_wing(idx);
        }
        const uint8_t now = detect_wing();
        p.u8(now < NUM_WINGS ? now : NUM_WINGS);
        break;
    }
    case Handler::ORIENT:
        if (idx < ARRAY_SIZE(orientation_choices)) {
            param_set("AHRS_ORIENTATION", orientation_choices[idx].value);
        }
        p.u8(choice_index(orientation_choices, ARRAY_SIZE(orientation_choices),
                          int16_t(param_get("AHRS_ORIENTATION"))));
        break;
    case Handler::AUX_CH: {
        if (idx < ARRAY_SIZE(channel_choices)) {
            const uint8_t ch = uint8_t(channel_choices[idx].value);
            const bool in_use = channel_owner(ch, item.arg) >= 0;
            // never put the arm function on a switch that is not off: it would arm
            const bool unsafe = item.arg == AUX_ARM && ch != 0 && !switch_low(ch);
            if (!in_use && !unsafe) {
                set_aux_channel(item.arg, ch);
            }
        }
        p.u8(choice_index(channel_choices, ARRAY_SIZE(channel_choices), aux_channel(item.arg)));
        break;
    }
    case Handler::MODE_POS: {
        static const char *const pairs[3][2] = {
            {"FLTMODE1", "FLTMODE2"}, {"FLTMODE3", "FLTMODE4"}, {"FLTMODE5", "FLTMODE6"},
        };
        if (idx < ARRAY_SIZE(mode_choices)) {
            param_set(pairs[item.arg][0], mode_choices[idx].value);
            param_set(pairs[item.arg][1], mode_choices[idx].value);
        }
        p.u8(choice_index(mode_choices, ARRAY_SIZE(mode_choices), int16_t(param_get(pairs[item.arg][1]))));
        break;
    }
    case Handler::OUT_FUNC:
        if (idx < ARRAY_SIZE(function_choices)) {
            set_output_param(item.arg, "FUNCTION", function_choices[idx].value);
        }
        p.u8(choice_index(function_choices, ARRAY_SIZE(function_choices),
                          int16_t(output_param(item.arg, "FUNCTION"))));
        break;
    case Handler::OUT_REV:
        if (idx < ARRAY_SIZE(reversed_choices)) {
            set_output_param(item.arg, "REVERSED", reversed_choices[idx].value);
        }
        p.u8(choice_index(reversed_choices, ARRAY_SIZE(reversed_choices),
                          int16_t(output_param(item.arg, "REVERSED"))));
        break;
    case Handler::LINK_LOSS:
        if (idx < ARRAY_SIZE(link_loss_choices)) {
            param_set("FS_LONG_ACTN", link_loss_choices[idx].value);
        }
        p.u8(choice_index(link_loss_choices, ARRAY_SIZE(link_loss_choices), int16_t(param_get("FS_LONG_ACTN"))));
        break;
    case Handler::GPS_FITTED:
        if (idx < ARRAY_SIZE(gps_choices) && set_gps_fitted(gps_choices[idx].value != 0)) {
            // the GPS driver and EKF3 start at boot
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "StabRX: GPS %s, restarting", gps_fitted() ? "on" : "off");
            reboot_at_ms = AP_HAL::millis() + 1500;
        }
        p.u8(gps_fitted() ? 1 : 0);
        break;
    default:
        // info entries are read only
        break;
    }
    return p.len();
}

/*
  commands: a popup on the radio that walks through steps
 */
void PlaneGyroMenu::command_prompt(uint8_t status, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(cmd.info, sizeof(cmd.info), fmt, ap);
    va_end(ap);
    cmd.status = status;
}

// show a result; the ELRS script closes a popup as soon as a command
// reports READY, so results are shown as a confirmation the pilot dismisses
void PlaneGyroMenu::command_finish(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(cmd.info, sizeof(cmd.info), fmt, ap);
    va_end(ap);
    cmd.status = CMD_CONFIRMATION_NEEDED;
    cmd.stage = Stage::DONE;
}

bool PlaneGyroMenu::text_fits() const
{
    return strlen(items[cmd.item].name) + strnlen(cmd.info, sizeof(cmd.info)) <= CMD_TEXT_MAX;
}

void PlaneGyroMenu::command_close()
{
    cmd.status = CMD_READY;
    cmd.info[0] = 0;
    cmd.stage = Stage::DONE;
}

void PlaneGyroMenu::command(uint8_t index, uint8_t action)
{
    if (action == CMD_START) {
        command_start(index);
        return;
    }
    if (cmd.item != index) {
        // a late poll or answer for a command that is no longer open
        return;
    }
    cmd.request_ms = AP_HAL::millis();
    switch (action) {
    case CMD_CONFIRM:
        command_confirm();
        break;
    case CMD_POLL:
        command_poll();
        break;
    case CMD_CANCEL:
        command_cancel();
        break;
    default:
        break;
    }
}

void PlaneGyroMenu::restore_trims()
{
    bool restored = false;
    for (uint8_t n = 1; n <= NUM_OUTPUTS; n++) {
        if (cmd.saved_trim[n - 1] != 0) {
            // the copy was never stored, so only the value in use goes back
            set_output_param(n, "TRIM", cmd.saved_trim[n - 1], false);
            cmd.saved_trim[n - 1] = 0;
            restored = true;
        }
    }
    if (restored) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "StabRX: trim copy undone");
    }
}

void PlaneGyroMenu::abandon()
{
    restore_trims();
#if HAL_INS_ACCELCAL_ENABLED
    if (cmd.stage == Stage::ACAL_POSE || cmd.stage == Stage::ACAL_SAMPLE) {
        AP_AccelCal *acal = AP::ins().get_acal();
        if (acal != nullptr && acal->running()) {
            // ends with the offsets as they were
            acal->cancel();
        }
    }
#endif
}

#if HAL_INS_ACCELCAL_ENABLED
void PlaneGyroMenu::accel_cal_pose_prompt()
{
    command_prompt(CMD_CONFIRMATION_NEEDED, "Pose %u/6: %s, then OK", unsigned(cmd.step + 1),
                   accel_cal_poses[cmd.step]);
    if (!text_fits()) {
        command_prompt(CMD_CONFIRMATION_NEEDED, "%u/6: %s, OK", unsigned(cmd.step + 1),
                       accel_cal_poses[cmd.step]);
    }
}

/*
  the accel calibration runs in the main loop (AP_Vehicle's
  accel_cal_update). Pass it each pose the pilot confirms, as a ground
  station does with MAV_CMD_ACCELCAL_VEHICLE_POS, and show its progress.
 */
void PlaneGyroMenu::accel_cal_update()
{
    AP_AccelCal *acal = AP::ins().get_acal();
    if (acal == nullptr) {
        return;
    }
    switch (accel_cal_result.result) {
    case AccelCalResult::Result::SUCCESS:
        // the new offsets apply after a restart (stock pre-arm check)
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "StabRX: accel calibration done, restarting");
        reboot_at_ms = AP_HAL::millis() + 1500;
        command_finish("Done, restarting");
        return;
    case AccelCalResult::Result::FAILED:
        command_finish("Failed: hold each pose still");
        return;
    case AccelCalResult::Result::CANCELLED:
        command_finish("Cancelled");
        return;
    case AccelCalResult::Result::NONE:
        break;
    }
    if (cmd.stage != Stage::ACAL_SAMPLE) {
        return;
    }
    if (!cmd.acal_accepted) {
        // refused until the calibration is ready for this pose
        cmd.acal_accepted = acal->gcs_vehicle_position(cmd.step + 1);
        if (!cmd.acal_accepted && AP_HAL::millis() - cmd.acal_ms > 5000) {
            if (acal->running()) {
                acal->cancel();
            }
            command_finish("Not ready, try again");
        }
        return;
    }
    const accel_cal_status_t status = acal->get_status();
    if (status == ACCEL_CAL_COLLECTING_SAMPLE) {
        cmd.acal_sampling = true;
    } else if (status == ACCEL_CAL_WAITING_FOR_ORIENTATION && cmd.acal_sampling) {
        cmd.step++;
        if (cmd.step < ARRAY_SIZE(accel_cal_poses)) {
            cmd.stage = Stage::ACAL_POSE;
            accel_cal_pose_prompt();
        }
    } else if (cmd.acal_sampling && cmd.step + 1U >= ARRAY_SIZE(accel_cal_poses)) {
        // all poses taken: the fit and the save follow
        command_prompt(CMD_PROGRESS, "Calculating");
    }
}
#endif

const RC_Channel *PlaneGyroMenu::stick_off_centre(int16_t percent)
{
    // raw values: a trim inside the stick dead zone still moves the
    // surfaces in MANUAL, so it must count here too
    for (const RC_Channel *c : {plane.channel_roll, plane.channel_pitch, plane.channel_rudder}) {
        const int32_t half_throw = MAX((c->get_radio_max() - c->get_radio_min()) / 2, 1);
        if (abs(int32_t(c->get_radio_in()) - c->get_radio_trim()) * 100 > percent * half_throw) {
            return c;
        }
    }
    return nullptr;
}

void PlaneGyroMenu::command_cancel()
{
    abandon();
    command_close();
}

void PlaneGyroMenu::command_start(uint8_t index)
{
    // abandon whatever was open before
    if (cmd.item >= 0 && cmd.stage != Stage::DONE) {
        abandon();
    }
    cmd = {};
    cmd.item = index;
    cmd.start_ms = AP_HAL::millis();
    cmd.request_ms = cmd.start_ms;

    const Item &item = items[index];
    if (item.handler == Handler::MODE_CHECK) {
        // only shows the switch, so it also works armed
        if (aux_channel(AUX_MODE) == 0) {
            command_finish("No mode switch: set it in Radio");
            return;
        }
        cmd.stage = Stage::MODE_WATCH;
        mode_watch_prompt();
        return;
    }
    if (armed()) {
        command_finish("Disarm first");
        return;
    }

    switch (item.handler) {
    case Handler::TRIM_COPY:
        if (plane.control_mode != &plane.mode_manual) {
            command_finish("Switch to MANUAL first");
            return;
        }
        if (!rc().has_valid_input()) {
            command_finish("No RC signal");
            return;
        }
        cmd.stage = Stage::CONFIRM;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Trims set? Hands off sticks");
        break;
    case Handler::ORIENT_DETECT:
        cmd.stage = Stage::ORIENT_LEVEL;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Hold model level, nose forward");
        break;
#if HAL_INS_ACCELCAL_ENABLED
    case Handler::CAL_ACCEL:
        if (AP::ins().get_acal() != nullptr && AP::ins().get_acal()->running()) {
            command_finish("Already running from a GCS");
            return;
        }
        // forget the result of an earlier run
        accel_cal_result.result = AccelCalResult::Result::NONE;
        cmd.stage = Stage::ACAL_POSE;
        accel_cal_pose_prompt();
        break;
#endif
    case Handler::CAL_LEVEL:
        cmd.stage = Stage::CONFIRM;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Hold model at level attitude");
        break;
    case Handler::RESET:
        cmd.stage = Stage::CONFIRM;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Erase ALL settings and restart?");
        break;
    case Handler::LEARN_STICKS:
    case Handler::AUX_LEARN:
        if (!rc().has_valid_input()) {
            command_finish("No RC signal");
            return;
        }
        if (item.handler == Handler::AUX_LEARN && item.arg == AUX_RTL && !gps_present()) {
            command_finish("RTL needs a GPS");
            return;
        }
        for (uint8_t ch = 0; ch < ARRAY_SIZE(cmd.base); ch++) {
            cmd.base[ch] = radio_in(ch);
        }
        if (item.handler == Handler::LEARN_STICKS) {
            // the centres are taken at the OK: stick trims become RCn_TRIM
            cmd.stage = Stage::STICK_CENTER;
            command_prompt(CMD_CONFIRMATION_NEEDED, "Center sticks & trims, throttle low");
        } else {
            // flipping an assigned arm switch here does not arm: arming
            // is refused while a command is open (busy())
            cmd.stage = Stage::AUX_MOVE;
            command_prompt(CMD_PROGRESS, "%s", aux_specs[item.arg].prompt);
        }
        break;
    default:
        command_finish("Not a command");
        break;
    }
}

// the mode switch position, the mode it selects and the mode flown now
void PlaneGyroMenu::mode_watch_prompt()
{
    const uint8_t ch = aux_channel(AUX_MODE);
    const uint16_t pwm = radio_in(ch - 1);
    if (!rc().has_valid_input() || pwm <= RC_Channel::RC_MIN_LIMIT_PWM || pwm >= RC_Channel::RC_MAX_LIMIT_PWM) {
        command_prompt(CMD_PROGRESS, "Ch%u: no signal", unsigned(ch));
        return;
    }
    // ArduPilot's six mode switch positions, in pairs for a 3 position switch
    uint8_t position;
    if (pwm < 1231) {
        position = 0;
    } else if (pwm < 1361) {
        position = 1;
    } else if (pwm < 1491) {
        position = 2;
    } else if (pwm < 1621) {
        position = 3;
    } else if (pwm < 1750) {
        position = 4;
    } else {
        position = 5;
    }
    static const char *const position_names[] = {"Low", "Middle", "High"};
    char param[10];
    hal.util->snprintf(param, sizeof(param), "FLTMODE%u", unsigned(position + 1));
    const int16_t selected = int16_t(param_get(param));
    const Mode *selected_mode = plane.mode_from_mode_num(Mode::Number(selected));
    char selected_name[10] = "?";
    if (selected_mode != nullptr) {
        mode_name(*selected_mode, selected_name, sizeof(selected_name));
    }
    if (selected == int16_t(plane.control_mode->mode_number())) {
        command_prompt(CMD_PROGRESS, "Ch%u %s: %s", unsigned(ch), position_names[position / 2], selected_name);
    } else {
        char current[10];
        mode_name(*plane.control_mode, current, sizeof(current));
        command_prompt(CMD_PROGRESS, "Ch%u %s: %s, now %s", unsigned(ch), position_names[position / 2],
                       selected_name, current);
    }
}

void PlaneGyroMenu::step_prompt()
{
    command_prompt(CMD_PROGRESS, "%s", stick_steps[cmd.step].prompt);
}

void PlaneGyroMenu::learn_sticks_step()
{
    const uint32_t now = AP_HAL::millis();
    if (now - cmd.start_ms > LEARN_TIMEOUT_MS) {
        command_finish("Timed out");
        return;
    }
    // how long each channel has been held where it is
    int16_t delta[4];
    for (uint8_t i = 0; i < 4; i++) {
        const uint16_t v = radio_in(i);
        if (abs(int32_t(v) - cmd.held_val[i]) > STICK_HOLD_US) {
            cmd.held_val[i] = v;
            cmd.held_ms[i] = now;
        }
        delta[i] = int16_t(v) - int16_t(cmd.base[i]);
    }

    const StickStep &step = stick_steps[cmd.step];
    uint8_t ch = 0;
    if (step.first) {
        // the stick channel not yet learned that moved furthest, clearly ahead of the rest
        int16_t best = 0, second = 0;
        for (uint8_t c = 1; c <= 4; c++) {
            bool used = false;
            for (const StickStep &done : stick_steps) {
                if (&done == &step) {
                    break;
                }
                used |= done.first && cmd.stick_ch[done.stick] == c;
            }
            const int16_t d = abs(delta[c - 1]);
            if (used) {
                continue;
            }
            if (d > best) {
                second = best;
                best = d;
                ch = c;
            } else if (d > second) {
                second = d;
            }
        }
        if (ch == 0 || best < STICK_DETECT_US || best - second < STICK_MARGIN_US) {
            return;
        }
    } else {
        // the learned channel, back past its centre the other way (the
        // throttle: back near its low end)
        ch = cmd.stick_ch[step.stick];
        const int16_t d = delta[ch - 1];
        const int16_t span = int16_t(cmd.hi[ch - 1]) - int16_t(cmd.lo[ch - 1]);
        const bool first_up = !cmd.stick_reversed[step.stick];
        const bool back = step.stick == 3 ? abs(d) * 5 <= span : (first_up ? d <= -STICK_DETECT_US : d >= STICK_DETECT_US);
        if (!back) {
            return;
        }
    }
    if (now - cmd.held_ms[ch - 1] < STICK_HOLD_MS) {
        return;
    }
    if (step.first) {
        cmd.stick_ch[step.stick] = ch;
        cmd.stick_reversed[step.stick] = delta[ch - 1] < 0;
    }
    cmd.start_ms = now;
    if (++cmd.step < ARRAY_SIZE(stick_steps)) {
        step_prompt();
        return;
    }

    // all steps done: check the travel and centres before offering to save
    for (uint8_t s = 0; s < 4; s++) {
        const uint8_t i = cmd.stick_ch[s] - 1;
        if (cmd.hi[i] - cmd.lo[i] < LEARN_MIN_SPAN) {
            command_finish("Ch%u travel too short, retry", unsigned(i + 1));
            return;
        }
    }
    for (uint8_t s = 0; s < 3; s++) {
        // roll, pitch and yaw centre: where the stick sat at the start
        const uint8_t i = cmd.stick_ch[s] - 1;
        const int32_t mid = (int32_t(cmd.lo[i]) + cmd.hi[i]) / 2;
        if (abs(int32_t(cmd.base[i]) - mid) * 100 > STICK_CENTRE_TOL_PCT * (cmd.hi[i] - cmd.lo[i])) {
            command_finish("Ch%u not centred at start, retry", unsigned(i + 1));
            return;
        }
    }
    // show the endpoints before offering to save
    cmd.stage = Stage::STICK_REVIEW;
    cmd.step = 0;
    review_prompt();
}

// two sticks' endpoints per page: aileron and elevator, then rudder and throttle
void PlaneGyroMenu::review_prompt()
{
    const uint8_t s = cmd.step * 2;
    const uint8_t a = cmd.stick_ch[s] - 1;
    const uint8_t b = cmd.stick_ch[s + 1] - 1;
    command_prompt(CMD_CONFIRMATION_NEEDED, "Ends %c %u-%u %c %u-%u",
                   stick_letters[s], unsigned(cmd.lo[a]), unsigned(cmd.hi[a]),
                   stick_letters[s + 1], unsigned(cmd.lo[b]), unsigned(cmd.hi[b]));
}

void PlaneGyroMenu::track_sticks()
{
    for (uint8_t i = 0; i < 4; i++) {
        const uint16_t v = radio_in(i);
        if (v > RC_Channel::RC_MIN_LIMIT_PWM && v < RC_Channel::RC_MAX_LIMIT_PWM) {
            cmd.lo[i] = MIN(cmd.lo[i], v);
            cmd.hi[i] = MAX(cmd.hi[i], v);
        }
    }
}

void PlaneGyroMenu::command_confirm()
{
    const Item &item = items[cmd.item];
    if (cmd.stage == Stage::DONE) {
        // OK on a result: close the popup
        command_close();
        return;
    }
    if (armed()) {
        abandon();
        command_finish("Disarm first");
        return;
    }

    switch (cmd.stage) {
    case Stage::CONFIRM:
        switch (item.handler) {
        case Handler::TRIM_COPY: {
            if (plane.control_mode != &plane.mode_manual || !rc().has_valid_input()) {
                command_finish("Needs MANUAL and RC signal");
                break;
            }
            // trims are small: a bigger offset is a stick being held
            const RC_Channel *held = stick_off_centre(TRIM_COPY_MAX_PCT);
            if (held != nullptr) {
                command_finish("Ch%u off centre: release the stick", unsigned(held->ch()));
                break;
            }
            // in MANUAL each surface output sits where the sticks and
            // trims put it, mixing included: make that its neutral
            uint8_t copied = 0;
            for (uint8_t n = 1; n <= NUM_OUTPUTS; n++) {
                const SRV_Channel *out = SRV_Channels::srv_channel(n - 1);
                if (out == nullptr || !is_surface(out->get_function())) {
                    continue;
                }
                // in use from now, stored at the last OK: a copy that is
                // abandoned half way is gone after a restart
                cmd.saved_trim[n - 1] = uint16_t(output_param(n, "TRIM"));
                set_output_param(n, "TRIM", out->get_output_pwm(), false);
                copied++;
            }
            if (copied == 0) {
                command_finish("No surface outputs set");
                break;
            }
            cmd.stage = Stage::TRIM_CENTER;
            command_prompt(CMD_CONFIRMATION_NEEDED, "Now center the TX trims, then OK");
            break;
        }
        case Handler::CAL_LEVEL:
            plane.g2.gyro.request_level_calibration();
            cmd.stage = Stage::CAL_RUNNING;
            command_prompt(CMD_PROGRESS, "Calibrating, hold still");
            break;
        case Handler::RESET:
            AP_Param::erase_all();
            reboot_at_ms = AP_HAL::millis() + 1500;
            cmd.stage = Stage::DONE;
            command_prompt(CMD_PROGRESS, "Settings erased, restarting");
            break;
        default:
            break;
        }
        break;

#if HAL_INS_ACCELCAL_ENABLED
    case Stage::ACAL_POSE: {
        if (cmd.step == 0) {
            // the stock calibration, as a ground station starts it. Its
            // own messages and pose requests go to the first MAVLink port
            GCS_MAVLINK *link = gcs().chan(0);
            AP::ins().acal_init();
            AP_AccelCal *acal = AP::ins().get_acal();
            if (link == nullptr || acal == nullptr) {
                command_finish("Not available");
                break;
            }
            AP_AccelCal::register_client(&accel_cal_result);
            acal->start(link, 0, 0);
        }
        cmd.acal_accepted = false;
        cmd.acal_sampling = false;
        cmd.acal_ms = AP_HAL::millis();
        cmd.stage = Stage::ACAL_SAMPLE;
        command_prompt(CMD_PROGRESS, "Hold still");
        break;
    }
#endif

    case Stage::ORIENT_LEVEL: {
        Vector3f a = AP::ins().get_accel();
        // undo the current mounting to get the sensor's own axes
        a.rotate_inverse(Rotation(int16_t(param_get("AHRS_ORIENTATION"))));
        if (fabsf(a.length() - GRAVITY_MSS) > 1.5f) {
            command_finish("Moving, hold still and retry");
            break;
        }
        cmd.level_accel = a;
        cmd.stage = Stage::ORIENT_NOSE_UP;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Raise the nose about 45 deg");
        break;
    }

    case Stage::ORIENT_NOSE_UP: {
        Vector3f a = AP::ins().get_accel();
        a.rotate_inverse(Rotation(int16_t(param_get("AHRS_ORIENTATION"))));
        uint8_t rotation;
        if (!orientation_from(cmd.level_accel, a, rotation)) {
            command_finish("Not detected, try again");
            break;
        }
        param_set("AHRS_ORIENTATION", rotation);
        const uint8_t idx = choice_index(orientation_choices, ARRAY_SIZE(orientation_choices), rotation);
        if (idx < ARRAY_SIZE(orientation_choices)) {
            command_finish("%s, now calibrate level", orientation_choices[idx].label);
        } else {
            command_finish("Rotation %u, now calibrate level", unsigned(rotation));
        }
        break;
    }

    case Stage::TRIM_CENTER: {
        const RC_Channel *off = stick_off_centre(TRIM_CENTRED_PCT);
        if (off != nullptr) {
            // still the copied neutral plus the TX trim: ask again, Exit undoes
            command_prompt(CMD_CONFIRMATION_NEEDED, "Ch%u trim not centred, then OK", unsigned(off->ch()));
            break;
        }
        uint8_t copied = 0;
        for (uint8_t n = 1; n <= NUM_OUTPUTS; n++) {
            if (cmd.saved_trim[n - 1] != 0) {
                set_output_param(n, "TRIM", output_param(n, "TRIM"));
                cmd.saved_trim[n - 1] = 0;
                copied++;
            }
        }
        command_finish("Trims copied to %u outputs", unsigned(copied));
        break;
    }

    case Stage::STICK_CENTER:
        for (uint8_t ch = 0; ch < ARRAY_SIZE(cmd.base); ch++) {
            cmd.base[ch] = radio_in(ch);
        }
        for (uint8_t i = 0; i < 4; i++) {
            cmd.lo[i] = cmd.hi[i] = cmd.held_val[i] = cmd.base[i];
            cmd.held_ms[i] = AP_HAL::millis();
        }
        cmd.start_ms = AP_HAL::millis();
        cmd.step = 0;
        cmd.stage = Stage::STICK_STEP;
        step_prompt();
        break;

    case Stage::STICK_REVIEW:
        if (++cmd.step < 2) {
            review_prompt();
            break;
        }
        cmd.stage = Stage::STICK_CONFIRM;
        command_prompt(CMD_CONFIRMATION_NEEDED, "%c%u %c%u %c%u %c%u Save?",
                       stick_letters[0], unsigned(cmd.stick_ch[0]),
                       stick_letters[1], unsigned(cmd.stick_ch[1]),
                       stick_letters[2], unsigned(cmd.stick_ch[2]),
                       stick_letters[3], unsigned(cmd.stick_ch[3]));
        break;

    case Stage::STICK_CONFIRM: {
        bool remapped = false;
        for (uint8_t s = 0; s < 4; s++) {
            const uint8_t ch = cmd.stick_ch[s];
            if (uint8_t(param_get(rcmap_params[s])) != ch) {
                remapped = true;
            }
            param_set(rcmap_params[s], ch);
            char name[17];
            hal.util->snprintf(name, sizeof(name), "RC%u_REVERSED", unsigned(ch));
            param_set(name, cmd.stick_reversed[s] ? 1 : 0);
            // endpoints as moved; centre (throttle: low) as held at the start
            hal.util->snprintf(name, sizeof(name), "RC%u_MIN", unsigned(ch));
            param_set(name, cmd.lo[ch - 1]);
            hal.util->snprintf(name, sizeof(name), "RC%u_MAX", unsigned(ch));
            param_set(name, cmd.hi[ch - 1]);
            hal.util->snprintf(name, sizeof(name), "RC%u_TRIM", unsigned(ch));
            param_set(name, cmd.base[ch - 1]);
        }
        if (remapped) {
            // a new channel order only takes effect after a restart
            reboot_at_ms = AP_HAL::millis() + 1500;
            cmd.stage = Stage::DONE;
            command_prompt(CMD_PROGRESS, "Saved, restarting");
        } else {
            command_finish("Sticks saved");
        }
        break;
    }

    case Stage::AUX_REPLACE:
        aux_found(item.arg, true);
        break;

    case Stage::AUX_CONFIRM:
        if (item.arg == AUX_ARM && !switch_low(cmd.aux_ch)) {
            // ask again: assigning it now would arm the vehicle
            command_prompt(CMD_CONFIRMATION_NEEDED, "Arm switch off first, then OK");
            break;
        }
        aux_save(item.arg);
        break;

    case Stage::KNOB_LOW:
        cmd.knob_low = radio_in(cmd.aux_ch - 1);
        cmd.stage = Stage::KNOB_HIGH;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Knob to HIGH gain end, then OK");
        break;

    case Stage::KNOB_HIGH: {
        const uint16_t high = radio_in(cmd.aux_ch - 1);
        if (abs(int32_t(high) - cmd.knob_low) < LEARN_MIN_SPAN) {
            cmd.stage = Stage::KNOB_LOW;
            command_prompt(CMD_CONFIRMATION_NEEDED, "Turn further. LOW end, then OK");
            break;
        }
        // the knob's ends become the channel's range; low gain at the low
        // end whichever way the knob turns
        const uint8_t ch = cmd.aux_ch;
        char name[17];
        hal.util->snprintf(name, sizeof(name), "RC%u_MIN", unsigned(ch));
        param_set(name, MIN(high, cmd.knob_low));
        hal.util->snprintf(name, sizeof(name), "RC%u_MAX", unsigned(ch));
        param_set(name, MAX(high, cmd.knob_low));
        hal.util->snprintf(name, sizeof(name), "RC%u_TRIM", unsigned(ch));
        param_set(name, (uint32_t(high) + cmd.knob_low) / 2);
        hal.util->snprintf(name, sizeof(name), "RC%u_REVERSED", unsigned(ch));
        param_set(name, high < cmd.knob_low ? 1 : 0);
        if (cmd.replaces >= 0) {
            set_aux_channel(cmd.replaces, 0);
        }
        set_aux_channel(AUX_GAIN, ch);
        if (cmd.replaces >= 0) {
            // no room for the range next to the replaced function
            command_finish("Gain knob: Ch%u (was %s)%s", unsigned(ch), aux_specs[cmd.replaces].label,
                           high < cmd.knob_low ? ", reversed" : "");
        }
        if (cmd.replaces < 0 || !text_fits()) {
            command_finish("Gain knob: Ch%u %u-%u%s", unsigned(ch), unsigned(MIN(high, cmd.knob_low)),
                           unsigned(MAX(high, cmd.knob_low)), high < cmd.knob_low ? ", reversed" : "");
        }
        break;
    }

    default:
        break;
    }
}

// the channel for a switch function is known (cmd.aux_ch)
void PlaneGyroMenu::aux_found(uint8_t func, bool replace_confirmed)
{
    if (func == AUX_GAIN) {
        // learn the knob's ends too: they set its range and direction
        cmd.stage = Stage::KNOB_LOW;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Ch%u. Knob to LOW gain end, OK", unsigned(cmd.aux_ch));
        return;
    }
    cmd.stage = Stage::AUX_CONFIRM;
    if (func == AUX_ARM) {
        command_prompt(CMD_CONFIRMATION_NEEDED, "Ch%u: switch it off, then OK", unsigned(cmd.aux_ch));
    } else if (replace_confirmed) {
        // the pilot has just said yes to this channel
        aux_save(func);
    } else {
        command_prompt(CMD_CONFIRMATION_NEEDED, "Use Ch%u for %s?", unsigned(cmd.aux_ch), aux_specs[func].label);
    }
}

void PlaneGyroMenu::aux_save(uint8_t func)
{
    if (cmd.replaces >= 0) {
        set_aux_channel(cmd.replaces, 0);
    }
    set_aux_channel(func, cmd.aux_ch);
    if (cmd.replaces >= 0) {
        command_finish("%s: Ch%u (was %s)", aux_specs[func].label, unsigned(cmd.aux_ch),
                       aux_specs[cmd.replaces].label);
    }
    if (cmd.replaces < 0 || !text_fits()) {
        command_finish("%s: Ch%u", aux_specs[func].label, unsigned(cmd.aux_ch));
    }
}

void PlaneGyroMenu::command_poll()
{
    const Item &item = items[cmd.item];

    switch (cmd.stage) {
    case Stage::CAL_RUNNING: {
        const PlaneGyro::LevelCalState state = plane.g2.gyro.level_calibration_state();
        if (state == PlaneGyro::LEVEL_CAL_DONE) {
            command_finish("Calibration done");
        } else if (state == PlaneGyro::LEVEL_CAL_FAILED) {
            command_finish("Calibration failed");
        }
        break;
    }

    case Stage::MODE_WATCH:
        mode_watch_prompt();
        break;

    case Stage::AUX_MOVE: {
        int16_t best_delta = 0;
        uint8_t best_ch = 0;
        for (uint8_t ch = 5; ch <= 16; ch++) {
            const int16_t delta = int16_t(radio_in(ch - 1)) - int16_t(cmd.base[ch - 1]);
            if (abs(delta) > abs(best_delta)) {
                best_delta = delta;
                best_ch = ch;
            }
        }
        if (abs(best_delta) < LEARN_SWITCH_DELTA) {
            break;
        }
        cmd.aux_ch = best_ch;
        const int8_t owner = channel_owner(best_ch, item.arg);
        if (owner >= 0) {
            // the old function stays until the final save, so a cancel
            // at any step leaves it in place
            cmd.replaces = owner;
            cmd.stage = Stage::AUX_REPLACE;
            command_prompt(CMD_CONFIRMATION_NEEDED, "Ch%u is the %s. Replace?", unsigned(best_ch),
                           aux_specs[owner].label);
            if (!text_fits()) {
                command_prompt(CMD_CONFIRMATION_NEEDED, "Ch%u is %s. Replace?", unsigned(best_ch),
                               aux_specs[owner].label);
            }
            break;
        }
        aux_found(item.arg, false);
        break;
    }

    default:
        break;
    }
}

/*
  answer the radio's requests for our entries
 */
void PlaneGyroMenu::process_requests()
{
    AP_CRSF_Telem *crsf = AP::crsf_telem();
    if (crsf == nullptr) {
        return;
    }
    WITH_SEMAPHORE(crsf->get_semaphore());

    for (uint8_t n = 0; n < 8; n++) {
        uint8_t param_id;
        uint8_t events;
        AP_CRSF_Telem::ScriptedPayload payload;
        if (crsf->peek_menu_event(param_id, payload, events) == 0) {
            break;
        }
        const Item *item = find_item(param_id);
        if (item == nullptr) {
            // not ours: leave it for whoever else serves menus
            break;
        }
        crsf->pop_menu_event();

        uint8_t buf[200];
        uint8_t len;
        if (events & AP_CRSF_Telem::PARAMETER_WRITE) {
            len = write(*item, payload.payload, payload.payload_length, buf, sizeof(buf));
        } else {
            len = pack(*item, buf, sizeof(buf));
        }
        crsf->send_write_response(len, (const char *)buf);
    }
}

bool PlaneGyroMenu::busy() const
{
    if (reboot_at_ms != 0) {
        return true;
    }
    return cmd.item >= 0 && cmd.stage != Stage::NONE && cmd.stage != Stage::DONE &&
           cmd.stage != Stage::MODE_WATCH;
}

/*
  a command stays open only while the radio asks about it. When the
  radio script is closed or the link drops in the middle of one, undo
  what was not confirmed and let the model arm again.
 */
void PlaneGyroMenu::command_watchdog()
{
    if (cmd.item < 0 || cmd.stage == Stage::NONE || cmd.stage == Stage::DONE) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (cmd.stage == Stage::CAL_RUNNING && plane.g2.gyro.level_calibration_state() == PlaneGyro::LEVEL_CAL_PENDING) {
        // the calibration blocks the main loop, and with it our answers
        cmd.request_ms = now;
        return;
    }
    if (cmd.stage == Stage::AUX_MOVE && now - cmd.start_ms > LEARN_TIMEOUT_MS) {
        command_finish("Timed out");
        return;
    }
    const uint32_t limit = cmd.status == CMD_PROGRESS ? ABANDON_PROGRESS_MS : ABANDON_CONFIRM_MS;
    if (now - cmd.request_ms > limit) {
        abandon();
        command_finish("Timed out");
    }
}

void PlaneGyroMenu::update()
{
    if (!built) {
        // build once a CRSF receiver is talking to us
        const AP_RCProtocol_CRSF *crsf = AP_RCProtocol_CRSF::get_singleton();
        if (crsf == nullptr || !crsf->is_rx_active()) {
            return;
        }
        built = build();
        return;
    }

    if (reboot_at_ms != 0 && AP_HAL::millis() > reboot_at_ms && !armed()) {
        hal.scheduler->reboot(false);
    }

    // track stick travel while learning: the radio stops polling while a
    // confirmation is on screen, so this cannot wait for a poll
    if (cmd.stage == Stage::STICK_STEP) {
        track_sticks();
    }
    if (cmd.stage == Stage::STICK_STEP) {
        learn_sticks_step();
    }

#if HAL_INS_ACCELCAL_ENABLED
    if (cmd.stage == Stage::ACAL_POSE || cmd.stage == Stage::ACAL_SAMPLE) {
        accel_cal_update();
    }
#endif

    process_requests();
    command_watchdog();
}

#endif  // AP_PLANE_GYRO_MENU_ENABLED
