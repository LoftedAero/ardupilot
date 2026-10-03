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

// how often the radio polls a running command, in 10 ms units
constexpr uint8_t CMD_POLL_INTERVAL = 25;

// how far (us) a channel must move to count while learning
constexpr int16_t LEARN_STICK_DELTA = 350;
constexpr int16_t LEARN_RELEASE_DELTA = 150;
constexpr int16_t LEARN_SWITCH_DELTA = 300;
constexpr uint32_t LEARN_TIMEOUT_MS = 30000;

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
const Choice reversed_choices[] = {{"Normal", 0}, {"Reversed", 1}};
const Choice on_off_choices[] = {{"Off", 0}, {"On", 1}};
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
    NUM_GAIN_MIN, NUM_GAIN_MAX, NUM_ACRO_ROLL, NUM_ACRO_PITCH, NUM_ROLL_LIMIT,
    NUM_PITCH_MAX, NUM_PITCH_MIN, NUM_AUTOTUNE_LEVEL, NUM_RTL_ALT,
};
const NumberSpec number_specs[] = {
    {"GYRO_GAIN_MIN", 0, 1, 0.05, 2, ""},
    {"GYRO_GAIN_MAX", 1, 3, 0.05, 2, ""},
    {"ACRO_ROLL_RATE", 30, 500, 10, 0, "d/s"},
    {"ACRO_PITCH_RATE", 30, 500, 10, 0, "d/s"},
    {"ROLL_LIMIT_DEG", 10, 85, 5, 0, "deg"},
    {"PTCH_LIM_MAX_DEG", 5, 60, 5, 0, "deg"},
    {"PTCH_LIM_MIN_DEG", -60, -5, 5, 0, "deg"},
    {"AUTOTUNE_LEVEL", 1, 10, 1, 0, ""},
    {"RTL_ALTITUDE", 20, 400, 10, 0, "m"},
};

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

// stick learning order: roll, pitch, yaw, throttle
const char *const stick_prompts[] = {
    "Move AILERON stick right",
    "Pull ELEVATOR stick back",
    "Move RUDDER stick right",
    "Move THROTTLE to full",
};
const char *const rcmap_params[] = {"RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_YAW", "RCMAP_THROTTLE"};
const char stick_letters[] = {'A', 'E', 'R', 'T'};

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

void PlaneGyroMenu::set_output_param(uint8_t output, const char *field, float value)
{
    char name[17];
    hal.util->snprintf(name, sizeof(name), "SERVO%u_%s", unsigned(output), field);
    param_set(name, value);
}

bool PlaneGyroMenu::gps_present()
{
    return AP::gps().status() != AP_GPS::NO_GPS;
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
    wing_choice = detect_wing();

    add(root, Handler::STATUS, 0, "Status");

    AP_CRSF_Telem::ScriptedMenu *m = submenu(root, "1 Calibration");
    add(m, Handler::ORIENT, 0, "Orientation");
    add(m, Handler::ORIENT_DETECT, 0, "Detect orientation");
    add(m, Handler::CAL_LEVEL, 0, "Calibrate level");
    add(m, Handler::CAL_GYRO, 0, "Calibrate gyros");

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

    // the wing template sits above the channels it sets
    m = submenu(root, "4 Outputs");
    add(m, Handler::WING_TYPE, 0, "Wing type");
    add(m, Handler::WING_APPLY, 0, "Apply template");
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
    add(m, Handler::NUMBER, NUM_ACRO_ROLL, "ACRO roll rate");
    add(m, Handler::NUMBER, NUM_ACRO_PITCH, "ACRO pitch rate");
    add(m, Handler::NUMBER, NUM_ROLL_LIMIT, "FBWA bank limit");
    add(m, Handler::NUMBER, NUM_PITCH_MAX, "FBWA pitch up");
    add(m, Handler::NUMBER, NUM_PITCH_MIN, "FBWA pitch down");
    add(m, Handler::NUMBER, NUM_AUTOTUNE_LEVEL, "Autotune level");
    add(m, Handler::TRIM_CAP, 0, "Trim capture");

    m = submenu(root, "Tools");
    add(m, Handler::FIRMWARE, 0, "Firmware");
    add(m, Handler::RESET, 0, "Reset to defaults");

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gyro: setup menu ready (%u entries)", unsigned(num_items));
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
        case AP_GPS::NO_GPS:
            strncpy(gps, "none", sizeof(gps));
            break;
        case AP_GPS::NO_FIX:
            strncpy(gps, "no fix", sizeof(gps));
            break;
        case AP_GPS::GPS_OK_FIX_2D:
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
    case Handler::NO_GPS_INFO:
        pack_info(p, gps_present() ? CRSF_HIDDEN : 0, item.name, "needs a GPS");
        break;
    case Handler::WING_TYPE: {
        // the template choice is menu state, not a parameter
        static const Choice wings[] = {{"Normal", 0}, {"Flying wing", 1}, {"V-tail", 2}};
        pack_select(p, 0, item.name, wings, NUM_WINGS, wing_choice);
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
    case Handler::NUMBER: {
        const NumberSpec &n = number_specs[item.arg];
        const uint8_t flags = item.arg == NUM_RTL_ALT ? no_gps_hidden : 0;
        pack_number(p, flags, item.name, param_get(n.param), n.min, n.max, n.step, n.decimals, n.unit);
        break;
    }
    case Handler::TRIM_CAP:
        pack_select(p, 0, item.name, on_off_choices, ARRAY_SIZE(on_off_choices),
                    int16_t(param_get("GYRO_TRIM_CAP")));
        break;
    case Handler::WING_APPLY:
    case Handler::ORIENT_DETECT:
    case Handler::CAL_LEVEL:
    case Handler::CAL_GYRO:
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
        p.str(active ? cmd.info : "");
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
    case Handler::WING_APPLY:
    case Handler::ORIENT_DETECT:
    case Handler::CAL_LEVEL:
    case Handler::CAL_GYRO:
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

    // number entries carry a 4 byte value
    switch (item.handler) {
    case Handler::OUT_MIN:
    case Handler::OUT_TRIM:
    case Handler::OUT_MAX:
    case Handler::NUMBER: {
        if (length >= 4) {
            const int32_t raw = payload_i32(payload);
            if (item.handler == Handler::NUMBER) {
                const NumberSpec &n = number_specs[item.arg];
                float scale = 1;
                for (uint8_t i = 0; i < n.decimals; i++) {
                    scale *= 10;
                }
                param_set(n.param, constrain_float(raw / scale, n.min, n.max));
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
    const uint8_t idx = length >= 1 ? payload[0] : 0xFF;
    switch (item.handler) {
    case Handler::WING_TYPE:
        if (idx < NUM_WINGS) {
            wing_choice = idx;
        }
        p.u8(wing_choice);
        break;
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
    case Handler::TRIM_CAP:
        if (idx < ARRAY_SIZE(on_off_choices)) {
            param_set("GYRO_TRIM_CAP", on_off_choices[idx].value);
        }
        p.u8(choice_index(on_off_choices, ARRAY_SIZE(on_off_choices), int16_t(param_get("GYRO_TRIM_CAP"))));
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

void PlaneGyroMenu::restore_arm_switch()
{
    // put back an arm switch that was cleared while learning; only once
    // that switch is off, otherwise assigning it would arm the vehicle
    if (cmd.saved_arm_ch > 0 && aux_channel(AUX_ARM) == 0) {
        if (switch_low(uint8_t(cmd.saved_arm_ch))) {
            set_aux_channel(AUX_ARM, uint8_t(cmd.saved_arm_ch));
            cmd.saved_arm_ch = 0;
        }
    } else {
        cmd.saved_arm_ch = 0;
    }
}

void PlaneGyroMenu::command_cancel()
{
    restore_arm_switch();
    command_close();
}

void PlaneGyroMenu::command_start(uint8_t index)
{
    // abandon whatever was open before
    if (cmd.item >= 0 && cmd.stage != Stage::DONE) {
        restore_arm_switch();
    }
    const int8_t pending_arm = cmd.saved_arm_ch;
    cmd = {};
    cmd.saved_arm_ch = pending_arm;
    cmd.item = index;
    cmd.start_ms = AP_HAL::millis();

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
    case Handler::WING_APPLY:
        if (wing_choice >= NUM_WINGS) {
            command_finish("Pick a wing type first");
            return;
        }
        cmd.stage = Stage::CONFIRM;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Set channels 1-4 for %s?", wing_labels[wing_choice]);
        break;
    case Handler::ORIENT_DETECT:
        cmd.stage = Stage::ORIENT_LEVEL;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Hold model level, nose forward");
        break;
    case Handler::CAL_LEVEL:
        cmd.stage = Stage::CONFIRM;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Hold model at level attitude");
        break;
    case Handler::CAL_GYRO:
        cmd.stage = Stage::CONFIRM;
        command_prompt(CMD_CONFIRMATION_NEEDED, "Keep the model still");
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
            cmd.base[ch] = RC_Channels::get_radio_in(ch);
        }
        if (item.handler == Handler::LEARN_STICKS) {
            cmd.stage = Stage::STICK_MOVE;
            stick_prompt();
        } else {
            if (item.arg == AUX_ARM) {
                // flipping the arm switch while it is assigned would arm
                const uint8_t ch = aux_channel(AUX_ARM);
                if (ch != 0) {
                    cmd.saved_arm_ch = int8_t(ch);
                    set_aux_channel(AUX_ARM, 0);
                }
            }
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
    const uint16_t pwm = RC_Channels::get_radio_in(ch - 1);
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

void PlaneGyroMenu::stick_prompt()
{
    command_prompt(CMD_PROGRESS, "%s", stick_prompts[cmd.stick]);
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
        restore_arm_switch();
        command_finish("Disarm first");
        return;
    }

    switch (cmd.stage) {
    case Stage::CONFIRM:
        switch (item.handler) {
        case Handler::WING_APPLY:
            apply_wing(wing_choice);
            command_finish("Channels 1-4 set for %s", wing_labels[wing_choice]);
            break;
        case Handler::CAL_LEVEL:
        case Handler::CAL_GYRO:
            plane.g2.gyro.request_calibration(item.handler == Handler::CAL_LEVEL ? 2 : 1);
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

    case Stage::STICK_CONFIRM: {
        bool remapped = false;
        for (uint8_t s = 0; s < 4; s++) {
            if (uint8_t(param_get(rcmap_params[s])) != cmd.stick_ch[s]) {
                remapped = true;
            }
            param_set(rcmap_params[s], cmd.stick_ch[s]);
            char name[17];
            hal.util->snprintf(name, sizeof(name), "RC%u_REVERSED", unsigned(cmd.stick_ch[s]));
            param_set(name, cmd.stick_reversed[s] ? 1 : 0);
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

    case Stage::AUX_CONFIRM:
        if (item.arg == AUX_ARM && !switch_low(cmd.aux_ch)) {
            // ask again: assigning it now would arm the vehicle
            command_prompt(CMD_CONFIRMATION_NEEDED, "Arm switch off first, then OK");
            break;
        }
        set_aux_channel(item.arg, cmd.aux_ch);
        cmd.saved_arm_ch = 0;
        command_finish("%s: Ch%u", aux_specs[item.arg].label, unsigned(cmd.aux_ch));
        break;

    default:
        break;
    }
}

void PlaneGyroMenu::command_poll()
{
    const Item &item = items[cmd.item];
    const uint32_t now = AP_HAL::millis();

    switch (cmd.stage) {
    case Stage::CAL_RUNNING: {
        const int8_t state = plane.g2.gyro.calibration_state();
        if (state == 0) {
            command_finish("Calibration done");
        } else if (state < 0) {
            command_finish("Calibration failed");
        }
        break;
    }

    case Stage::STICK_MOVE: {
        if (now - cmd.start_ms > LEARN_TIMEOUT_MS) {
            command_finish("Timed out");
            break;
        }
        // the stick channel that moved furthest, among channels 1-4 not yet learned
        int16_t best_delta = 0;
        uint8_t best_ch = 0;
        for (uint8_t ch = 1; ch <= 4; ch++) {
            bool used = false;
            for (uint8_t s = 0; s < cmd.stick; s++) {
                used |= cmd.stick_ch[s] == ch;
            }
            if (used) {
                continue;
            }
            const int16_t delta = int16_t(RC_Channels::get_radio_in(ch - 1)) - int16_t(cmd.base[ch - 1]);
            if (abs(delta) > abs(best_delta)) {
                best_delta = delta;
                best_ch = ch;
            }
        }
        if (abs(best_delta) >= LEARN_STICK_DELTA) {
            cmd.stick_ch[cmd.stick] = best_ch;
            // right, back (nose up) and full throttle should all raise the value
            cmd.stick_reversed[cmd.stick] = best_delta < 0;
            cmd.stage = Stage::STICK_RELEASE;
            command_prompt(CMD_PROGRESS, "Ch%u. Now release the stick", unsigned(best_ch));
        }
        break;
    }

    case Stage::STICK_RELEASE: {
        const uint8_t ch = cmd.stick_ch[cmd.stick];
        const int16_t delta = int16_t(RC_Channels::get_radio_in(ch - 1)) - int16_t(cmd.base[ch - 1]);
        // throttle is learned last: no need to wait for it to come back
        if (abs(delta) <= LEARN_RELEASE_DELTA || cmd.stick == 3) {
            cmd.stick++;
            cmd.start_ms = now;
            if (cmd.stick < 4) {
                cmd.stage = Stage::STICK_MOVE;
                stick_prompt();
            } else {
                cmd.stage = Stage::STICK_CONFIRM;
                command_prompt(CMD_CONFIRMATION_NEEDED, "%c%u %c%u %c%u %c%u Save?",
                               stick_letters[0], unsigned(cmd.stick_ch[0]),
                               stick_letters[1], unsigned(cmd.stick_ch[1]),
                               stick_letters[2], unsigned(cmd.stick_ch[2]),
                               stick_letters[3], unsigned(cmd.stick_ch[3]));
            }
        }
        break;
    }

    case Stage::MODE_WATCH:
        mode_watch_prompt();
        break;

    case Stage::AUX_MOVE: {
        if (now - cmd.start_ms > LEARN_TIMEOUT_MS) {
            restore_arm_switch();
            command_finish("Timed out");
            break;
        }
        int16_t best_delta = 0;
        uint8_t best_ch = 0;
        for (uint8_t ch = 5; ch <= 16; ch++) {
            const int16_t delta = int16_t(RC_Channels::get_radio_in(ch - 1)) - int16_t(cmd.base[ch - 1]);
            if (abs(delta) > abs(best_delta)) {
                best_delta = delta;
                best_ch = ch;
            }
        }
        if (abs(best_delta) < LEARN_SWITCH_DELTA) {
            break;
        }
        const int8_t owner = channel_owner(best_ch, item.arg);
        if (owner >= 0) {
            restore_arm_switch();
            command_finish("Ch%u is the %s", unsigned(best_ch), aux_specs[owner].label);
            break;
        }
        cmd.aux_ch = best_ch;
        cmd.stage = Stage::AUX_CONFIRM;
        if (item.arg == AUX_ARM) {
            command_prompt(CMD_CONFIRMATION_NEEDED, "Ch%u: switch it off, then OK", unsigned(best_ch));
        } else {
            command_prompt(CMD_CONFIRMATION_NEEDED, "Use Ch%u for %s?", unsigned(best_ch), aux_specs[item.arg].label);
        }
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

    // finish restoring an arm switch once it has been turned off
    if (cmd.saved_arm_ch > 0 && (cmd.stage == Stage::DONE || cmd.item < 0)) {
        restore_arm_switch();
    }

    process_requests();
}

#endif  // AP_PLANE_GYRO_MENU_ENABLED
