#pragma once

/*
  The gyro setup menu: a transmitter menu served over CRSF to ELRS and
  TBS radios (ExpressLRS Lua "Other Devices", TBS Agent Lite).

  It is built natively on AP_CRSF_Telem's scripted-menu structures, so
  no Lua scripting is needed. The menu follows the Loft GCS setup order
  (calibration, radio, flight modes, outputs with the wing template,
  failsafe, tuning) and reads and writes ordinary ArduPilot parameters.
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_RCTelemetry/AP_RCTelemetry_config.h>

#ifndef AP_PLANE_GYRO_MENU_ENABLED
#define AP_PLANE_GYRO_MENU_ENABLED (AP_PLANE_GYRO_ENABLED && HAL_CRSF_TELEM_ENABLED && AP_CRSF_SCRIPTING_ENABLED)
#endif

#if AP_PLANE_GYRO_MENU_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_RCTelemetry/AP_CRSF_Telem.h>

class PlaneGyroMenu {
public:
    // called at 50Hz: builds the menu once a CRSF receiver is talking,
    // then answers the radio's read and write requests
    void update();

    // true while a command that arming must not interrupt is open on the
    // radio, or a restart is pending. The mode switch check is not one:
    // it changes nothing.
    bool busy() const;

private:
    // what an entry does when the radio reads or writes it
    enum class Handler : uint8_t {
        STATUS,         // info: armed state, mode, GPS
        FIRMWARE,       // info: firmware version
        MENU_VERSION,   // info, hidden: this menu's layout version, for radio apps
        NO_GPS_INFO,    // info shown in place of RTL settings without a GPS
        WING_TYPE,      // select: sets output channels 1-4 when written
        TRIM_COPY,      // command: copy transmitter trims into the servo trims
        ORIENT,         // select: AHRS_ORIENTATION
        ORIENT_DETECT,  // command: detect orientation (level, then nose up)
        CAL_LEVEL,      // command: level (simple accel) calibration
        CAL_GYRO,       // command: gyro calibration
        LEARN_STICKS,   // command: learn stick channels and directions
        AUX_CH,         // select: channel of a switch function (arg = AuxFunc)
        AUX_LEARN,      // command: learn a switch channel (arg = AuxFunc)
        MODE_POS,       // select: mode for switch position (arg 0..2)
        MODE_CHECK,     // command: show the mode switch position and mode live
        OUT_FUNC,       // select: SERVOn_FUNCTION (arg = output)
        OUT_MIN,        // number: SERVOn_MIN
        OUT_TRIM,       // number: SERVOn_TRIM
        OUT_MAX,        // number: SERVOn_MAX
        OUT_REV,        // select: SERVOn_REVERSED
        LINK_LOSS,      // select: FS_LONG_ACTN
        NUMBER,         // number bound to a parameter (arg = number spec)
        RESET,          // command: reset all parameters and reboot
    };

    // switch functions the radio menu can assign
    enum AuxFunc : uint8_t {
        AUX_MODE = 0,
        AUX_ARM,
        AUX_RTL,
        AUX_AUTOTUNE,
        AUX_GAIN,
        AUX_COUNT
    };

    struct Item {
        uint8_t id;         // CRSF parameter id
        Handler handler;
        uint8_t arg;
        const char *name;
    };

    // stages of the command currently running on the radio
    enum class Stage : uint8_t {
        NONE,
        CONFIRM,            // waiting for OK on a confirmation prompt
        CAL_RUNNING,        // calibration requested, waiting for result
        ORIENT_LEVEL,       // waiting for OK with the model level
        ORIENT_NOSE_UP,     // waiting for OK with the nose raised
        STICK_CENTER,       // waiting for OK with sticks and trims centred
        STICK_STEP,         // waiting for the stick move of the current step
        STICK_REVIEW,       // showing the learned endpoints, waiting for OK
        STICK_CONFIRM,      // learned all sticks, waiting for OK to save
        AUX_MOVE,           // waiting for a switch to move
        AUX_REPLACE,        // learned a channel in use, waiting for OK to replace its function
        AUX_CONFIRM,        // learned a switch, waiting for OK to save
        KNOB_LOW,           // gain knob found, waiting for OK at its low gain end
        KNOB_HIGH,          // waiting for OK at the knob's high gain end
        MODE_WATCH,         // showing the mode switch until the pilot exits
        TRIM_CENTER,        // trims copied, waiting for the TX trims to be centred
        DONE,               // finished, reporting the result
    };

    static constexpr uint8_t MAX_ITEMS = 110;
    static constexpr uint8_t NUM_OUTPUTS = 8;

    Item items[MAX_ITEMS];
    uint8_t num_items;
    bool built;

    // the command popup currently open on the radio
    struct {
        int16_t item = -1;          // index into items, -1 when idle
        Stage stage;
        uint32_t start_ms;
        uint32_t request_ms;        // when the radio last asked about the command
        char info[48];              // text shown in the popup
        uint8_t status;             // CRSF command status to report
        // stick learning
        uint16_t base[16];          // channel values when learning started
        uint8_t step;               // index into the stick learning steps, then the review page
        uint8_t stick_ch[4];
        bool stick_reversed[4];
        uint16_t lo[4], hi[4];      // travel seen on channels 1-4 while learning
        uint16_t held_val[4];       // channels 1-4: value being held, and since when
        uint32_t held_ms[4];
        // switch learning
        uint8_t aux_ch;
        int8_t replaces = -1;       // function aux_ch is taken from at the save, or -1
        uint16_t knob_low;          // gain knob value at its low gain end
        // orientation detection
        Vector3f level_accel;
        // trim copy: servo trims before the copy, 0 where not changed. The
        // copied trims are in use but not stored until the last OK
        uint16_t saved_trim[NUM_OUTPUTS];
    } cmd;

    uint32_t reboot_at_ms;

    bool build();
    void add(AP_CRSF_Telem::ScriptedMenu *menu, Handler handler, uint8_t arg, const char *name);
    AP_CRSF_Telem::ScriptedMenu *submenu(AP_CRSF_Telem::ScriptedMenu *parent, const char *name);
    Item *find_item(uint8_t id);
    void process_requests();

    // build the CRSF parameter entry for an item; returns its length
    uint8_t pack(const Item &item, uint8_t *buf, uint8_t size);
    // apply a write from the radio; returns the response length
    uint8_t write(const Item &item, const uint8_t *payload, uint8_t length, uint8_t *buf, uint8_t size);

    // commands
    void command(uint8_t index, uint8_t action);
    void command_start(uint8_t index);
    void command_confirm();
    void command_poll();
    void command_cancel();
    // switch learning: the next step once the channel is known, and the save
    void aux_found(uint8_t func, bool replace_confirmed);
    void aux_save(uint8_t func);
    void command_finish(const char *fmt, ...) FMT_PRINTF(2, 3);
    void command_close();
    // true when the open command's name and popup text fit in one reply
    bool text_fits() const;
    void command_prompt(uint8_t status, const char *fmt, ...) FMT_PRINTF(3, 4);
    void step_prompt();
    // widen the travel seen on channels 1-4 while learning sticks
    void track_sticks();
    // complete the current stick learning step once its move is held
    void learn_sticks_step();
    void review_prompt();
    void mode_watch_prompt();
    // undo a trim copy that was not confirmed
    void restore_trims();
    // end a command the radio no longer asks about
    void command_watchdog();
    // first stick (roll, pitch, yaw) further from centre than percent, or nullptr
    static const RC_Channel *stick_off_centre(int16_t percent);

    // parameter helpers
    static float param_get(const char *name);
    static void param_set(const char *name, float value);
    static float output_param(uint8_t output, const char *field);
    // save false changes the value in use but not the stored one
    static void set_output_param(uint8_t output, const char *field, float value, bool save = true);

    // switch functions
    static uint8_t aux_channel(uint8_t func);
    static void set_aux_channel(uint8_t func, uint8_t ch);
    static int8_t channel_owner(uint8_t ch, uint8_t except_func);

    // wing templates
    static uint8_t detect_wing();
    static void apply_wing(uint8_t wing);

    static bool gps_present();
    static bool armed();
    // a flight mode's name in capitals, short enough for the radio screen
    static void mode_name(const Mode &mode, char *buf, uint8_t size);
    bool orientation_from(const Vector3f &level, const Vector3f &nose_up, uint8_t &rotation) const;
};

#endif  // AP_PLANE_GYRO_MENU_ENABLED
