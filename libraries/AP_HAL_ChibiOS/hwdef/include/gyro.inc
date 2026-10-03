# Shared feature profile for "gyro" builds of ArduPlane.
#
# Goal: a stabilization-only firmware that replaces a consumer RC gyro
# (AS3X, FrSky S6R/S8R, Aura, HobbyEagle A3) on powered fixed-wing
# models. Keeps MANUAL / ACRO / FBWA / RTL, RC input, servo
# mixing, autotune and logging. Removes navigation and payload
# subsystems that a gyro never needs.
#
# Include this AFTER the base board hwdef so it can override the
# board's own feature defines, e.g.
#   include ../KakuteF4-Wing/hwdef.dat
#   include ../include/gyro.inc

# --- Navigation ------------------------------------------------------
# EKF3 is the single largest flash consumer. DCM is enough for
# rate/attitude stabilization and for GPS-based RTL.
define HAL_NAVEKF3_AVAILABLE 0
define AP_TERRAIN_AVAILABLE 0
define HAL_RALLY_ENABLED 0
define AP_FENCE_ENABLED 0
define HAL_ADSB_ENABLED 0
define HAL_SOARING_ENABLED 0
undef AP_LANDINGGEAR_ENABLED
define AP_LANDINGGEAR_ENABLED 0
# AP_MISSION_ENABLED cannot be turned off without ArduPlane source
# changes (GCS_MAVLink_Plane overrides a virtual guarded by it).

# --- VTOL ------------------------------------------------------------
# Powered fixed-wing only. Quadplane support is large.
define HAL_QUADPLANE_ENABLED 0

# --- Video / FPV -----------------------------------------------------
# base boards and minimize_*.inc already define these; undef first
undef OSD_ENABLED AP_TRAMP_ENABLED
define OSD_ENABLED 0
define OSD_PARAM_ENABLED 0
define HAL_OSD_SIDEBAR_ENABLE 0
define HAL_PLUSCODE_ENABLE 0
define AP_VIDEOTX_ENABLED 0
define AP_SMARTAUDIO_ENABLED 0
define AP_TRAMP_ENABLED 0
define HAL_RUNCAM_ENABLED 0
define AP_CAMERA_ENABLED 0
define HAL_MOUNT_ENABLED 0
# MSP DisplayPort OSD and MSP telemetry go with the OSD
undef HAL_MSP_ENABLED HAL_WITH_MSP_DISPLAYPORT HAL_MSP_SENSORS_ENABLED
define HAL_MSP_ENABLED 0
define HAL_WITH_MSP_DISPLAYPORT 0
define HAL_MSP_SENSORS_ENABLED 0
# Drop the OSD font files that the base board put into ROMFS.
undef ROMFS

# --- Transmitter menu ------------------------------------------------
# The "Gyro Setup" menu is built natively in ArduPlane (gyro_menu.cpp)
# on the CRSF scripted-menu engine, so Lua scripting is not needed and
# stays off (frees ~60 KB of RAM and ~120 KB of flash on an F405).
# The menu engine is normally tied to scripting and the OSD, so it is
# switched on explicitly.
define AP_SCRIPTING_ENABLED 0
undef HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED
define HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED 1
define AP_CRSF_SCRIPTING_ENABLED 1

# --- Keep ------------------------------------------------------------
# Logging stays on for autotune review (board must provide a
# dataflash or SD backend). Autotune, transmitter tuning knob and
# CRSF telemetry stay on.
define HAL_LOGGING_ENABLED 1
define AP_TUNING_ENABLED 1
define HAL_CRSF_TELEM_ENABLED 1
