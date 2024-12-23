from dataclasses import dataclass

MSP_OSD_CONFIG = 84        # Out message: Get OSD settings - Betaflight
MSP_NAME = 10
MSP_BATTERY_STATE = 130    # Out message: Connected/Disconnected, Voltage, Current Used

@dataclass
class MspOsdConfig:
    osdflags: int
    video_system: int
    units: int
    rssi_alarm: int
    cap_alarm: int
    old_timer_alarm: int
    osd_item_count: int               # 56
    alt_alarm: int
    osd_rssi_value_pos: int
    osd_main_batt_voltage_pos: int
    osd_crosshairs_pos: int
    osd_artificial_horizon_pos: int
    osd_horizon_sidebars_pos: int
    osd_item_timer_1_pos: int
    osd_item_timer_2_pos: int
    osd_flymode_pos: int
    osd_craft_name_pos: int
    osd_throttle_pos_pos: int
    osd_vtx_channel_pos: int
    osd_current_draw_pos: int
    osd_mah_drawn_pos: int
    osd_gps_speed_pos: int
    osd_gps_sats_pos: int
    osd_altitude_pos: int
    osd_roll_pids_pos: int
    osd_pitch_pids_pos: int
    osd_yaw_pids_pos: int
    osd_power_pos: int
    osd_pidrate_profile_pos: int
    osd_warnings_pos: int
    osd_avg_cell_voltage_pos: int
    osd_gps_lon_pos: int
    osd_gps_lat_pos: int
    osd_debug_pos: int
    osd_pitch_angle_pos: int
    osd_roll_angle_pos: int
    osd_main_batt_usage_pos: int
    osd_disarmed_pos: int
    osd_home_dir_pos: int
    osd_home_dist_pos: int
    osd_numerical_heading_pos: int
    osd_numerical_vario_pos: int
    osd_compass_bar_pos: int
    osd_esc_tmp_pos: int
    osd_esc_rpm_pos: int
    osd_remaining_time_estimate_pos: int
    osd_rtc_datetime_pos: int
    osd_adjustment_range_pos: int
    osd_core_temperature_pos: int
    osd_anti_gravity_pos: int
    osd_g_force_pos: int
    osd_motor_diag_pos: int
    osd_log_status_pos: int
    osd_flip_arrow_pos: int
    osd_link_quality_pos: int
    osd_flight_dist_pos: int
    osd_stick_overlay_left_pos: int
    osd_stick_overlay_right_pos: int
    osd_display_name_pos: int
    osd_esc_rpm_freq_pos: int
    osd_rate_profile_name_pos: int
    osd_pid_profile_name_pos: int
    osd_profile_name_pos: int
    osd_rssi_dbm_value_pos: int
    osd_rc_channels_pos: int
    osd_stat_count: int               # 24
    osd_stat_rtc_date_time: int
    osd_stat_timer_1: int
    osd_stat_timer_2: int
    osd_stat_max_speed: int
    osd_stat_max_distance: int
    osd_stat_min_battery: int
    osd_stat_end_battery: int
    osd_stat_battery: int
    osd_stat_min_rssi: int
    osd_stat_max_current: int
    osd_stat_used_mah: int
    osd_stat_max_altitude: int
    osd_stat_blackbox: int
    osd_stat_blackbox_number: int
    osd_stat_max_g_force: int
    osd_stat_max_esc_temp: int
    osd_stat_max_esc_rpm: int
    osd_stat_min_link_quality: int
    osd_stat_flight_distance: int
    osd_stat_max_fft: int
    osd_stat_total_flights: int
    osd_stat_total_time: int
    osd_stat_total_dist: int
    osd_stat_min_rssi_dbm: int
    osd_timer_count: int
    osd_timer_1: int
    osd_timer_2: int
    enabledwarnings: int
    osd_warning_count: int            # 16
    enabledwarnings_1_41_plus: int
    osd_profile_count: int            # 1
    osdprofileindex: int              # 1
    overlay_radio_mode: int           # 0

@dataclass
class MspName:
    craft_name: str                   # 15 characters max possible displayed in the goggles

@dataclass
class MspBatteryState:
    batteryCellCount: int
    batteryCapacity: int
    legacyBatteryVoltage: int
    mAhDrawn: int
    amperage: int
    batteryState: int
    batteryVoltage: int

@dataclass
class MspStatusDJI:
    cycleTime: int
    i2cErrorCounter: int
    sensor: int                       # MSP_STATUS_SENSOR_...
    flightModeFlags: int              # see getActiveModes()
    configProfileIndex: int
    averageSystemLoadPercent: int     # 0...100
    armingFlags: int                  # 0x0103 or 0x0301
    accCalibrationAxisFlags: int      # 0
    DJI_ARMING_DISABLE_FLAGS_COUNT: int  # 25
    djiPackArmingDisabledFlags: int   # (1 << 24)
