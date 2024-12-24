from dataclasses import dataclass
import struct

MSP_OSD_CONFIG = 84        # Out message: Get OSD settings - Betaflight
MSP_NAME = 10
MSP_BATTERY_STATE = 130    # Out message: Connected/Disconnected, Voltage, Current Used

@dataclass
class MspOsdConfig:
    osdflags: int = 0
    video_system: int = 0
    units: int = 0
    rssi_alarm: int = 0
    cap_alarm: int = 0
    old_timer_alarm: int = 0
    osd_item_count: int = 0               # 56
    alt_alarm: int = 0
    osd_rssi_value_pos: int = 0
    osd_main_batt_voltage_pos: int = 0
    osd_crosshairs_pos: int = 0
    osd_artificial_horizon_pos: int = 0
    osd_horizon_sidebars_pos: int = 0
    osd_item_timer_1_pos: int = 0
    osd_item_timer_2_pos: int = 0
    osd_flymode_pos: int = 0
    osd_craft_name_pos: int = 0
    osd_throttle_pos_pos: int = 0
    osd_vtx_channel_pos: int = 0
    osd_current_draw_pos: int = 0
    osd_mah_drawn_pos: int = 0
    osd_gps_speed_pos: int = 0
    osd_gps_sats_pos: int = 0
    osd_altitude_pos: int = 0
    osd_roll_pids_pos: int = 0
    osd_pitch_pids_pos: int = 0
    osd_yaw_pids_pos: int = 0
    osd_power_pos: int = 0
    osd_pidrate_profile_pos: int = 0
    osd_warnings_pos: int = 0
    osd_avg_cell_voltage_pos: int = 0
    osd_gps_lon_pos: int = 0
    osd_gps_lat_pos: int = 0
    osd_debug_pos: int = 0
    osd_pitch_angle_pos: int = 0
    osd_roll_angle_pos: int = 0
    osd_main_batt_usage_pos: int = 0
    osd_disarmed_pos: int = 0
    osd_home_dir_pos: int = 0
    osd_home_dist_pos: int = 0
    osd_numerical_heading_pos: int = 0
    osd_numerical_vario_pos: int = 0
    osd_compass_bar_pos: int = 0
    osd_esc_tmp_pos: int = 0
    osd_esc_rpm_pos: int = 0
    osd_remaining_time_estimate_pos: int = 0
    osd_rtc_datetime_pos: int = 0
    osd_adjustment_range_pos: int = 0
    osd_core_temperature_pos: int = 0
    osd_anti_gravity_pos: int = 0
    osd_g_force_pos: int = 0
    osd_motor_diag_pos: int = 0
    osd_log_status_pos: int = 0
    osd_flip_arrow_pos: int = 0
    osd_link_quality_pos: int = 0
    osd_flight_dist_pos: int = 0
    osd_stick_overlay_left_pos: int = 0
    osd_stick_overlay_right_pos: int = 0
    osd_display_name_pos: int = 0
    osd_esc_rpm_freq_pos: int = 0
    osd_rate_profile_name_pos: int = 0
    osd_pid_profile_name_pos: int = 0
    osd_profile_name_pos: int = 0
    osd_rssi_dbm_value_pos: int = 0
    osd_rc_channels_pos: int = 0
    osd_stat_count: int = 0               # 24
    osd_stat_rtc_date_time: int = 0
    osd_stat_timer_1: int = 0
    osd_stat_timer_2: int = 0
    osd_stat_max_speed: int = 0
    osd_stat_max_distance: int = 0
    osd_stat_min_battery: int = 0
    osd_stat_end_battery: int = 0
    osd_stat_battery: int = 0
    osd_stat_min_rssi: int = 0
    osd_stat_max_current: int = 0
    osd_stat_used_mah: int = 0
    osd_stat_max_altitude: int = 0
    osd_stat_blackbox: int = 0
    osd_stat_blackbox_number: int = 0
    osd_stat_max_g_force: int = 0
    osd_stat_max_esc_temp: int = 0
    osd_stat_max_esc_rpm: int = 0
    osd_stat_min_link_quality: int = 0
    osd_stat_flight_distance: int = 0
    osd_stat_max_fft: int = 0
    osd_stat_total_flights: int = 0
    osd_stat_total_time: int = 0
    osd_stat_total_dist: int = 0
    osd_stat_min_rssi_dbm: int = 0
    osd_timer_count: int = 0
    osd_timer_1: int = 0
    osd_timer_2: int = 0
    enabledwarnings: int = 0
    osd_warning_count: int = 0            # 16
    enabledwarnings_1_41_plus: int = 0
    osd_profile_count: int = 0            # 1
    osdprofileindex: int = 0              # 1
    overlay_radio_mode: int = 0           # 0

    def serialize(self):
        return struct.pack('<' + 'I' * 100,
                           self.osdflags,
                           self.video_system,
                           self.units,
                           self.rssi_alarm,
                           self.cap_alarm,
                           self.old_timer_alarm,
                           self.osd_item_count,
                           self.alt_alarm,
                           self.osd_rssi_value_pos,
                           self.osd_main_batt_voltage_pos,
                           self.osd_crosshairs_pos,
                           self.osd_artificial_horizon_pos,
                           self.osd_horizon_sidebars_pos,
                           self.osd_item_timer_1_pos,
                           self.osd_item_timer_2_pos,
                           self.osd_flymode_pos,
                           self.osd_craft_name_pos,
                           self.osd_throttle_pos_pos,
                           self.osd_vtx_channel_pos,
                           self.osd_current_draw_pos,
                           self.osd_mah_drawn_pos,
                           self.osd_gps_speed_pos,
                           self.osd_gps_sats_pos,
                           self.osd_altitude_pos,
                           self.osd_roll_pids_pos,
                           self.osd_pitch_pids_pos,
                           self.osd_yaw_pids_pos,
                           self.osd_power_pos,
                           self.osd_pidrate_profile_pos,
                           self.osd_warnings_pos,
                           self.osd_avg_cell_voltage_pos,
                           self.osd_gps_lon_pos,
                           self.osd_gps_lat_pos,
                           self.osd_debug_pos,
                           self.osd_pitch_angle_pos,
                           self.osd_roll_angle_pos,
                           self.osd_main_batt_usage_pos,
                           self.osd_disarmed_pos,
                           self.osd_home_dir_pos,
                           self.osd_home_dist_pos,
                           self.osd_numerical_heading_pos,
                           self.osd_numerical_vario_pos,
                           self.osd_compass_bar_pos,
                           self.osd_esc_tmp_pos,
                           self.osd_esc_rpm_pos,
                           self.osd_remaining_time_estimate_pos,
                           self.osd_rtc_datetime_pos,
                           self.osd_adjustment_range_pos,
                           self.osd_core_temperature_pos,
                           self.osd_anti_gravity_pos,
                           self.osd_g_force_pos,
                           self.osd_motor_diag_pos,
                           self.osd_log_status_pos,
                           self.osd_flip_arrow_pos,
                           self.osd_link_quality_pos,
                           self.osd_flight_dist_pos,
                           self.osd_stick_overlay_left_pos,
                           self.osd_stick_overlay_right_pos,
                           self.osd_display_name_pos,
                           self.osd_esc_rpm_freq_pos,
                           self.osd_rate_profile_name_pos,
                           self.osd_pid_profile_name_pos,
                           self.osd_profile_name_pos,
                           self.osd_rssi_dbm_value_pos,
                           self.osd_rc_channels_pos,
                           self.osd_stat_count,
                           self.osd_stat_rtc_date_time,
                           self.osd_stat_timer_1,
                           self.osd_stat_timer_2,
                           self.osd_stat_max_speed,
                           self.osd_stat_max_distance,
                           self.osd_stat_min_battery,
                           self.osd_stat_end_battery,
                           self.osd_stat_battery,
                           self.osd_stat_min_rssi,
                           self.osd_stat_max_current,
                           self.osd_stat_used_mah,
                           self.osd_stat_max_altitude,
                           self.osd_stat_blackbox,
                           self.osd_stat_blackbox_number,
                           self.osd_stat_max_g_force,
                           self.osd_stat_max_esc_temp,
                           self.osd_stat_max_esc_rpm,
                           self.osd_stat_min_link_quality,
                           self.osd_stat_flight_distance,
                           self.osd_stat_max_fft,
                           self.osd_stat_total_flights,
                           self.osd_stat_total_time,
                           self.osd_stat_total_dist,
                           self.osd_stat_min_rssi_dbm,
                           self.osd_timer_count,
                           self.osd_timer_1,
                           self.osd_timer_2,
                           self.enabledwarnings,
                           self.osd_warning_count,
                           self.enabledwarnings_1_41_plus,
                           self.osd_profile_count,
                           self.osdprofileindex,
                           self.overlay_radio_mode)

@dataclass
class MspName:
    craft_name: str                   # 15 characters max possible displayed in the goggles

    def serialize(self):
        name_bytes = self.craft_name.encode('ascii')
        if len(name_bytes) > 15:
            raise ValueError("craft_name cannot exceed 15 ASCII characters.")
        name_bytes_padded = name_bytes.ljust(15, b'\x00')
        return name_bytes_padded

@dataclass
class MspBatteryState:
    batteryCellCount: int
    batteryCapacity: int
    legacyBatteryVoltage: int
    mAhDrawn: int
    amperage: int
    batteryState: int
    batteryVoltage: int

    def serialize(self):
        return struct.pack('<IIIIIII',
                           self.batteryCellCount,
                           self.batteryCapacity,
                           self.legacyBatteryVoltage,
                           self.mAhDrawn,
                           self.amperage,
                           self.batteryState,
                           self.batteryVoltage)

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

    def serialize(self):
        return struct.pack('<IIIIIIIIII',
                           self.cycleTime,
                           self.i2cErrorCounter,
                           self.sensor,
                           self.flightModeFlags,
                           self.configProfileIndex,
                           self.averageSystemLoadPercent,
                           self.armingFlags,
                           self.accCalibrationAxisFlags,
                           self.DJI_ARMING_DISABLE_FLAGS_COUNT,
                           self.djiPackArmingDisabledFlags)
