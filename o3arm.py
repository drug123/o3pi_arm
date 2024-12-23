import time
import serial
from MSP import MSP
from MSP_OSD import *
from OSD_positions_config import *
from MSP_constants import *
from MSP_structs import *

# Configure the serial port
MSP_PORT = '/dev/serial0'  # Replace with the correct serial port
BAUD_RATE = 115200

# Arm Logic
unarmedMillis = 3000  # Wait time before arming in milliseconds

# Voltage and Battery Reading
vbat = 0
batteryCellCount = 3

# Flight Mode Flags
flightModeFlags = 0x00000002
previousFlightMode = flightModeFlags

# Initialize MSP data structures
battery_state = MspBatteryState(0, 0, 0, 0, 0, 0, 0)
name = MspName(craft_name="Rert")
fc_version = MSPFCVersion(version_major=4, version_minor=1, version_patch_level=1)
fc_variant = MSPFCVariant(flight_control_identifier="BTFL")
status_DJI = MspStatusDJI(
    cycleTime=0x0080,
    i2cErrorCounter=0,
    sensor=0x23,
    configProfileIndex=0,
    averageSystemLoadPercent=7,
    accCalibrationAxisFlags=0,
    DJI_ARMING_DISABLE_FLAGS_COUNT=20,
    djiPackArmingDisabledFlags=(1 << 24)
)

def main():
    print("main()")
    global flightModeFlags, previousFlightMode, vbat, batteryCellCount

    # Initialize serial communication
    ser = serial.Serial(MSP_PORT, BAUD_RATE)
    msp = MSP(ser)

    activityDetected = False
    activityDetectedMillis_MSP = 0
    general_counter = 0

    while True:
        if not activityDetected:
            # Wait for Air Unit to send data
            if msp.activity_detected():
                activityDetected = True
                activityDetectedMillis_MSP = current_millis()
        else:
            currentMillis_MSP = current_millis()
            if (currentMillis_MSP - activityDetectedMillis_MSP) >= unarmedMillis:
                set_flight_mode_flags(True)
            else:
                set_flight_mode_flags(False)

            send_msp_to_airunit(msp)
            general_counter += 100  # Adjust as needed
            time.sleep(0.1)

def current_millis():
    print("current_millis()")
    return int(round(time.time() * 1000))

def set_flight_mode_flags(arm):
    print(f"set_flight_mode_flags(arm={arm})")
    global flightModeFlags
    if flightModeFlags == 0x00000002 and arm:
        flightModeFlags = 0x00000003  # Arm
    elif flightModeFlags == 0x00000003 and not arm:
        flightModeFlags = 0x00000002  # Disarm

def send_msp_to_airunit(msp):
    print(f"send_msp_to_airunit(msp={msp})")
    # Send MSP messages
    msp.send(MSP_FC_VARIANT, fc_variant)
    msp.send(MSP_FC_VERSION, fc_version)
    msp.send(MSP_NAME, name)
    
    # Update status
    status_DJI.flightModeFlags = flightModeFlags
    status_DJI.armingFlags = 0x0303 
    msp.send(MSP_STATUS_EX, status_DJI)
    status_DJI.armingFlags = 0x0000
    msp.send(MSP_STATUS, status_DJI)

    # Update battery state
    update_battery_state()
    msp.send(MSP_BATTERY_STATE, battery_state)

    # Send OSD config
    send_osd_config(msp)

def update_battery_state():
    print("update_battery_state()")
    global vbat, batteryCellCount
    # Read voltage (replace with actual voltage reading code)
    vbat = getVoltageSample()
    # Set battery cell count based on voltage
    if vbat < 43:
        batteryCellCount = 1
    elif vbat < 85:
        batteryCellCount = 2
    elif vbat < 127:
        batteryCellCount = 3
    elif vbat < 169:
        batteryCellCount = 4
    elif vbat < 211:
        batteryCellCount = 5
    else:
        batteryCellCount = 6

    battery_state.amperage = 0
    battery_state.batteryVoltage = vbat * 10
    battery_state.mAhDrawn = 0
    battery_state.batteryCellCount = batteryCellCount
    battery_state.batteryCapacity = 0
    battery_state.batteryState = 0
    battery_state.legacyBatteryVoltage = vbat

def getVoltageSample():
    print("getVoltageSample()")
    # Implement voltage reading from Raspberry Pi GPIO or ADC
    # Return voltage value
    return 0  # Placeholder

def send_osd_config(msp):
    print(f"send_osd_config(msp={msp})")
    msp_osd_config = MspOsdConfig()
    # Set OSD positions from OSD_positions_config
    msp_osd_config.units = 1

    msp_osd_config.osd_item_count = 56
    msp_osd_config.osd_stat_count = 24
    msp_osd_config.osd_timer_count = 2
    msp_osd_config.osd_warning_count = 16
    msp_osd_config.osd_profile_count = 1
    msp_osd_config.osdprofileindex = 1
    msp_osd_config.overlay_radio_mode = 0

    msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos
    msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos
    msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos
    msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos
    msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos
    msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos
    msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos
    msp_osd_config.osd_flymode_pos = osd_flymode_pos
    msp_osd_config.osd_craft_name_pos = osd_craft_name_pos
    msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos
    msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos
    msp_osd_config.osd_current_draw_pos = osd_current_draw_pos
    msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos
    msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos
    msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos
    msp_osd_config.osd_altitude_pos = osd_altitude_pos
    msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos
    msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos
    msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos
    msp_osd_config.osd_power_pos = osd_power_pos
    msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos
    msp_osd_config.osd_warnings_pos = osd_warnings_pos
    msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos
    msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos
    msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos
    msp_osd_config.osd_debug_pos = osd_debug_pos
    msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos
    msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos
    msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos
    msp_osd_config.osd_disarmed_pos = osd_disarmed_pos
    msp_osd_config.osd_home_dir_pos = osd_home_dir_pos
    msp_osd_config.osd_home_dist_pos = osd_home_dist_pos
    msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos
    msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos
    msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos
    msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos
    msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos
    msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos
    msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos
    msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos
    msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos
    msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos
    msp_osd_config.osd_g_force_pos = osd_g_force_pos
    msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos
    msp_osd_config.osd_log_status_pos = osd_log_status_pos
    msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos
    msp_osd_config.osd_link_quality_pos = osd_link_quality_pos
    msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos
    msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos
    msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos
    msp_osd_config.osd_display_name_pos = osd_display_name_pos
    msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos
    msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos
    msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos
    msp_osd_config.osd_profile_name_pos = osd_profile_name_pos
    msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos
    msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos

    msp.send(MSP_OSD_CONFIG, msp_osd_config)

if __name__ == "__main__":
    main()