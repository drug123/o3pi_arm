from dataclasses import dataclass, field, MISSING
from typing import List

@dataclass
class MSPApiVersion:
    protocol_version: int
    api_major: int
    api_minor: int

@dataclass
class MSPStatus:
    cycle_time: int
    i2c_error_counter: int
    sensor: int
    flight_mode_flags: int
    config_profile_index: int

@dataclass
class MSPFCVariant:
    flight_control_identifier: str = field(metadata={"length": 4})

@dataclass
class MSPFCVersion:
    version_major: int
    version_minor: int
    version_patch_level: int


@dataclass
class MSPBoardInfo:
    board_identifier: str = field(metadata={"length": 4})
    hardware_revision: int = field(default_factory=MISSING)

@dataclass
class MSPBuildInfo:
    build_date: str = field(metadata={"length": 11})
    build_time: str = field(metadata={"length": 8})
    short_git_revision: str = field(metadata={"length": 7})

@dataclass
class MSPRawIMU:
    acc: List[int] = field(default_factory=lambda: [0, 0, 0])
    gyro: List[int] = field(default_factory=lambda: [0, 0, 0])
    mag: List[int] = field(default_factory=lambda: [0, 0, 0])

@dataclass
class MSPSensorStatus:
    is_hardware_healthy: int
    hw_gyro_status: int
    hw_accelerometer_status: int
    hw_compass_status: int
    hw_barometer_status: int
    hw_gps_status: int
    hw_rangefinder_status: int
    hw_pitotmeter_status: int
    hw_optical_flow_status: int

@dataclass
class MSPServo:
    servo: List[int] = field(default_factory=lambda: [0] * 8)

@dataclass
class MSPServoConfigurations:
    conf: List[dict] = field(default_factory=lambda: [{}] * 8)

@dataclass
class MSPMotor:
    motor: List[int] = field(default_factory=lambda: [0] * 8)

@dataclass
class MSPRC:
    channel_value: List[int] = field(default_factory=lambda: [0] * 16)

@dataclass
class MSPAttitude:
    roll: int
    pitch: int
    yaw: int

@dataclass
class MSPAltitude:
    estimated_actual_position: int
    estimated_actual_velocity: int
    baro_latest_altitude: int

@dataclass
class MSPSonarAltitude:
    altitude: int

@dataclass
class MSPAnalog:
    vbat: int
    mah_drawn: int
    rssi: int
    amperage: int

@dataclass
class MSPArmingConfig:
    auto_disarm_delay: int
    disarm_kill_switch: int

@dataclass
class MSPLoopTime:
    looptime: int

@dataclass
class MSPRCTuning:
    rc_rate8: int
    rc_expo8: int
    rates: List[int] = field(default_factory=lambda: [0, 0, 0])
    dyn_thr_pid: int = field(default_factory=MISSING)
    thr_mid8: int = field(default_factory=MISSING)
    thr_expo8: int = field(default_factory=MISSING)
    tpa_breakpoint: int = field(default_factory=MISSING)
    rc_yaw_expo8: int = field(default_factory=MISSING)

@dataclass
class MSPPID:
    roll: List[int] = field(default_factory=lambda: [0, 0, 0])
    pitch: List[int] = field(default_factory=lambda: [0, 0, 0])
    yaw: List[int] = field(default_factory=lambda: [0, 0, 0])
    pos_z: List[int] = field(default_factory=lambda: [0, 0, 0])
    pos_xy: List[int] = field(default_factory=lambda: [0, 0, 0])
    vel_xy: List[int] = field(default_factory=lambda: [0, 0, 0])
    surface: List[int] = field(default_factory=lambda: [0, 0, 0])
    level: List[int] = field(default_factory=lambda: [0, 0, 0])
    heading: List[int] = field(default_factory=lambda: [0, 0, 0])
    vel_z: List[int] = field(default_factory=lambda: [0, 0, 0])

@dataclass
class MSPMisc:
    midrc: int
    minthrottle: int
    maxthrottle: int
    mincommand: int
    failsafe_throttle: int
    gps_provider: int
    gps_baudrate: int
    gps_ubx_sbas: int
    multiwii_current_meter_output: int
    rssi_channel: int
    dummy: int
    mag_declination: int
    vbatscale: int
    vbatmincellvoltage: int
    vbatmaxcellvoltage: int
    vbatwarningcellvoltage: int

@dataclass
class MSPRawGPS:
    fix_type: int
    num_sat: int
    lat: int
    lon: int
    alt: int
    ground_speed: int
    ground_course: int
    hdop: int

@dataclass
class MSPCompGPS:
    distance_to_home: int
    direction_to_home: int
    heartbeat: int

@dataclass
class MSPNavStatus:
    mode: int
    state: int
    active_wp_action: int
    active_wp_number: int
    error: int
    mag_hold_heading: int

@dataclass
class MSPGPSSVInfo:
    dummy1: int
    dummy2: int
    dummy3: int
    dummy4: int
    hdop: int

@dataclass
class MSPGPSStatistics:
    last_message_dt: int
    errors: int
    timeouts: int
    packet_count: int
    hdop: int
    eph: int
    epv: int

@dataclass
class MSPUID:
    uid0: int
    uid1: int
    uid2: int

@dataclass
class MSPFeature:
    feature_mask: int

@dataclass
class MSPBoardAlignment:
    roll_deci_degrees: int
    pitch_deci_degrees: int
    yaw_deci_degrees: int

@dataclass
class MSPCurrentMeterConfig:
    current_meter_scale: int
    current_meter_offset: int
    current_meter_type: int
    battery_capacity: int

@dataclass
class MSPRXConfig:
    serialrx_provider: int
    maxcheck: int
    midrc: int
    mincheck: int
    spektrum_sat_bind: int
    rx_min_usec: int
    rx_max_usec: int
    dummy1: int
    dummy2: int
    dummy3: int
    rx_spi_protocol: int
    rx_spi_id: int
    rx_spi_rf_channel_count: int

@dataclass
class MSPRXMap:
    rxmap: List[int] = field(default_factory=lambda: [0] * 8)

@dataclass
class MSPSensorAlignment:
    gyro_align: int
    acc_align: int
    mag_align: int

@dataclass
class MSPCalibrationData:
    acc_zero_x: int
    acc_zero_y: int
    acc_zero_z: int
    acc_gain_x: int
    acc_gain_y: int
    acc_gain_z: int
    mag_zero_x: int
    mag_zero_y: int
    mag_zero_z: int

@dataclass
class MSPSetHead:
    mag_hold_heading: int

@dataclass
class MSPSetRawRC:
    channel: List[int] = field(default_factory=lambda: [0] * 16)

@dataclass
class MSPSetPID:
    roll: List[int] = field(default_factory=lambda: [0, 0, 0])
    pitch: List[int] = field(default_factory=lambda: [0, 0, 0])
    yaw: List[int] = field(default_factory=lambda: [0, 0, 0])
    pos_z: List[int] = field(default_factory=lambda: [0, 0, 0])
    pos_xy: List[int] = field(default_factory=lambda: [0, 0, 0])
    vel_xy: List[int] = field(default_factory=lambda: [0, 0, 0])
    surface: List[int] = field(default_factory=lambda: [0, 0, 0])
    level: List[int] = field(default_factory=lambda: [0, 0, 0])
    heading: List[int] = field(default_factory=lambda: [0, 0, 0])
    vel_z: List[int] = field(default_factory=lambda: [0, 0, 0])

@dataclass
class MSPSetRawGPS:
    fix_type: int
    num_sat: int
    lat: int
    lon: int
    alt: int
    ground_speed: int

@dataclass
class MSPSetWP:
    waypoint_number: int
    action: int
    lat: int
    lon: int
    alt: int
    p1: int
    p2: int
    p3: int
    flag: int