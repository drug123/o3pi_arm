import serial
import struct
import time

# Constants for MSP message IDs
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_BUILD_INFO = 5
MSP_CALIBRATION_DATA = 14
MSP_FEATURE = 36
MSP_BOARD_ALIGNMENT = 38
MSP_CURRENT_METER_CONFIG = 40
MSP_RX_CONFIG = 44
MSP_SONAR_ALTITUDE = 58
MSP_ARMING_CONFIG = 61
MSP_RX_MAP = 64
MSP_LOOP_TIME = 73
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_SERVO = 103
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_RC_TUNING = 111
MSP_PID = 112
MSP_MISC = 114
MSP_SERVO_CONFIGURATIONS = 120
MSP_NAV_STATUS = 121
MSP_SENSOR_ALIGNMENT = 126
MSP_STATUS_EX = 150
MSP_SENSOR_STATUS = 151
MSP_BOXIDS = 119
MSP_UID = 160
MSP_GPSSVINFO = 164
MSP_GPSSTATISTICS = 166
MSP_SET_PID = 202

# MSP commands
MSP_SET_HEAD = 211
MSP_SET_RAW_RC = 200
MSP_SET_RAW_GPS = 201
MSP_SET_WP = 209

# MSP mode bits
MSP_MODE_ARM = 0
MSP_MODE_ANGLE = 1
MSP_MODE_HORIZON = 2
MSP_MODE_NAVALTHOLD = 3
MSP_MODE_MAG = 4
MSP_MODE_HEADFREE = 5
MSP_MODE_HEADADJ = 6
MSP_MODE_CAMSTAB = 7
MSP_MODE_NAVRTH = 8
MSP_MODE_NAVPOSHOLD = 9
MSP_MODE_PASSTHRU = 10
MSP_MODE_BEEPERON = 11
MSP_MODE_LEDLOW = 12
MSP_MODE_LLIGHTS = 13
MSP_MODE_OSD = 14
MSP_MODE_TELEMETRY = 15
MSP_MODE_GTUNE = 16
MSP_MODE_SONAR = 17
MSP_MODE_BLACKBOX = 18
MSP_MODE_FAILSAFE = 19
MSP_MODE_NAVWP = 20
MSP_MODE_AIRMODE = 21
MSP_MODE_HOMERESET = 22
MSP_MODE_GCSNAV = 23
MSP_MODE_HEADINGLOCK = 24
MSP_MODE_SURFACE = 25
MSP_MODE_FLAPERON = 26
MSP_MODE_TURNASSIST = 27
MSP_MODE_NAVLAUNCH = 28
MSP_MODE_AUTOTRIM = 29

# Data structures for MSP messages
class MSPApiVersion:
    def __init__(self, protocol_version=0, api_major=0, api_minor=0):
        self.protocol_version = protocol_version
        self.api_major = api_major
        self.api_minor = api_minor
    
    def serialize(self):
        return struct.pack('<HHH', self.protocol_version, self.api_major, self.api_minor)

class MSPStatus:
    def __init__(self, cycle_time=0, i2c_error_counter=0, sensor=0, flight_mode_flags=0, config_profile_index=0):
        self.cycle_time = cycle_time
        self.i2c_error_counter = i2c_error_counter
        self.sensor = sensor
        self.flight_mode_flags = flight_mode_flags
        self.config_profile_index = config_profile_index
    
    def serialize(self):
        return struct.pack('<HHHBB', 
                           self.cycle_time, 
                           self.i2c_error_counter, 
                           self.sensor, 
                           self.flight_mode_flags, 
                           self.config_profile_index)

class MSPFCVariant:
    def __init__(self, flight_control_identifier='FC01'):
        self.flight_control_identifier = flight_control_identifier
    
    def serialize(self):
        identifier_bytes = self.flight_control_identifier.encode('ascii')
        if len(identifier_bytes) != 4:
            raise ValueError("flight_control_identifier must be exactly 4 ASCII characters.")
        return identifier_bytes

class MSPFCVersion:
    def __init__(self, version_major=0, version_minor=0, version_patch_level=0):
        self.version_major = version_major
        self.version_minor = version_minor
        self.version_patch_level = version_patch_level
    
    def serialize(self):
        return struct.pack('<HHH', self.version_major, self.version_minor, self.version_patch_level)

class MSPBoardInfo:
    def __init__(self, hardware_revision=0, board_identifier='B001'):
        self.hardware_revision = hardware_revision
        self.board_identifier = board_identifier
    
    def serialize(self):
        identifier_bytes = self.board_identifier.encode('ascii')
        if len(identifier_bytes) != 4:
            raise ValueError("board_identifier must be exactly 4 ASCII characters.")
        return identifier_bytes + struct.pack('<I', self.hardware_revision)

class MSPBuildInfo:
    def __init__(self, build_date='20230101', build_time='120000', short_git_revision='abc1234'):
        self.build_date = build_date
        self.build_time = build_time
        self.short_git_revision = short_git_revision
    
    def serialize(self):
        return (self.build_date.encode('ascii') +
                self.build_time.encode('ascii') +
                self.short_git_revision.encode('ascii'))

class MSPRawIMU:
    def __init__(self, acc=None, gyro=None, mag=None):
        self.acc = acc if acc is not None else [0, 0, 0]
        self.gyro = gyro if gyro is not None else [0, 0, 0]
        self.mag = mag if mag is not None else [0, 0, 0]
    
    def serialize(self):
        return struct.pack('<iii', *self.acc) + struct.pack('<iii', *self.gyro) + struct.pack('<iii', *self.mag)

class MSPSensorStatus:
    def __init__(self, is_hardware_healthy=0, hw_gyro_status=0, hw_accelerometer_status=0,
                 hw_compass_status=0, hw_barometer_status=0, hw_gps_status=0,
                 hw_rangefinder_status=0, hw_pitotmeter_status=0, hw_optical_flow_status=0):
        self.is_hardware_healthy = is_hardware_healthy
        self.hw_gyro_status = hw_gyro_status
        self.hw_accelerometer_status = hw_accelerometer_status
        self.hw_compass_status = hw_compass_status
        self.hw_barometer_status = hw_barometer_status
        self.hw_gps_status = hw_gps_status
        self.hw_rangefinder_status = hw_rangefinder_status
        self.hw_pitotmeter_status = hw_pitotmeter_status
        self.hw_optical_flow_status = hw_optical_flow_status
    
    def serialize(self):
        return struct.pack('<IIIIIIIII',
                           self.is_hardware_healthy,
                           self.hw_gyro_status,
                           self.hw_accelerometer_status,
                           self.hw_compass_status,
                           self.hw_barometer_status,
                           self.hw_gps_status,
                           self.hw_rangefinder_status,
                           self.hw_pitotmeter_status,
                           self.hw_optical_flow_status)

class MSPServo:
    def __init__(self, servo=None):
        self.servo = servo if servo is not None else [0] * 8
    
    def serialize(self):
        return struct.pack('<' + 'H' * 8, *self.servo)

class MSPServoConfigurations:
    def __init__(self, conf=None):
        self.conf = conf if conf is not None else [{}] * 8
    
    def serialize(self):
        # Example serialization, adjust based on actual structure
        serialized_conf = b''.join([struct.pack('<I', 0) for _ in self.conf])  # Placeholder
        return serialized_conf

class MSPMotor:
    def __init__(self, motor=None):
        self.motor = motor if motor is not None else [0] * 8
    
    def serialize(self):
        return struct.pack('<' + 'H' * 8, *self.motor)

class MSPRC:
    def __init__(self, channel_value=None):
        self.channel_value = channel_value if channel_value is not None else [0] * 16
    
    def serialize(self):
        return struct.pack('<' + 'H' * 16, *self.channel_value)

class MSPAttitude:
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
    def serialize(self):
        return struct.pack('<hhh', self.roll, self.pitch, self.yaw)

class MSPAltitude:
    def __init__(self, estimated_actual_position=0, estimated_actual_velocity=0, baro_latest_altitude=0):
        self.estimated_actual_position = estimated_actual_position
        self.estimated_actual_velocity = estimated_actual_velocity
        self.baro_latest_altitude = baro_latest_altitude
    
    def serialize(self):
        return struct.pack('<iii', self.estimated_actual_position,
                           self.estimated_actual_velocity, self.baro_latest_altitude)

class MSPSonarAltitude:
    def __init__(self, altitude=0):
        self.altitude = altitude
    
    def serialize(self):
        return struct.pack('<i', self.altitude)

class MSPAnalog:
    def __init__(self, vbat=0, mah_drawn=0, rssi=0, amperage=0):
        self.vbat = vbat
        self.mah_drawn = mah_drawn
        self.rssi = rssi
        self.amperage = amperage

    def serialize(self):
        return struct.pack('<IIII', self.vbat, self.mah_drawn, self.rssi, self.amperage)

class MSPArmingConfig:
    def __init__(self, auto_disarm_delay=0, disarm_kill_switch=0):
        self.auto_disarm_delay = auto_disarm_delay
        self.disarm_kill_switch = disarm_kill_switch
    
    def serialize(self):
        return struct.pack('<BB', self.auto_disarm_delay, self.disarm_kill_switch)

class MSPLoopTime:
    def __init__(self, looptime=0):
        self.looptime = looptime
    
    def serialize(self):
        return struct.pack('<I', self.looptime)

class MSPRCTuning:
    def __init__(self, rc_rate8=0, rc_expo8=0, dyn_thr_pid=0, thr_mid8=0, thr_expo8=0,
                 tpa_breakpoint=0, rc_yaw_expo8=0, rates=None):
        self.rc_rate8 = rc_rate8
        self.rc_expo8 = rc_expo8
        self.dyn_thr_pid = dyn_thr_pid
        self.thr_mid8 = thr_mid8
        self.thr_expo8 = thr_expo8
        self.tpa_breakpoint = tpa_breakpoint
        self.rc_yaw_expo8 = rc_yaw_expo8
        self.rates = rates if rates is not None else [0, 0, 0]
    
    def serialize(self):
        header = struct.pack('<BBIIBBB',
                             self.rc_rate8,
                             self.rc_expo8,
                             self.dyn_thr_pid,
                             self.thr_mid8,
                             self.thr_expo8,
                             self.tpa_breakpoint,
                             self.rc_yaw_expo8)
        rates_packed = struct.pack('<III', *self.rates)
        return header + rates_packed

class MSPPID:
    def __init__(self, roll=None, pitch=None, yaw=None, pos_z=None, pos_xy=None,
                 vel_xy=None, surface=None, level=None, heading=None, vel_z=None):
        self.roll = roll if roll is not None else [0, 0, 0]
        self.pitch = pitch if pitch is not None else [0, 0, 0]
        self.yaw = yaw if yaw is not None else [0, 0, 0]
        self.pos_z = pos_z if pos_z is not None else [0, 0, 0]
        self.pos_xy = pos_xy if pos_xy is not None else [0, 0, 0]
        self.vel_xy = vel_xy if vel_xy is not None else [0, 0, 0]
        self.surface = surface if surface is not None else [0, 0, 0]
        self.level = level if level is not None else [0, 0, 0]
        self.heading = heading if heading is not None else [0, 0, 0]
        self.vel_z = vel_z if vel_z is not None else [0, 0, 0]
    
    def serialize(self):
        return (struct.pack('<III', *self.roll) +
                struct.pack('<III', *self.pitch) +
                struct.pack('<III', *self.yaw) +
                struct.pack('<III', *self.pos_z) +
                struct.pack('<III', *self.pos_xy) +
                struct.pack('<III', *self.vel_xy) +
                struct.pack('<III', *self.surface) +
                struct.pack('<III', *self.level) +
                struct.pack('<III', *self.heading) +
                struct.pack('<III', *self.vel_z))

class MSPMisc:
    def __init__(self, midrc=0, minthrottle=0, maxthrottle=0, mincommand=0,
                 failsafe_throttle=0, gps_provider=0, gps_baudrate=0,
                 gps_ubx_sbas=0, multiwii_current_meter_output=0, rssi_channel=0,
                 dummy=0, mag_declination=0, vbatscale=0, vbatmincellvoltage=0,
                 vbatmaxcellvoltage=0, vbatwarningcellvoltage=0):
        self.midrc = midrc
        self.minthrottle = minthrottle
        self.maxthrottle = maxthrottle
        self.mincommand = mincommand
        self.failsafe_throttle = failsafe_throttle
        self.gps_provider = gps_provider
        self.gps_baudrate = gps_baudrate
        self.gps_ubx_sbas = gps_ubx_sbas
        self.multiwii_current_meter_output = multiwii_current_meter_output
        self.rssi_channel = rssi_channel
        self.dummy = dummy
        self.mag_declination = mag_declination
        self.vbatscale = vbatscale
        self.vbatmincellvoltage = vbatmincellvoltage
        self.vbatmaxcellvoltage = vbatmaxcellvoltage
        self.vbatwarningcellvoltage = vbatwarningcellvoltage
    
    def serialize(self):
        return struct.pack('<HHHHHHHHHHHHHHHH',
                           self.midrc,
                           self.minthrottle,
                           self.maxthrottle,
                           self.mincommand,
                           self.failsafe_throttle,
                           self.gps_provider,
                           self.gps_baudrate,
                           self.gps_ubx_sbas,
                           self.multiwii_current_meter_output,
                           self.rssi_channel,
                           self.dummy,
                           self.mag_declination,
                           self.vbatscale,
                           self.vbatmincellvoltage,
                           self.vbatmaxcellvoltage,
                           self.vbatwarningcellvoltage)

class MSPRawGPS:
    def __init__(self, fix_type=0, num_sat=0, lat=0, lon=0, alt=0, ground_speed=0, ground_course=0, hdop=0):
        self.fix_type = fix_type
        self.num_sat = num_sat
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.ground_speed = ground_speed
        self.ground_course = ground_course
        self.hdop = hdop
    
    def serialize(self):
        return struct.pack('<HHIIIIIH',
                           self.fix_type,
                           self.num_sat,
                           self.lat,
                           self.lon,
                           self.alt,
                           self.ground_speed,
                           self.ground_course,
                           self.hdop)

class MSPCompGPS:
    def __init__(self, distance_to_home=0, direction_to_home=0, heartbeat=0):
        self.distance_to_home = distance_to_home
        self.direction_to_home = direction_to_home
        self.heartbeat = heartbeat
    
    def serialize(self):
        return struct.pack('<III', self.distance_to_home, self.direction_to_home, self.heartbeat)

class MSPNavStatus:
    def __init__(self, mode=0, state=0, active_wp_action=0, active_wp_number=0, error=0, mag_hold_heading=0):
        self.mode = mode
        self.state = state
        self.active_wp_action = active_wp_action
        self.active_wp_number = active_wp_number
        self.error = error
        self.mag_hold_heading = mag_hold_heading
    
    def serialize(self):
        return struct.pack('<IIIIII', self.mode, self.state, self.active_wp_action,
                           self.active_wp_number, self.error, self.mag_hold_heading)

class MSPGPSSVInfo:
    def __init__(self, dummy1=0, dummy2=0, dummy3=0, dummy4=0, hdop=0):
        self.dummy1 = dummy1
        self.dummy2 = dummy2
        self.dummy3 = dummy3
        self.dummy4 = dummy4
        self.hdop = hdop
    
    def serialize(self):
        return struct.pack('<HHHHH', self.dummy1, self.dummy2, self.dummy3, self.dummy4, self.hdop)

class MSPGPSStatistics:
    def __init__(self, last_message_dt=0, errors=0, timeouts=0, packet_count=0, hdop=0, eph=0, epv=0):
        self.last_message_dt = last_message_dt
        self.errors = errors
        self.timeouts = timeouts
        self.packet_count = packet_count
        self.hdop = hdop
        self.eph = eph
        self.epv = epv
    
    def serialize(self):
        return struct.pack('<IIIIIII', self.last_message_dt, self.errors,
                           self.timeouts, self.packet_count, self.hdop,
                           self.eph, self.epv)

class MSPUID:
    def __init__(self, uid0=0, uid1=0, uid2=0):
        self.uid0 = uid0
        self.uid1 = uid1
        self.uid2 = uid2
    
    def serialize(self):
        return struct.pack('<III', self.uid0, self.uid1, self.uid2)

class MSPFeature:
    def __init__(self, feature_mask=0):
        self.feature_mask = feature_mask
    
    def serialize(self):
        return struct.pack('<I', self.feature_mask)

class MSPBoardAlignment:
    def __init__(self, roll_deci_degrees=0, pitch_deci_degrees=0, yaw_deci_degrees=0):
        self.roll_deci_degrees = roll_deci_degrees
        self.pitch_deci_degrees = pitch_deci_degrees
        self.yaw_deci_degrees = yaw_deci_degrees
    
    def serialize(self):
        return struct.pack('<III', self.roll_deci_degrees, self.pitch_deci_degrees, self.yaw_deci_degrees)

class MSPCurrentMeterConfig:
    def __init__(self, current_meter_scale=0, current_meter_offset=0, current_meter_type=0, battery_capacity=0):
        self.current_meter_scale = current_meter_scale
        self.current_meter_offset = current_meter_offset
        self.current_meter_type = current_meter_type
        self.battery_capacity = battery_capacity
    
    def serialize(self):
        return struct.pack('<HHHH', self.current_meter_scale,
                           self.current_meter_offset, self.current_meter_type,
                           self.battery_capacity)

class MSPRXConfig:
    def __init__(self, serialrx_provider=0, maxcheck=0, midrc=0, mincheck=0,
                 spektrum_sat_bind=0, rx_min_usec=0, rx_max_usec=0, dummy1=0,
                 dummy2=0, dummy3=0, rx_spi_protocol=0, rx_spi_id=0,
                 rx_spi_rf_channel_count=0):
        self.serialrx_provider = serialrx_provider
        self.maxcheck = maxcheck
        self.midrc = midrc
        self.mincheck = mincheck
        self.spektrum_sat_bind = spektrum_sat_bind
        self.rx_min_usec = rx_min_usec
        self.rx_max_usec = rx_max_usec
        self.dummy1 = dummy1
        self.dummy2 = dummy2
        self.dummy3 = dummy3
        self.rx_spi_protocol = rx_spi_protocol
        self.rx_spi_id = rx_spi_id
        self.rx_spi_rf_channel_count = rx_spi_rf_channel_count
    
    def serialize(self):
        return struct.pack('<HHHHHHHHHHHHH',
                           self.serialrx_provider,
                           self.maxcheck,
                           self.midrc,
                           self.mincheck,
                           self.spektrum_sat_bind,
                           self.rx_min_usec,
                           self.rx_max_usec,
                           self.dummy1,
                           self.dummy2,
                           self.dummy3,
                           self.rx_spi_protocol,
                           self.rx_spi_id,
                           self.rx_spi_rf_channel_count)

class MSPRXMap:
    def __init__(self, rxmap=None):
        self.rxmap = rxmap if rxmap is not None else [0] * 8
    
    def serialize(self):
        return struct.pack('<' + 'B' * 8, *self.rxmap)

class MSPSensorAlignment:
    def __init__(self, gyro_align=0, acc_align=0, mag_align=0):
        self.gyro_align = gyro_align
        self.acc_align = acc_align
        self.mag_align = mag_align
    
    def serialize(self):
        return struct.pack('<III', self.gyro_align, self.acc_align, self.mag_align)

class MSPCalibrationData:
    def __init__(self, acc_zero_x=0, acc_zero_y=0, acc_zero_z=0,
                 acc_gain_x=0, acc_gain_y=0, acc_gain_z=0,
                 mag_zero_x=0, mag_zero_y=0, mag_zero_z=0):
        self.acc_zero_x = acc_zero_x
        self.acc_zero_y = acc_zero_y
        self.acc_zero_z = acc_zero_z
        self.acc_gain_x = acc_gain_x
        self.acc_gain_y = acc_gain_y
        self.acc_gain_z = acc_gain_z
        self.mag_zero_x = mag_zero_x
        self.mag_zero_y = mag_zero_y
        self.mag_zero_z = mag_zero_z
    
    def serialize(self):
        return struct.pack('<IIIIIIIII',
                           self.acc_zero_x, self.acc_zero_y, self.acc_zero_z,
                           self.acc_gain_x, self.acc_gain_y, self.acc_gain_z,
                           self.mag_zero_x, self.mag_zero_y, self.mag_zero_z)

class MSPSetHead:
    def __init__(self, mag_hold_heading=0):
        self.mag_hold_heading = mag_hold_heading
    
    def serialize(self):
        return struct.pack('<I', self.mag_hold_heading)

class MSPSetRawRC:
    def __init__(self, channel=None):
        self.channel = channel if channel is not None else [0] * 16
    
    def serialize(self):
        return struct.pack('<' + 'H' * 16, *self.channel)

class MSPSetPID:
    def __init__(self, roll=None, pitch=None, yaw=None, pos_z=None, pos_xy=None,
                 vel_xy=None, surface=None, level=None, heading=None, vel_z=None):
        self.roll = roll if roll is not None else [0, 0, 0]
        self.pitch = pitch if pitch is not None else [0, 0, 0]
        self.yaw = yaw if yaw is not None else [0, 0, 0]
        self.pos_z = pos_z if pos_z is not None else [0, 0, 0]
        self.pos_xy = pos_xy if pos_xy is not None else [0, 0, 0]
        self.vel_xy = vel_xy if vel_xy is not None else [0, 0, 0]
        self.surface = surface if surface is not None else [0, 0, 0]
        self.level = level if level is not None else [0, 0, 0]
        self.heading = heading if heading is not None else [0, 0, 0]
        self.vel_z = vel_z if vel_z is not None else [0, 0, 0]
    
    def serialize(self):
        return (struct.pack('<III', *self.roll) +
                struct.pack('<III', *self.pitch) +
                struct.pack('<III', *self.yaw) +
                struct.pack('<III', *self.pos_z) +
                struct.pack('<III', *self.pos_xy) +
                struct.pack('<III', *self.vel_xy) +
                struct.pack('<III', *self.surface) +
                struct.pack('<III', *self.level) +
                struct.pack('<III', *self.heading) +
                struct.pack('<III', *self.vel_z))

class MSPSetRawGPS:
    def __init__(self, fix_type=0, num_sat=0, lat=0, lon=0, alt=0, ground_speed=0):
        self.fix_type = fix_type
        self.num_sat = num_sat
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.ground_speed = ground_speed
    
    def serialize(self):
        return struct.pack('<HHIIIH', self.fix_type, self.num_sat, self.lat,
                           self.lon, self.alt, self.ground_speed)

class MSPSetWP:
    def __init__(self, waypoint_number=0, action=0, lat=0, lon=0, alt=0, p1=0, p2=0, p3=0, flag=0):
        self.waypoint_number = waypoint_number
        self.action = action
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.flag = flag
    
    def serialize(self):
        return struct.pack('<HHIIIHHHH', self.waypoint_number, self.action,
                           self.lat, self.lon, self.alt, self.p1, self.p2,
                           self.p3, self.flag)

class MSP:
    def __init__(self, ser, timeout=0.5):
        self.ser = ser
        self.timeout = timeout

    def reset(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send(self, message_id, payload=b''):
        if hasattr(payload, 'serialize') and callable(payload.serialize):
            # Serialize the payload to bytes
            payload = payload.serialize()
        size = len(payload)
        header = b'$M<'
        checksum = size ^ message_id
        for byte in payload:
            checksum ^= byte
        frame = header + bytes([size]) + bytes([message_id]) + payload + bytes([checksum])
        self.ser.write(frame)

    def recv(self, message_id=None, max_size=256):
        start_time = time.time()
        while True:
            if self.ser.in_waiting >= 3:
                header = self.ser.read(3)
                if header == b'$M>' or header == b'$M<':
                    size = self.ser.read(1)[0]
                    recv_id = self.ser.read(1)[0]
                    payload = self.ser.read(size)
                    checksum = self.ser.read(1)[0]
                    checksum_calc = size ^ recv_id
                    for byte in payload:
                        checksum_calc ^= byte
                    if checksum == checksum_calc:
                        if message_id is None or recv_id == message_id:
                            return recv_id, payload
                else:
                    continue
            if time.time() - start_time >= self.timeout:
                return None, None

    def activity_detected(self):
        start_time = time.time()
        while True:
            if self.ser.in_waiting > 0:
                return True
            if time.time() - start_time >= self.timeout:
                return False

    def wait_for(self, message_id):
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            recv_id, payload = self.recv(message_id)
            if recv_id == message_id:
                return payload
        return None

    def request(self, message_id):
        self.send(message_id)
        return self.wait_for(message_id)

    def command(self, message_id, payload=b'', wait_ack=True):
        self.send(message_id, payload)
        if wait_ack:
            ack = self.wait_for(message_id)
            return ack is not None
        return True

    def get_active_modes(self):
        status_payload = self.request(MSP_STATUS)
        if status_payload:
            status_data = struct.unpack('<HHHIB', status_payload)
            status = MSPStatus(*status_data)
            boxids_payload = self.request(MSP_BOXIDS)
            if boxids_payload:
                boxids = list(boxids_payload)
                active_modes = 0
                for i, boxid in enumerate(boxids):
                    if status.flight_mode_flags & (1 << i):
                        if boxid in self.BOXIDS:
                            mode_bit = self.BOXIDS.index(boxid)
                            active_modes |= 1 << mode_bit
                return active_modes
        return None

    # Define BOXIDS as per C++ implementation
    BOXIDS = [
        0,  #  0: MSP_MODE_ARM
        1,  #  1: MSP_MODE_ANGLE
        2,  #  2: MSP_MODE_HORIZON
        3,  #  3: MSP_MODE_NAVALTHOLD (cleanflight BARO)
        5,  #  4: MSP_MODE_MAG
        6,  #  5: MSP_MODE_HEADFREE
        7,  #  6: MSP_MODE_HEADADJ
        8,  #  7: MSP_MODE_CAMSTAB
        10, #  8: MSP_MODE_NAVRTH (cleanflight GPSHOME)
        11, #  9: MSP_MODE_NAVPOSHOLD (cleanflight GPSHOLD)
        12, # 10: MSP_MODE_PASSTHRU
        13, # 11: MSP_MODE_BEEPERON
        15, # 12: MSP_MODE_LEDLOW
        16, # 13: MSP_MODE_LLIGHTS
        19, # 14: MSP_MODE_OSD
        20, # 15: MSP_MODE_TELEMETRY
        21, # 16: MSP_MODE_GTUNE
        22, # 17: MSP_MODE_SONAR
        26, # 18: MSP_MODE_BLACKBOX
        27, # 19: MSP_MODE_FAILSAFE
        28, # 20: MSP_MODE_NAVWP (cleanflight AIRMODE)
        29, # 21: MSP_MODE_AIRMODE (cleanflight DISABLE3DSWITCH)
        30, # 22: MSP_MODE_HOMERESET (cleanflight FPVANGLEMIX)
        31, # 23: MSP_MODE_GCSNAV (cleanflight BLACKBOXERASE)
        32, # 24: MSP_MODE_HEADINGLOCK
        33, # 25: MSP_MODE_SURFACE
        34, # 26: MSP_MODE_FLAPERON
        35, # 27: MSP_MODE_TURNASSIST
        36, # 28: MSP_MODE_NAVLAUNCH
        37, # 29: MSP_MODE_AUTOTRIM
    ]