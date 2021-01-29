# MSP Messages

These MSP messages correspond to INAV 2.6.0

### 1 - MSP_API_VERSION

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Protocol             | UINT 8               | 0 Same version as MSPv1 and MSPv2 - messages format didn't change and it is backward compatible |
| Major                | UINT 8               | 2 Increments when major changes are made |
| Minor                | UINT 8               | 4 Increments when any change is made, reset after major changes are made |

### 2 - MSP_FC_VARIANT

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Identifier           | string               | See table below. 4 characters |

| Identifier | Description |
| ---------- | ----------- |
| MWII       | MultiWii    |
| BAFL       | Baseflight  |
| BTFL       | Betaflight  |
| CLFL       | Cleanflight |
| INAV       | INav        |
| RCFL       | Raceflight  |

### 3 - MSP_FC_VERSION

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Major                | UINT 8               |                      |
| Minor                | UINT 8               |                      |
| Patch Level          | UINT 8               |                      |

### 4 - MSP_BOARD_INFO

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Identifier           | string               | 4 characters         |
| Version              | UINT 16              | TODO                 |
| OSD Support          | UINT 8               | TODO                 |
| Comms Capabilities   | UINT 8               | TODO                 |
| Name Length          | UINT 8               | Length of Name string |
| Name                 | string               |                      |

### 5 - MSP_BUILD_INFO

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Build Date           | string               | 11 characters        |
| Build Time           | string               | 8 characters         |
| Short Git Revision   | string               | 7 characters         |

### 6 - MSP_INAV_PID

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Async Mode                   | UINT 8       |                      |
| Accelerometer Task Frequency | UINT 16      |                      |
| Attitude Task Frequency      | UINT 16      |                      |
| Mag Hold Rate Limit          | UINT 8       |                      |
| Mag Hold Error LPF Frequency | UINT 8       |                      |
| Yaw Jump Prevention Limit    | UINT 16      |                      |
| Gyroscope LPF                | UINT 8       |                      |
| Accelerometer Soft LPF Hz    | UINT 8       |                      |
| Reserved                     | 4 x UINT 8   | 4 reserved bytes     |

### 7 - MSP_SET_INAV_PID

See: [MSP_INAV_PID](###-6---MSP_INAV_PID)

### 10 - MSP_NAME

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Name                 | string               |                      |

### 11 - MSP_SET_NAME

See: [MSP_NAME](###-10---MSP_NAME)

### 12 - MSP_NAV_POSHOLD

|         Data                    |       Data Type      |       Comments       |
| --------------------            | :------------------: | -------------------- |
| User Control Mode               | UINT 8               |                      |
| Max Speed                       | UINT 16              |                      |
| Max Climb Rate                  | UINT 16              |                      |
| Max Manual Speed                | UINT 16              |                      |
| Max Manual Climb Rate           | UINT 16              |                      |
| Max Bank Angle                  | UINT 8               |                      |
| Use Throttle Middle For Althold | UINT 8               |                      |
| Hover Throttle                  | UINT 16              |                      |

### 13 - MSP_SET_NAV_POSHOLD

See: [MSP_NAV_POSHOLD](###-12---MSP_NAV_POSHOLD)

### 14 - MSP_CALIBRATION_DATA

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Calibrations Flag    | UINT 8 bit field     | see table below      |
| accelerometer Zero X | UINT 16              |                      |
| Accelerometer Zero Y | UINT 16              |                      |
| Accelerometer Zero Z | UINT 16              |                      |
| Accelerometer Gain X | UINT 16              |                      |
| Accelerometer Gain Y | UINT 16              |                      |
| Accelerometer Gain Z | UINT 16              |                      |
| Magnetometer Zero X  | UINT 16              |                      |
| Magnetometer Zero Y  | UINT 16              |                      |
| Magnetometer Zero Z  | UINT 16              |                      |
| Optical Flow Scale   | UINT 16              | |  divided by 256.0 |
| Magnetometer Gain X  | UINT 16              |                      |
| Magnetometer Gain Y  | UINT 16              |                      |
| Magnetometer Gain Z  | UINT 16              |                      |

|         Data         |       Bit Number     |
| -------------------- | :------------------: |
| Accelerometer  Pos 0 | 0 -LSB-              |
| Accelerometer  Pos 1 | 1                    |
| Accelerometer  Pos 2 | 2                    |
| Accelerometer  Pos 3 | 3                    |
| Accelerometer  Pos 4 | 4                    |
| Accelerometer  Pos 5 | 5 -MSB-              |

### 15 - MSP_SET_CALIBRATION_DATA

Same struct as [MSP_CALIBRATION_DATA](###-14---MSP_CALIBRATION_DATA) without Calibration Flag

### 16 - MSP_POSITION_ESTIMATION_CONFIG

TODO get data names

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Vertical position barometer weight  | UINT 16              | Divide by 100        |
| Vertical position GPS weight        | UINT 16              | Divide by 100        |
| Vertical speed GPS weight           | UINT 16              | Divide by 100        |
| Horizontal position GPS weight      | UINT 16              | Divide by 100        |
| Horizontal speed GPS weight         | UINT 16              | Divide by 100        |
| Min. GPS satellites for a valid fix | UINT 8               |                      |
| use_gps_velned                      | bool                 |                      |

### 17 - MSP_SET_POSITION_ESTIMATION_CONFIG

See: [MSP_POSITION_ESTIMATION_CONFIG](###-16---MSP_POSITION_ESTIMATION_CONFIG)

### 18 - MSP_WP_MISSION_LOAD

TODO: Empty data

### 19 - MSP_WP_MISSION_SAVE

TODO: Empty data

### 20 - MSP_WP_GETINFO

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Waypoint Capabilities | UINT 8              |                      |
| Max Waypoints         | UINT 8              |                      |
| Is Valid Mission      | Bool                |                      |
| Count Busy Points     | UINT 8              |                      |

### 21 - MSP_RTH_AND_LAND_CONFIG

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Min RTH Distance           | UINT 16        |                      |
| RTH Climb First            | UINT 8         |                      |
| RTH Climb Ignore Emergency | UINT 8         |                      |
| RTH Tail First             | UINT 8         |                      |
| RTH Allow Landing          | UINT 8         |                      |
| RTH Alt Control Mode       | UINT 8         |                      |
| RTH Abort Threshold        | UINT 16        |                      |
| RTH Altitude               | UINT 16        |                      |
| Land Descent Rate          | UINT 16        |                      |
| Land Slowdown Min Alt      | UINT 16        |                      |
| Land Slowdown Max Alt      | UINT 16        |                      |
| Emergency Descent Rate     | UINT 16        |                      |

### 22 - MSP_SET_RTH_AND_LAND_CONFIG

See: [MSP_SET_RTH_AND_LAND_CONFIG](###-22---MSP_SET_RTH_AND_LAND_CONFIG)

### 23 - MSP_FW_CONFIG

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Cruise Throttle      | UINT 16              |                      |
| Min Throttle         | UINT 16              |                      |
| Max Throttle         | UINT 16              |                      |
| Max Bank Angle       | UINT 8               |                      |
| Max Climb Angle      | UINT 8               |                      |
| Max Dive Angle       | UINT 8               |                      |
| Pitch To Throttle    | UINT 8               |                      |
| Loiter Radius        | UINT 16              |                      |

### 24 - MSP_SET_FW_CONFIG

See: [MSP_FW_CONFIG](###-23---MSP_FW_CONFIG)

### 34 - MSP_MODE_RANGES

Array of multiple Mode Ranges packed in the format below. A maximum number of 20 modes can be packed in the array.

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Box ID               | UINT 8               | Id of iNav mode      |
| Aux Channel Index    | UINT 8               | RC Channel input     |
| Start Step           | UINT 8               | Range start. 900 + |  * 25 |
| End Step             | UINT 8               | Range end. 900 + |  * 25 |

### 35 - MSP_SET_MODE_RANGE

Set one mode range at the time.

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Mode Range Index     | UINT 8               | Id of range          |
| Box ID               | UINT 8               | Id of iNav mode      |
| Aux Channel Index    | UINT 8               | RC Channel input     |
| Start Step           | UINT 8               | Range start. 900 + |  * 25 |
| End Step             | UINT 8               | Range end. 900 + |  * 25 |

### 36 - MSP_FEATURE

TODO: Not used// Figure out how it works... mask?

### 37 - MSP_SET_FEATURE

TODO: Not used

### 38 - MSP_BOARD_ALIGNMENT

TODO: Not used// Figure out how it works... mask?
|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Roll                 | UINT 16              |                      |
| Pitch                | UINT 16              |                      |
| Yaw                  | UINT 16              |                      |

### 39 - MSP_SET_BOARD_ALIGNMENT

See: [MSP_BOARD_ALIGNMENT](###-38---MSP_BOARD_ALIGNMENT)

### 40 - MSP_CURRENT_METER_CONFIG

TODO: Not really used?

### 41 - MSP_SET_CURRENT_METER_CONFIG

TODO: Not really used?

### 42 - MSP_MIXER //TODO not used
### 43 - MSP_SET_MIXER // TODO not used
### 44 - MSP_RX_CONFIG

|         Data                | Data Type |       Comments       |
| --------------------------- | :-------: | -------------------- |
| Serial RX Provider          | UINT 8    |                      |
| Max Check                   | UINT 16   |                      |
| Mid Rc                      | UINT 16   |                      |
| Min Check                   | UINT 16   |                      |
| Spektrum Sat Bind           | UINT 8    |                      |
| Rx Min USec                 | UINT 16   |                      |
| Rx Max USec                 | UINT 16   |                      |
| RC Interpolation            | UINT 8    | Betaflight only      |
| RC Interpolation Interval   | UINT 8    | Betaflight only      |
| Air Mode Activate Threshold | UINT 16   | Betaflight only      |
| RX SPI Protocol             | UINT 8    |                      |
| RX SPI Id                   | UINT 32   |                      |
| RX SPI Channel Count        | UINT 8    |                      |
| FPV Camera Angle Degrees    | UINT 8    | Betaflight only      |
| Receiver Type               | UINT 8    | Inav only            |


### 45 - MSP_SET_RX_CONFIG
### 46 - MSP_LED_COLORS
### 47 - MSP_SET_LED_COLORS
### 48 - MSP_LED_STRIP_CONFIG
### 49 - MSP_SET_LED_STRIP_CONFIG
### 50 - MSP_RSSI_CONFIG
### 51 - MSP_SET_RSSI_CONFIG
### 52 - MSP_ADJUSTMENT_RANGES
### 53 - MSP_SET_ADJUSTMENT_RANGE
### 54 - MSP_CF_SERIAL_CONFIG
### 55 - MSP_SET_CF_SERIAL_CONFIG
### 56 - MSP_VOLTAGE_METER_CONFIG
### 57 - MSP_SET_VOLTAGE_METER_CONFIG
### 58 - MSP_SONAR_ALTITUDE
### 59 - MSP_PID_CONTROLLER
### 60 - MSP_SET_PID_CONTROLLER
### 61 - MSP_ARMING_CONFIG
### 62 - MSP_SET_ARMING_CONFIG
### 64 - MSP_RX_MAP
### 65 - MSP_SET_RX_MAP
### 66 - MSP_BF_CONFIG

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Mixer Configuration  | UINT 8               |                      |
| Features             | UINT 32              |                      |
| Serial Rx Type       | UINT 8               |                      |
| Board Align Roll     | UINT 16              | -180 - 360           |
| Board Align Pitch    | UINT 16              | -180 - 360           |
| Board Align Yaw      | UINT 16              | -180 - 360           |
| Current Scale        | UINT 16              |                      |
| Current Offset       | UINT 16              |                      |

### 67 - MSP_SET_BF_CONFIG

See: [MSP_SET_BF_CONFIG](###-67---MSP_SET_BF_CONFIG)

### 68 - MSP_REBOOT
### 69 - MSP_BF_BUILD_INFO
### 70 - MSP_DATAFLASH_SUMMARY
### 71 - MSP_DATAFLASH_READ
### 72 - MSP_DATAFLASH_ERASE
### 73 - MSP_LOOP_TIME
### 74 - MSP_SET_LOOP_TIME
### 75 - MSP_FAILSAFE_CONFIG
### 76 - MSP_SET_FAILSAFE_CONFIG
### 77 - MSP_RXFAIL_CONFIG
### 78 - MSP_SET_RXFAIL_CONFIG
### 79 - MSP_SDCARD_SUMMARY
### 80 - MSP_BLACKBOX_CONFIG
### 81 - MSP_SET_BLACKBOX_CONFIG
### 82 - MSP_TRANSPONDER_CONFIG
### 83 - MSP_SET_TRANSPONDER_CONFIG
### 84 - MSP_OSD_CONFIG
### 85 - MSP_SET_OSD_CONFIG
### 86 - MSP_OSD_CHAR_READ
### 87 - MSP_OSD_CHAR_WRITE
### 88 - MSP_VTX_CONFIG
### 89 - MSP_SET_VTX_CONFIG
### 90 - MSP_ADVANCED_CONFIG
### 91 - MSP_SET_ADVANCED_CONFIG
### 92 - MSP_FILTER_CONFIG
### 93 - MSP_SET_FILTER_CONFIG
### 94 - MSP_PID_ADVANCED
### 95 - MSP_SET_PID_ADVANCED
### 96 - MSP_SENSOR_CONFIG

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| Accelerometer        | UINT 8               |                      |
| Barometer            | UINT 8               |                      |
| Magnetometer         | UINT 8               |                      |
| Pitot                | UINT 8               |                      |
| Rangefinder          | UINT 8               |                      |
| Optical Flow Sensor  | UINT 8               |                      |

### 97 - MSP_SET_SENSOR_CONFIG

See: [MSP_SET_SENSOR_CONFIG](###-96---MSP_SENSOR_CONFIG)

### 98 - MSP_SPECIAL_PARAMETERS

Not used

### 99 - MSP_SET_SPECIAL_PARAMETERS

Not used

### 100 - MSP_IDENT

-- Depricated


### 101 - MSP_STATUS

-- Depricated (Still allowed but not recommended)

### 102 - MSP_RAW_IMU
### 103 - MSP_SERVO
### 104 - MSP_MOTOR
### 105 - MSP_RC
### 106 - MSP_RAW_GPS
### 107 - MSP_COMP_GPS
### 108 - MSP_ATTITUDE
### 109 - MSP_ALTITUDE
### 110 - MSP_ANALOG
### 111 - MSP_RC_TUNING
### 112 - MSP_PID
### 113 - MSP_ACTIVEBOXES
### 114 - MSP_MISC
### 115 - MSP_MOTOR_PINS
### 116 - MSP_BOXNAMES
### 117 - MSP_PIDNAMES
### 118 - MSP_WP
### 119 - MSP_BOXIDS
### 120 - MSP_SERVO_CONFIGURATIONS
### 121 - MSP_NAV_STATUS
### 122 - MSP_NAV_CONFIG
### 124 - MSP_3D
### 125 - MSP_RC_DEADBAND
### 126 - MSP_SENSOR_ALIGNMENT
### 127 - MSP_LED_STRIP_MODECOLOR

### 150 - MSP_STATUS_EX

### 151 - MSP_SENSOR_STATUS


### 160 - MSP_UID
### 164 - MSP_GPSSVINFO
### 166 - MSP_GPSSTATISTICS
### 180 - MSP_OSD_VIDEO_CONFIG
### 181 - MSP_SET_OSD_VIDEO_CONFIG
### 182 - MSP_DISPLAYPORT
### 186 - MSP_SET_TX_INFO
### 187 - MSP_TX_INFO
### 200 - MSP_SET_RAW_RC
### 201 - MSP_SET_RAW_GPS
### 202 - MSP_SET_PID
### 203 - MSP_SET_BOX
### 204 - MSP_SET_RC_TUNING
### 205 - MSP_ACC_CALIBRATION
### 206 - MSP_MAG_CALIBRATION
### 207 - MSP_SET_MISC
### 208 - MSP_RESET_CONF
### 209 - MSP_SET_WP
### 210 - MSP_SELECT_SETTING
### 211 - MSP_SET_HEAD
### 212 - MSP_SET_SERVO_CONFIGURATION
### 214 - MSP_SET_MOTOR
### 215 - MSP_SET_NAV_CONFIG
### 217 - MSP_SET_3D
### 218 - MSP_SET_RC_DEADBAND
### 219 - MSP_SET_RESET_CURR_PID
### 220 - MSP_SET_SENSOR_ALIGNMENT
### 221 - MSP_SET_LED_STRIP_MODECOLOR
### 239 - MSP_SET_ACC_TRIM
### 240 - MSP_BIND
### 240 - MSP_ACC_TRIM
### 241 - MSP_SERVO_MIX_RULES
### 242 - MSP_ALARMS
### 242 - MSP_SET_SERVO_MIX_RULE
### 245 - MSP_SET_PASSTHROUGH
### 246 - MSP_RTC
### 247 - MSP_SET_RTC
### 250 - MSP_EEPROM_WRITE
### 251 - MSP_RESERVE_1
### 252 - MSP_RESERVE_2
### 253 - MSP_DEBUGMSG
### 254 - MSP_DEBUG
### 255 - MSP_V2_FRAME
### 0x1001 - MSP2_COMMON_TZ
Not found
### 0x1002 - MSP2_COMMON_SET_TZ
### 0x1003 - MSP2_COMMON_SETTING
### 0x1004 - MSP2_COMMON_SET_SETTING
### 0x1005 - MSP2_COMMON_MOTOR_MIXER
### 0x1006 - MSP2_COMMON_SET_MOTOR_MIXER
### 0x1007 - MSP2_COMMON_SETTING_INFO
### 0x1008 - MSP2_COMMON_PG_LIST
### 0x1009 - MSP2_COMMON_SERIAL_CONFIG
### 0x100A - MSP2_COMMON_SET_SERIAL_CONFIG
### 0x100B - MSP2_COMMON_SET_RADAR_POS
### 0x100C - MSP2_COMMON_SET_RADAR_ITD
### 0x1F01 - MSP2_SENSOR_RANGEFINDER
### 0x1F02 - MSP2_SENSOR_OPTIC_FLOW
### 0x1F03 - MSP2_SENSOR_GPS
### 0x1F04 - MSP2_SENSOR_COMPASS
### 0x1F05 - MSP2_SENSOR_BAROMETER
### 0x1F06 - MSP2_SENSOR_AIRSPEED
### 0x2000 - MSP2_INAV_STATUS

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| cycleTime            | UINT 16              |                      |
| offset               | UINT 16              |                      |
| i2cError             | UINT 16              |                      |
| offset               | UINT 16              |                      |
| activeSensors        | UINT 16              |                      |
| offset               | UINT 16              |                      |
| cpuload              | UINT 16              |                      |
| offset               | UINT 16              |                      |
| profile_byte         | UINT 8               |                      |
| armingFlags          | UINT 32              |                      |
| offset               | UINT 32              |                      |

| profile = profile_byte & 0x0F
| battery_profile = (profile_byte & 0xF0) >> 4

### 0x2001 - MSP2_INAV_OPTICAL_FLOW

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| raw_quality          | UINT 8               |                      |
| flow_rate_x          | UINT 16              |                      |
| flow_rate_y          | UINT 16              |                      |
| body_rate_x          | UINT 16              |                      |
| body_rate_y          | UINT 16              |                      |

### 0x2002 - MSP2_INAV_ANALOG

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| battery_voltage      | UINT 8               |                      |
| mAh_drawn            | UINT 16              |                      |
| rssi                 | UINT 16              |                      |
| amperage             | UINT 16              |                      |

### 0x2003 - MSP2_INAV_MISC

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| mid_rc               | UINT 16              |                      |
| min_throttle         | UINT 16              |                      |
| max_throttle         | UINT 16              |                      |
| min_command          | UINT 16              |                      |
| failsafe_throttle    | UINT 16              |                      |
| gps_provider         | UINT 8               |                      |
| gps_baudrate         | UINT 8               |                      |
| gps_ubx_sbas         | UINT 8               |                      |
| rssi_channel         | UINT 8               |                      |
| mag_declination      | UINT 16              |                      |
| voltage_scale        | UINT 16              |                      |
| cell_min             | UINT 16              |                      |
| cell_max             | UINT 16              |                      |
| cell_warning         | UINT 16              |                      |
| capacity             | UINT 32              |                      |
| capacity_warning     | UINT 32              |                      |
| capacity_critical    | UINT 32              |                      |
| capacity_units       | UINT 8               |                      |

### 0x2004 - MSP2_INAV_SET_MISC
### 0x2005 - MSP2_INAV_BATTERY_CONFIG

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
|voltage_scale         | UINT 16              |                      |
|cell_min              | UINT 16              |                      |
|cell_max              | UINT 16              |                      |
|cell_warning          | UINT 16              |                      |
|current_offset        | UINT 16              |                      |
|current_scale         | UINT 16              |                      |
|capacity              | UINT 32              |                      |
|capacity_warning      | UINT 32              |                      |
|capacity_critical     | UINT 32              |                      |
|capacity_units        | UINT 8               |                      |

### 0x2006 - MSP2_INAV_SET_BATTERY_CONFIG

### 0x2007 - MSP2_INAV_RATE_PROFILE

|          Data          |       Data Type      |       Comments       |
| ---------------------- | :------------------: | -------------------- |
| throttle_rc_mid        | UINT 8               |                      |
| throttle_rc_expo       | UINT 8               |                      |
| throttle_dyn_pid       | UINT 8               |                      |
| throttle_pa_breakpoint | UINT 16              |                      |
| stabilized_rc_expo     | UINT 8               |                      |
| stabilized_rc_yaw_expo | UINT 8               |                      |
| stabilized_rate_r      | UINT 8               |                      |
| stabilized_rate_p      | UINT 8               |                      |
| stabilized_rate_y      | UINT 8               |                      |
| manual_rc_expo         | UINT 8               |                      |
| manual_rc_yaw_expo     | UINT 8               |                      |
| manual_rate_r          | UINT 8               |                      |
| manual_rate_p          | UINT 8               |                      |
| manual_rate_y          | UINT 8               |                      |

### 0x2008 - MSP2_INAV_SET_RATE_PROFILE
### 0x2009 - MSP2_INAV_AIR_SPEED

|         Data         |       Data Type      |       Comments       |
| -------------------- | :------------------: | -------------------- |
| airspeed             | UINT 32              |                      |

### 0x200A - MSP2_INAV_OUTPUT_MAPPING
### 0x200B - MSP2_INAV_MC_BRAKING
### 0x200C - MSP2_INAV_SET_MC_BRAKING
### 0x2010 - MSP2_INAV_MIXER
### 0x2011 - MSP2_INAV_SET_MIXER
### 0x2012 - MSP2_INAV_OSD_LAYOUTS
### 0x2013 - MSP2_INAV_OSD_SET_LAYOUT_ITEM
### 0x2014 - MSP2_INAV_OSD_ALARMS
### 0x2015 - MSP2_INAV_OSD_SET_ALARMS
### 0x2016 - MSP2_INAV_OSD_PREFERENCES
### 0x2017 - MSP2_INAV_OSD_SET_PREFERENCES
### 0x2018 - MSP2_INAV_SELECT_BATTERY_PROFILE
### 0x2019 - MSP2_INAV_DEBUG
### 0x201A - MSP2_BLACKBOX_CONFIG
### 0x201B - MSP2_SET_BLACKBOX_CONFIG
### 0x201C - MSP2_INAV_TEMP_SENSOR_CONFIG
### 0x201D - MSP2_INAV_SET_TEMP_SENSOR_CONFIG
### 0x201E - MSP2_INAV_TEMPERATURES
### 0x2020 - MSP2_INAV_SERVO_MIXER
### 0x2021 - MSP2_INAV_SET_SERVO_MIXER
### 0x2022 - MSP2_INAV_LOGIC_CONDITIONS
### 0x2023 - MSP2_INAV_SET_LOGIC_CONDITIONS
### 0x2024 - MSP2_INAV_GLOBAL_FUNCTIONS
### 0x2025 - MSP2_INAV_SET_GLOBAL_FUNCTIONS
### 0x2026 - MSP2_INAV_LOGIC_CONDITIONS_STATUS
### 0x2027 - MSP2_INAV_GVAR_STATUS
### 0x2030 - MSP2_PID
### 0x2031 - MSP2_SET_PID
### 0x2032 - MSP2_INAV_OPFLOW_CALIBRATION
### 0x2033 - MSP2_INAV_FWUPDT_PREPARE
### 0x2034 - MSP2_INAV_FWUPDT_STORE
### 0x2035 - MSP2_INAV_FWUPDT_EXEC
### 0x2036 - MSP2_INAV_FWUPDT_ROLLBACK_PREPARE
### 0x2037 - MSP2_INAV_FWUPDT_ROLLBACK_EXEC
