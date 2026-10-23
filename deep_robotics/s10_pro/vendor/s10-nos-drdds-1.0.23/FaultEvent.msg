# Fault event message for system faults
# Fault Code Categories (Revised)
# 0x80xx: Joints (motors, drivers, encoders, IMU, etc.)
# 0x81xx: Battery (cells, BMS, voltage, current, temperature, etc.)
# 0x82xx: Mainboard (CPU, memory, storage, network, system time, etc.)
# 0x83xx: Peripherals (LiDAR, cameras, buttons, RTK, handheld devices, etc.)
# 0x84xx: Motion control (manual control & autonomous control)
# 0x85xx: Charging (autonomous and manual charging scenarios)
# 0x86xx: System monitoring (heartbeats, process liveness, module responsiveness, etc.)

# define event type Constant
uint8 TYPE_START  = 1
uint8 TYPE_STOP   = 2

# define Severity level Constant (Based on Syslog RFC 5424)
# 0-7: Standard severity levels, lower value = higher priority in some contexts, 
# but here we treat higher value = higher severity for application logic.
uint8 SEVERITY_DEBUG    = 0
uint8 SEVERITY_INFO     = 1
uint8 SEVERITY_NOTICE   = 2
uint8 SEVERITY_WARN     = 3
uint8 SEVERITY_ERROR    = 4
uint8 SEVERITY_FATAL    = 5	  #兼容性保留, = SEVERITY_CRITICAL
uint8 SEVERITY_CRITICAL = 5
uint8 SEVERITY_ALERT    = 6
uint8 SEVERITY_EMERG    = 7

# Aggregate logical uniqueness identifier = code + resources + source;
builtin_interfaces/Time timestamp  # Timestamp of fault occurrence
string name                        # Fault name; unique across types, identical for same-type faults
uint32 code                        # Fault code; unique across types, identical for same-type faults
uint8 type                         # Event type: start/stop (severity changes use START)
uint8[] severities                 # Severity levels: array for aggregated instances (warn/error/fatal)
string[] resources                 # Resource array: Indexable Resources(start from 0); Entity Resources(Resource name)
bool grouped                       # true, Multi-resource group tracking; false: single resource independent tracking;
string[] source                    # The name of the process that triggered this fault event, used to identify the reporting entity, only one element needs to be filled in;
string details                     # Additional data in compact JSON format
