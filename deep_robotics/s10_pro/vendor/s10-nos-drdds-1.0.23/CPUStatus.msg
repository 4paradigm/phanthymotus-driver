# Raw monitoring data for a single CPU/SoC

builtin_interfaces/Time timestamp   # Data acquisition time
string soc_id                       # SoC identifier (e.g., "CPU_103", "CPU_104")
int32 package_temp                  # Package temperature in °C 
int32[] temps                       # Per-zone temperatures in °C 
uint16 avg_util                     # Average utilization across all logical CPUs [0–100], 1%
uint16[] util                       # Utilization per logical CPU [0–100], 1%
uint32[] cur_freq_khz               # Current target frequency (scaling_cur_freq)
uint32[] hw_max_freq_khz            # Hardware max frequency (cpuinfo_max_freq)
uint32[] hw_min_freq_khz            # Hardware min frequency (cpuinfo_min_freq)
string[] gov_policy                 # scaling governor: performance/powersave
