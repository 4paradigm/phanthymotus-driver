# NetworkStatus.msg - Enhanced for robust anomaly detection

builtin_interfaces/Time timestamp   # Data acquisition time
string soc_id                       # SoC identifier (e.g., "CPU_103", "CPU_104")
string iface                        # Network interface name (e.g., "eth0", "wlan0")

# Operational state
bool is_up                          # Link up (operstate)

# THROUGHPUT (bytes/sec, exact)
uint64 tx_bytes_per_sec     # e.g., 12500000 = 100 Mbps
uint64 rx_bytes_per_sec

# Cumulative counters since boot/if-up
uint64 tx_packets           # Total transmitted packets
uint64 rx_packets           # Total received packets
uint64 tx_dropped           # TX packets dropped (e.g., queue full)
uint64 rx_dropped           # RX packets dropped (e.g., buffer full)
uint64 tx_errors            # TX errors (hardware issues)
uint64 rx_errors            # RX errors (hardware issues)
uint64 rx_crc_errors        # CRC errors (physical layer health)
uint64 rx_missed_errors     # RX packets missed (NIC ring buffer full)
