# FileSystemStatus.msg
builtin_interfaces/Time timestamp

string soc_id                 # SoC identifier (e.g., "CPU_103", "CPU_104")
string mount_point            # Mount point path (e.g., "/root-ro", "/boot")
string device_path            # e.g., "/dev/mmcblk0p12"
string fs_type                # "ext4", "f2fs", etc.

# Space usage
uint64 total_bytes
uint64 used_bytes
uint64 available_bytes

# Inode usage (prevents "No space left on device" despite free space)
uint64 used_inodes
uint64 free_inodes

bool is_readonly
