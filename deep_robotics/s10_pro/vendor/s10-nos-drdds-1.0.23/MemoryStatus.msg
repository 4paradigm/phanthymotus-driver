# Memory status monitoring

builtin_interfaces/Time timestamp      # Data acquisition time
string soc_id                          # SoC identifier (e.g., "CPU_103", "CPU_104")

# Memory information (from /proc/meminfo)
uint32 mem_total_kb                    # Total physical memory in KB (MemTotal)
uint32 mem_available_kb                # Available memory in KB (MemAvailable)
uint32 mem_buffers_kb                  # Buffer memory in KB (Buffers)
uint32 mem_cached_kb                   # Cache memory in KB (Cached)

# Swap information (from /proc/meminfo)
uint32 swap_total_kb                   # Total swap space in KB (SwapTotal)
uint32 swap_free_kb                    # Free swap space in KB (SwapFree)

