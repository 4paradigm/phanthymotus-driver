uint16 voltage             # total voltage, unit: 10 mV
int16  current             # total current, unit: 10 mA
uint16 remaining_capacity  # remaining capacity, unit: 10 mAh
uint16 nominal_capacity    # nominal capacity, unit: 10 mAh
uint16 cycles              # cycle count
uint16 production_date     # production date
uint16 balanced_low        # balance low
uint16 balanced_high       # balance high
uint16 protected_state     # protection state

uint8 software_version     # software version
uint8 battery_level        # remaining capacity percentage
uint8 mos_state            # MOS state
uint8 battery_quantity     # battery cell count
uint8 battery_ntc          # NTC count
float32[] battery_temperature  # battery temperature list
char[32] battery_serialnum # battery serial number
