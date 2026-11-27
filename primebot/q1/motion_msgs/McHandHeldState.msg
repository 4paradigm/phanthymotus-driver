# Robot motion-control hand-held state.

# The state is unknown when either hand has not produced a confirmed result.
uint8 UNKNOWN = 0

# Both hands are confirmed as not held.
uint8 BOTH_NOT_HELD = 1

# Only the left hand is held.
uint8 LEFT_HELD_ONLY = 2

# Only the right hand is held.
uint8 RIGHT_HELD_ONLY = 3

# Both hands are held.
uint8 BOTH_HELD = 4

# True when at least one hand is confirmed as held.
# False when neither hand is held or the combined state is unknown.
bool is_held

# Current combined hand-held state. The value must be one of the constants above.
uint8 hand_held
