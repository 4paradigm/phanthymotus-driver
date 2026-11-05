uint8 type  # effect type

# [color] color
# For gradient flow effect: color[0] is the start color, color[1] is the end color.
# For dual-color alternating flash effect: color[0] is color 1, color[1] is color 2.
uint8[] color

uint8 num     # LED count
bool dir      # direction: false=forward, true=reverse
uint8 cycle   # cycle period
