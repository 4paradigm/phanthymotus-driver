"""Command envelope verified against the commissioned vendor control chain.

kuavo_s63/task.info: ruckig wheel_move.max_vel = [0.8, 0.8, 1.6].
leju_node.launch and live ROS params: max_linear_vel=1.0, max_angular_vel=1.0.
Use their intersection, not the unrelated keyboard/route limits or a claimed
hardware maximum. Other firmware revisions may impose lower downstream caps.
"""
LINEAR_LIMIT = .8
ANGULAR_LIMIT = 1.0
SPEED_FRACTIONS = {'low': .3, 'medium': .6, 'high': 1.}
