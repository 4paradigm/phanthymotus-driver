"""
R1 arm service constants
"""
ARM_SERVICE_NAME = "arm"
ARM_API_VERSION = "1.0.0.0"

ROBOT_API_ID_ARM_SET_TASK = 7106       # Execute action by id
ROBOT_API_ID_ARM_LIST_ACTIONS = 7107   # List available actions
ROBOT_API_ID_ARM_ENABLE = 7108         # Enable arm SDK control
ROBOT_API_ID_ARM_RENAME = 7109         # Rename action (pre_name, new_name)
ROBOT_API_ID_ARM_RELEASE = 7110        # Release arm SDK control
ROBOT_API_ID_ARM_GET_STATUS = 7111     # Get arm status
ROBOT_API_ID_ARM_EXECUTE_NAME = 7112   # Execute action by name
ROBOT_API_ID_ARM_STOP = 7113           # Stop current action
