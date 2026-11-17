# Sourced only by the right-arm vendor follow script.
python3() {
    if [ "$#" -gt 0 ] &&
       [ "$(realpath -- "$1")" = "$FR3_VENDOR_RUN_ENV" ]; then
        shift
        command python3 "$FR3_VENDOR_RUN_ENV_COPY" "$@"
    else
        command python3 "$@"
    fi
}

launch_robot.py() {
    command launch_robot.py "$@" "robot_client.executable_cfg.robot_ip=$FR3_ROBOT_IP"
}
launch_gripper.py() {
    command launch_gripper.py "$@" "gripper.executable_cfg.robot_ip=$FR3_ROBOT_IP"
}
