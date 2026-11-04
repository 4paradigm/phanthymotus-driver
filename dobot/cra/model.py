"""Official V4 CR3 chain for the controller-identified CR3A (type 113)."""

from pathlib import Path


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
MODEL_COMMIT = "ec201c40d5db63185a3593ec9c39f80fe25aa527"


def resolve_model(snapshot, configured_name=""):
    code = snapshot.get("robot_type_code")
    configured = str(configured_name).strip().upper()
    if code == 113:
        if configured and configured != "CR3A":
            raise ValueError("configured model disagrees with feedback type 113 (CR3A)")
        return {"model_name": "CR3A", "model_source": "feedback_type_113"}
    if code is not None:
        raise ValueError(f"no verified skeleton model bundled for robot type {code}")
    if configured == "CR3A":
        return {"model_name": "CR3A", "model_source": "operator_config"}
    raise ValueError("CR3A identification requires feedback or explicit model_name=CR3A")


def model_resource(snapshot, configured_name=""):
    try:
        identity = resolve_model(snapshot, configured_name)
    except ValueError as exc:
        # The platform falls back to a G1 humanoid when the model call errors
        # or omits URDF. An empty, explicitly unavailable scene is preferable to
        # presenting a different robot as the connected arm.
        return {
            "available": False, "model_error": str(exc), "joint_names": [],
            "urdf": '<robot name="dobot_model_unavailable"><link name="base_link"/></robot>',
        }
    return {
        **identity, "available": True,
        "urdf": Path(__file__).with_name("resource").joinpath("cr3_robot.urdf").read_text(encoding="utf-8"),
        "joint_names": list(JOINT_NAMES),
        "model_commit": MODEL_COMMIT,
        "model_repository": "https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4",
    }
