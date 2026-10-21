"""ROS-independent canvas contract shared by Bumi and Q5 fitness cards."""

from __future__ import annotations


EXERCISES = ("squat", "dumbbell_curl", "lateral_raise")


def topic_out(namespace: str, *, local_milestone_voice: bool = False) -> list[dict]:
    prefix = f"/{namespace}/fitness/squat"
    ports = [
        {"topic": f"{prefix}/events", "format": "data/json",
         "message_type": "std_msgs/msg/String",
         "desc": "Sparse session, target, and rep events for Decision Core"},
        {"topic": f"{prefix}/status", "format": "data/json",
         "message_type": "std_msgs/msg/String",
         "desc": "Current count for a dashboard; do not connect to Decision Core"},
        {"topic": f"{prefix}/debug", "format": "data/json",
         "message_type": "std_msgs/msg/String",
         "desc": "供 Dashboard 查看中文姿态测量与阈值判定"},
    ]
    if local_milestone_voice:
        ports.append({
            "topic": f"{prefix}/voice", "format": "data/json",
            "message_type": "std_msgs/msg/String",
            "desc": "Short, expiring coach prompts; connect directly to perception TTS",
        })
    return ports


def tool_definition(namespace: str, *, coach_mode: bool = False,
                    local_milestone_voice: bool = False) -> dict:
    begin_description = (
        "When the user asks to start training, choose the exercise and plan, then call "
        "this action before counting. Use 2 sets of 8 reps with 90 seconds "
        "of rest unless the user specifies another valid plan. A repeated "
        "call for the same exercise keeps the active session and count. "
        "Finish an active workout before changing exercises. Read the returned "
        "plan before announcing it with TTS"
        if coach_mode else "Start a workout; repeated calls keep the active session"
    )
    return {
        "name": "squat_counter",
        "type": "processor",
        "multiInstance": False,
        "description": (
            "Count one selected person's squats, dumbbell curls, or lateral "
            "raises from the full pose "
            "skeleton stream. Use status for the current total; connect only "
            "the sparse events output to Decision Core. Status and debug "
            "outputs are for the dashboard."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": [
                    "start", "stop", "info", "status", "debug", "reset",
                    "begin", "next_set", "finish"]},
                "exercise": {"type": "string", "enum": list(EXERCISES),
                             "description": "Exercise to begin or reset; defaults to the current exercise (initially squat)"},
                "sets": {"type": "integer", "minimum": 1, "maximum": 5,
                         "description": "Number of planned sets in coach mode (default: 2)"},
                "reps_per_set": {"type": "integer", "minimum": 1, "maximum": 20,
                                 "description": "Complete repetitions required per set in coach mode (default: 8)"},
                "rest_s": {"type": "integer", "minimum": 15, "maximum": 180,
                           "description": "Rest seconds between sets in coach mode (default: 90)"},
                "input_topic": {
                    "type": "string",
                    "description": "Full pose /poses/skeleton ROS topic from the canvas connection",
                },
            },
            "required": ["action"],
            "x-action-params": {
                "start": {"params": ["input_topic"],
                          "description": "Subscribe to skeleton frames and start a new count"},
                "stop": {"params": [], "description": "Stop subscribing; retain final total"},
                "info": {"params": [], "description": "Show card ports and status"},
                "status": {"params": [], "description": "Read the authoritative current total"},
                "debug": {"params": [], "description": "查看中文姿态测量与计数判定"},
                "reset": {"params": ["exercise"], "description": "Clear the count and start a new session, optionally selecting an exercise"},
                "begin": {"params": ["exercise", "sets", "reps_per_set", "rest_s"],
                          "description": begin_description},
                "next_set": {"params": [], "description":
                             "Start the next set after the rest timer has finished"},
                "finish": {"params": [], "description":
                           "Freeze the workout and return its final summary"},
            },
        },
        "topic_in": [{
            "format": "sensor/pose2d",
            "message_type": "std_msgs/msg/String",
            "desc": "Full pose /poses/skeleton JSON with COCO keypoints",
        }],
        "topic_out": topic_out(namespace,
                               local_milestone_voice=local_milestone_voice),
    }
