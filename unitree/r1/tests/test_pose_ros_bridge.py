"""ROS-free contract test for the Tianyi camera-to-Agent-Core boundary."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_ros_service import _create_domain_nodes, event_payloads


class PoseRosBridgeTests(unittest.TestCase):
    def test_only_events_create_messages_with_session_progress(self):
        session = {"session_id": "exercise-1", "state": "running",
                   "progress": {"repetitions": 2, "target_repetitions": 5,
                                "elapsed_seconds": 12.4, "calibrated": True,
                                "phase": "ready"}}
        self.assertEqual(event_payloads([], 20, session), [])
        event = {"event": "rep_completed", "pose": "squat", "count": 2,
                 "target": 5, "narration": "第 2 次深蹲完成"}
        messages = event_payloads([event], 21, session)
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0]["schema"], "pose_check.event.v1")
        self.assertEqual(messages[0]["session_id"], "exercise-1")
        self.assertEqual(messages[0]["progress"]["repetitions"], 2)
        self.assertEqual(messages[0]["events"], [event])

    def test_final_rep_and_task_completion_are_separate_messages(self):
        session = {"session_id": "exercise-1", "state": "completed",
                   "progress": {"repetitions": 5, "target_repetitions": 5,
                                "elapsed_seconds": 30.0}}
        messages = event_payloads([
            {"event": "rep_completed", "pose": "squat", "count": 5},
            {"event": "step_completed", "pose": "squat", "count": 5,
             "task_completed": True},
        ], 30, session)
        self.assertEqual([message["event"] for message in messages],
                         ["rep_completed", "step_completed"])
        self.assertTrue(messages[-1]["task_completed"])

    def test_camera_and_result_use_different_domains(self):
        class Context:
            pass

        class Node:
            def __init__(self, name, context):
                self.name = name
                self.context = context

        class Ros:
            def __init__(self):
                self.domains = {}

            def init(self, *, context, domain_id):
                self.domains[context] = domain_id

            def create_node(self, name, *, context):
                return Node(name, context)

        class Executor:
            def __init__(self, *, context):
                self.context = context
                self.nodes = []

            def add_node(self, node):
                self.nodes.append(node)

        ros = Ros()
        camera_ctx, core_ctx, camera_node, core_node, camera_executor, core_executor = (
            _create_domain_nodes(ros, Context, Executor))
        self.assertEqual(ros.domains[camera_ctx], 0)
        self.assertEqual(ros.domains[core_ctx], 42)
        self.assertIs(camera_node.context, camera_ctx)
        self.assertIs(core_node.context, core_ctx)
        self.assertEqual(camera_executor.nodes, [camera_node])
        self.assertEqual(core_executor.nodes, [core_node])


if __name__ == "__main__":
    unittest.main()
