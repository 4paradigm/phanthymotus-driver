"""Exercise real ROS action lifecycle against a private fake Nav2 server.
Run with ROS_DOMAIN_ID=91. No velocity publisher or physical goal is created.
"""
import json
import os
import tempfile
import threading
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '91':
        raise RuntimeError('This probe requires the isolated ROS_DOMAIN_ID=91')
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from nav2_msgs.action import NavigateToPose
    from controlled_spatial import ControlledSpatialPlugin
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=6)
    server = Node('panther_fake_nav2_validation')
    executor.add_node(server)
    def execute(handle):
        mode = round(handle.request.pose.pose.position.x)
        if mode == 1:
            deadline = time.monotonic()+5
            while time.monotonic() < deadline and not handle.is_cancel_requested:
                time.sleep(0.01)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        elif mode == 2:
            handle.abort()
        else:
            time.sleep(0.1)
            handle.succeed()
        return NavigateToPose.Result()
    action = ActionServer(server, NavigateToPose, '/panther_validation/navigate_to_pose', execute,
        goal_callback=lambda goal: GoalResponse.REJECT if round(goal.pose.pose.position.x) == 3 else GoalResponse.ACCEPT,
        cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
    with tempfile.TemporaryDirectory() as tmp:
        plugin = ControlledSpatialPlugin({'native_slam_pcd_dir': tmp+'/maps',
                'native_slam_db_path': tmp+'/db', 'navigation_action': '/panther_validation/navigate_to_pose',
                'navigation_timeout_s': 10}, 'test', executor)
        notifications = []
        plugin._notify = lambda *args: notifications.append(args)
        plugin.start()
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        results = {}
        try:
            time.sleep(1)
            for case, x in [('succeeded', 0), ('aborted', 2), ('rejected', 3), ('cancelled', 1), ('timeout', 1)]:
                plugin.config['navigation_timeout_s'] = 0.4 if case == 'timeout' else 10
                accepted = plugin._navigate({'x': float(x), 'y': 0.0, 'yaw': 0.0})
                if case == 'cancelled':
                    deadline = time.monotonic()+3
                    while plugin.nav and plugin.nav['handle'] is None and time.monotonic()<deadline:
                        time.sleep(0.01)
                    plugin.stop_nav()
                deadline = time.monotonic()+6
                while plugin.nav and time.monotonic()<deadline:
                    time.sleep(0.02)
                assert plugin.nav is None, (case, plugin.info())
                expected = {'succeeded': 'completed', 'cancelled': 'cancelled'}.get(case, 'error')
                result = plugin.last_navigation
                assert result['status'] == expected, (case, result)
                assert result['action_id'] == accepted['action_id'], (case, result)
                results[case] = result
            time.sleep(0.2)
            assert len(notifications) == 5, notifications
            print(json.dumps({'domain': 91, 'action': '/panther_validation/navigate_to_pose',
                'cases': results, 'completion_callbacks': len(notifications)}, indent=2), flush=True)
        finally:
            plugin.stop()
            action.destroy()
            executor.remove_node(server)
            server.destroy_node()
            executor.shutdown()
            thread.join(timeout=3)
            rclpy.shutdown()


if __name__ == '__main__':
    main()
