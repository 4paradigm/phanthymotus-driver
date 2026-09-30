"""OS-process entry point for the high-volume AS2 lidar sensor."""


def _setup_logs():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def run_lidar(topic, source_topics, max_render_points, interface):
    """Own Unitree DDS, ROS publication, and conversion in a child process."""
    _setup_logs()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from lidar import _LidarNode

    try:
        ChannelFactoryInitialize(0, interface or None)
    except Exception as exc:
        print(
            f"[as2w-lidar-process] Unitree DDS init failed: {str(exc)[:180]}",
            flush=True,
        )
    rclpy.init(args=None)
    executor = SingleThreadedExecutor()
    node = _LidarNode(topic, executor, source_topics, max_render_points)
    try:
        executor.spin()
    finally:
        node.close()
        rclpy.shutdown()
