"""OS-process entry point for the high-volume AS2 lidar sensor."""


def _setup_logs():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def run_lidar(topic, source_topics, max_render_points, interface):
    """Own Unitree DDS, ROS publication, and conversion in a child process."""
    # Spawned processes do not inherit the main process's DDS participant.
    # Require its already-resolved adapter instead of SDK auto-selection.
    if not isinstance(interface, str) or not interface.strip() or interface.strip().lower() == "auto":
        raise ValueError("Lidar requires an explicitly resolved robot interface")
    interface = interface.strip()
    _setup_logs()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from lidar import _LidarNode

    try:
        ChannelFactoryInitialize(0, interface)
    except Exception as exc:
        print(
            f"[as2w-lidar-process] Unitree DDS init failed: {str(exc)[:180]}",
            flush=True,
        )
        raise RuntimeError("Lidar robot-interface DDS initialization failed") from exc
    rclpy.init(args=None)
    executor = SingleThreadedExecutor()
    node = _LidarNode(topic, executor, source_topics, max_render_points)
    try:
        executor.spin()
    finally:
        node.close()
        rclpy.shutdown()
