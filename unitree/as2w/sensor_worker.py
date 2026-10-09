"""OS-process entry point for the high-volume AS2 lidar sensor."""


def _setup_logs():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def run_lidar(topic, source_topics, max_render_points, interface, status_writer=None):
    """Own Unitree DDS, ROS publication, and conversion in a child process."""
    node = None
    executor = None
    ros_initialized = False
    try:
        # Spawned processes do not inherit the main process's DDS participant.
        # Require its already-resolved adapter instead of SDK auto-selection.
        if (not isinstance(interface, str) or not interface.strip()
                or interface.strip().lower() == "auto"):
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
            raise RuntimeError("Lidar robot-interface DDS initialization failed") from exc
        rclpy.init(args=None)
        ros_initialized = True
        executor = SingleThreadedExecutor()
        node = _LidarNode(topic, executor, source_topics, max_render_points)
        if not node.subs:
            raise RuntimeError("Lidar has no initialized Unitree DDS subscriptions")
        if status_writer is not None:
            status_writer.send({"state": "ready"})
        executor.spin()
        raise RuntimeError("Lidar executor stopped unexpectedly")
    except Exception as exc:
        if status_writer is not None:
            try:
                status_writer.send({"state": "error", "error": f"{type(exc).__name__}: {str(exc)[:220]}"})
            except (BrokenPipeError, EOFError, OSError):
                pass
        raise
    finally:
        # A failed setup must release the successful earlier stages too.
        try:
            if node is not None:
                node.close()
        finally:
            try:
                if executor is not None:
                    executor.shutdown(timeout_sec=1.0)
            finally:
                try:
                    if ros_initialized:
                        rclpy.shutdown()
                finally:
                    if status_writer is not None:
                        status_writer.close()
