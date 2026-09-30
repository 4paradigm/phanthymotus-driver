"""OS-process entry points for high-volume AS2 sensors.

The worker owns both the vendor subscriber/RPC and the ROS publisher.  This
keeps a blocked videohub call or a slow point-cloud conversion out of the
driver's control and state executor.
"""
import os


def _setup_logs():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def run_lidar(topic, source_topics, max_render_points, interface):
    _setup_logs()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from lidar import _LidarNode
    try:
        ChannelFactoryInitialize(0, interface or None)
    except Exception as exc:
        print(f"[as2w-lidar-process] Unitree DDS init failed: {str(exc)[:180]}", flush=True)
    rclpy.init(args=None)
    executor = SingleThreadedExecutor()
    node = _LidarNode(topic, executor, source_topics, max_render_points)
    try:
        executor.spin()
    finally:
        node.close()
        rclpy.shutdown()


def run_camera(topic, fps, interface, stop_event=None):
    _setup_logs()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rpc_proxy import RpcProxy
    from device import _CameraRgbNode
    rclpy.init(args=None)
    executor = SingleThreadedExecutor()
    proxy = RpcProxy(interface)
    node = _CameraRgbNode(topic, proxy, fps)
    executor.add_node(node.node)
    node.start()
    stop_thread = None
    if stop_event is not None:
        import threading

        def wait_for_stop():
            stop_event.wait()
            if stop_event.is_set():
                executor.shutdown()

        stop_thread = threading.Thread(target=wait_for_stop, daemon=True,
                                       name="as2w-camera-stop-watcher")
        stop_thread.start()
    try:
        executor.spin()
    finally:
        node.stop()
        proxy.stop()
        if stop_thread is not None:
            stop_thread.join(timeout=1.0)
        rclpy.shutdown()
