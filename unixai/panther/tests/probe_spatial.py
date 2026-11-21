"""Read-only Panther ROS contract probe; --mapping exercises stationary SLAM.
Never sends a navigation goal or a velocity command.
"""
import json
import sys
import threading
import time
from pathlib import Path


def main():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from controlled_spatial import ControlledSpatialPlugin
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    plugin = ControlledSpatialPlugin({'native_slam_pcd_dir': '/tmp/panther-spatial-validation/maps',
                    'native_slam_db_path': '/tmp/panther-spatial-validation/controlled_spatial.db',
                    'vendor_map_dir': '/home/unix_ai/map_dir', 'service_timeout_s': 25,
                    'single_lidar_topic': '/lidar_front/scan'}, 'validation', executor)
    plugin.start()
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(3)
        maps = plugin.dispatch('list_maps', {})
        report = {'maps': maps, 'info': plugin.info(),
                  'nav2_available': plugin.client.wait_for_server(timeout_sec=3),
                  'slam_publishers': plugin.node.get_publisher_names_and_types_by_node('slam_toolbox', '/')}
        if '--mapping' in sys.argv:
            if maps.get('vendor_maps') and '--allow-reset-with-saved-maps' not in sys.argv:
                raise RuntimeError('Refusing to reset: saved maps exist; explicitly pass --allow-reset-with-saved-maps after checking the active session')
            name = 'validation_' + time.strftime('%Y%m%d_%H%M%S')
            report['start_mapping'] = plugin.dispatch('start_mapping', {'map_name': name})
            for _ in range(15):
                time.sleep(1)
                if plugin.info()['pose'] is not None and plugin.grid is not None:
                    break
            report['mapping_info'] = plugin.info()
            if plugin.grid:
                report['grid'] = {'width': plugin.grid.info.width, 'height': plugin.grid.info.height,
                                  'resolution': plugin.grid.info.resolution}
            report['tag'] = plugin.dispatch('tag_place', {'name': 'stationary_start', 'description': 'Stationary interface verification; no driving performed'})
            report['save'] = plugin.dispatch('stop_mapping', {})
            if report['save'].get('state') == 'saved':
                report['load'] = plugin.dispatch('load_map', {'map_name': name})
                time.sleep(5)
                report['loaded_info'] = plugin.info()
            report['final_maps'] = plugin.dispatch('list_maps', {})
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    finally:
        plugin.stop()
        executor.shutdown()
        thread.join(timeout=3)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
