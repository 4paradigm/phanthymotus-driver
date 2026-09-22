"""Run only in an isolated candidate container: actual bundle, no robot network."""
import sys
import time
from types import SimpleNamespace

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

sys.path.insert(0, '/work')
from main import TianyiDeviceBundle

contexts = [Context(), Context()]
executors = []
bundle = None
try:
    for context, domain in zip(contexts, (0, 42)):
        rclpy.init(context=context, domain_id=domain)
        executors.append(SingleThreadedExecutor(context=context))
    ros = SimpleNamespace(ctx_tianyi=contexts[0], ctx_core=contexts[1],
                          executor_tianyi=executors[0], executor_core=executors[1])
    cfg = {'plugins': {'arm': {'enabled': True}, 'hand': {'enabled': True}},
           'teleop': {'enabled': True, 'live_enabled': False}}
    bundle = TianyiDeviceBundle(cfg, 'isolated_tianyi', ros, None)
    assert any(t['name'] == 'teleop_executor' for t in bundle.get_all_tools())
    bundle.start_all()
    teleop = bundle._teleop
    for cycle in range(2):
        teleop.start()
        time.sleep(0.4)
        assert teleop._bus_process.poll() is None
        assert teleop._thread.is_alive()
        result = bundle.dispatch('teleop_executor', {'action': 'claim'})
        assert result.get('code') == 'live_acceptance_missing', result
        status = teleop.info()
        assert not status['ownership_held'] and not status['publisher_present'], status
        assert teleop.arm._pos_publisher is None
        assert teleop.hand._left_pub is None and teleop.hand._right_pub is None
        assert not teleop.node.count_publishers('/arm/cmd_pos')
        teleop.stop()
        assert teleop.node is None and teleop._thread is None
        assert teleop._bus_process is None
    print('IMAGE SHADOW PASS: actual bundle; 2 restart cycles; claim rejected; hardware publishers=0')
finally:
    if bundle is not None:
        bundle.stop_all()
    for executor in executors:
        executor.shutdown()
    for context in contexts:
        rclpy.try_shutdown(context=context)
