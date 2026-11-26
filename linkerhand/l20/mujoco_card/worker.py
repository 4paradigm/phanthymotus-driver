"""Host the virtual hand player and any configured trajectory producers."""
from http.server import ThreadingHTTPServer
import os
from pathlib import Path
import signal
import threading

import yaml

from common.vendor_runtime import DriverBundle, make_handler
from mujoco_card.playback import Playback
from mujoco_card.simulation import CARD, HERE, Simulation
from mujoco_card.trajectories import SIDES, initial_poses


def build_virtual_cards(config, runtime):
    cards = {CARD: Simulation(runtime)}
    plugins = config.get("plugins", {})
    for name in ("hand_gesture", "hand_grasp"):
        public = name + "_sim"
        # Older installations declared only the source card's physical motion settings.
        enabled = plugins.get(public, {}).get("enabled", name in plugins)
        if enabled:
            from simulated_motion_device import SimulatedMotionCard
            cards[public] = SimulatedMotionCard(name, runtime, config)
    return cards


def main():
    from common import logsafe
    from mujoco_card.hand_model import HandModel
    from mujoco_card.sim_bus import SimulationBus, deny_hardware_sockets
    logsafe.install()
    deny_hardware_sockets()
    models={s:HandModel(HERE/'vendor.zip',s) for s in SIDES}
    config = yaml.safe_load(Path(os.environ.get("CONFIG_PATH", HERE.parent / "config.yaml")).read_text())
    bus=SimulationBus(int(os.getenv('ROS_DOMAIN_ID','42')))
    rt=Playback(models,initial_poses(config),bus,os.getenv('SIM_STATE_DIR','/var/lib/l20-sim'))
    bundle = DriverBundle(build_virtual_cards(config, rt).values())
    port = int(config['plugins']['l20_mujoco']['worker_port'])
    server = ThreadingHTTPServer(('127.0.0.1', port),
                                 make_handler(lambda: bundle, 'L20 仅仿真', 'linkerhand-g20-simulation'))
    def halt(*_):
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,halt);signal.signal(signal.SIGINT,halt)
    try:server.serve_forever()
    finally:rt.shutdown();bus.shutdown();server.server_close()


if __name__=='__main__': main()
