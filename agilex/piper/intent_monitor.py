"""Background pose service. Only its model is a public resource."""
from copy import deepcopy
from agilex.piper.monitor import PiperMonitorPlugin
from agilex.piper.status_card import PiperStatusCard


class IntentMonitor(PiperStatusCard):
    def get_tools(self):
        model = deepcopy(self.monitor.get_tools()[1])
        model['title'] = 'Skeleton model'
        model['description'] = 'PiPER URDF for the monitor renderer; no control commands.'
        return [model]


def add_intent_monitor(plugins, config, ros2):
    runtime = plugins[0]
    monitor = PiperMonitorPlugin(runtime.read_monitor_snapshot,
        namespace=runtime.topic.rsplit('/', 1)[0].strip('/'), device_name='PiPER ' + config.get('can_interface', 'can0'),
        ros2=ros2, language='en')
    background = IntentMonitor(runtime, monitor, auto_connect=True)
    runtime.monitor_service = background
    return [*plugins, background]
