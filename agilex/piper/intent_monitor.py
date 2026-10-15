"""Independent read-only Joints card on the control runtime's shared receiver."""
from copy import deepcopy
from common.vendor_runtime import tool
from agilex.piper.monitor import PiperMonitorPlugin
from agilex.piper.status_card import PiperStatusCard


class IntentMonitor(PiperStatusCard):
    def __init__(self, *args, joints_card=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.joints_card = joints_card

    def get_tools(self):
        model = deepcopy(self.monitor.get_tools()[1])
        model['title'] = 'Skeleton model'
        model['description'] = 'PiPER URDF for the monitor renderer; no control commands.'
        if not self.joints_card:
            return [model]
        joints = tool('joints', 'sensor',
            'Read-only PiPER joint feedback at 10 Hz. No motor, mode or motion commands. '
            'Monitoring is independent of Arm control. Stale feedback freezes the skeleton.',
            {'type': 'object', 'properties': {}, 'additionalProperties': False},
            topic_out=self.monitor._topics())
        joints['title'] = 'Joints'
        return [joints, model]

    def start(self):
        if self.joints_card:
            with self.runtime.lifecycle:
                if self.runtime.closed:
                    if self.runtime.gate.locked():
                        raise RuntimeError('The driver is stopping; wait before starting Joints.')
                    # Reopen only the read receiver after a global stop. The
                    # control card still needs its own Start before any action.
                    self.runtime.closed = False
                    self.runtime.stopped_cards.add('arm_control')
                    self.runtime.cancel.clear()
        try:
            return super().start()
        except RuntimeError as exc:
            if not self.joints_card:
                raise
            from agilex.piper.intent_control import public_error
            raise RuntimeError(public_error(exc)) from exc

    def info(self):
        if not self.joints_card:
            return super().info()
        result = self.monitor.info()
        # Diagnostics remain available on demand, but never become a JSON stream.
        error = self._connection_error
        if self.joints_card and error:
            from agilex.piper.intent_control import public_error
            error = public_error(error)
        result.update(connection_error=error,
                      connection_stopping=bool(self._connection_stop.is_set()
                          and self._connection_thread is not None and self._connection_thread.is_alive()))
        if result['connection_stopping']:
            result['state'] = 'stopping'
        return result

    def dispatch(self, action, args):
        if not self.joints_card:
            return super().dispatch(action, args)
        if args.get('_tool_name') == 'model':
            return self.monitor.dispatch(action, args)
        # Generic Core sensor Info adds an unused empty input_topic.
        allowed = {'action', 'instance_id', '_tool_name'}
        if args.get('input_topic') == '':
            allowed.add('input_topic')
        if set(args) - allowed:
            raise ValueError('Joints does not accept control parameters.')
        if action == 'joints':
            action = 'info'
        if action not in {'start', 'stop', 'info'}:
            raise ValueError('Joints only supports read-only lifecycle operations.')
        return getattr(self, action)()


def add_intent_monitor(plugins, config, ros2, *, joints_card=False):
    runtime = plugins[0]
    monitor = PiperMonitorPlugin(runtime.read_monitor_snapshot,
        namespace=runtime.topic.rsplit('/', 1)[0].strip('/'), device_name='PiPER ' + config.get('can_interface', 'can0'),
        ros2=ros2, language='en', publish_parameters=not joints_card)
    background = IntentMonitor(runtime, monitor, auto_connect=True, joints_card=joints_card)
    runtime.monitor_service = background
    return [*plugins, background]
