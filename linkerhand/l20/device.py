#!/usr/bin/env python3
"""LinkerHand L20 lifecycle adapter; no movement during startup or lifecycle calls."""
import threading
import can
from common.vendor_runtime import action_schema, tool

class HandPlugin:
    def __init__(self, config):
        self.iface = config['connection']['can_iface']
        self.can_id = int(config['hands'][0]['can_id'])
        self.lock = threading.RLock()
        self.bus = None
        self.last_command = None

    def start(self):
        with self.lock:
            if self.bus is None:
                self.bus = can.interface.Bus(channel=self.iface, interface='socketcan')

    def stop(self):
        with self.lock:
            if self.bus is not None:
                self.bus.shutdown()
                self.bus = None

    def get_tool(self):
        return tool('hand', 'actuator', 'LinkerHand L20 open/close control',
                    action_schema({'open':([], 'Open fingers'), 'close':([], 'Close fingers')}, {}))

    def info(self):
        return {'state':'ready' if self.bus is not None else 'stopped',
                'can_iface':self.iface, 'can_id':self.can_id, 'side':'right',
                'last_command':self.last_command, 'feedback_available':False,
                'topic_in':[], 'topic_out':[]}

    def dispatch(self, action, args):
        with self.lock:
            if action == 'info':
                return self.info()
            if action == 'config':
                # There are no user-editable hardware fields; ignore Core tracing metadata.
                forbidden = {'can_iface','can_id','hands','connection','mcp_port','ros'} & set(args)
                if forbidden:
                    raise ValueError('Hardware settings are fixed by this deployment')
                return {'state':'configured', 'config':{}, 'adapter_ok':True}
            if action == 'start':
                self.start()
                return self.info()
            if action == 'stop':
                self.stop()
                return {**self.info(), 'state':'idle'}
            if action not in ('open','close'):
                raise ValueError('Unsupported hand action')
            if self.bus is None:
                raise ValueError('Hand is stopped; start the card before sending commands')
            value = 255 if action == 'open' else 0
            self.bus.send(can.Message(arbitration_id=self.can_id, is_extended_id=False,
                                      data=[2,value,value,value,value,value]), timeout=1.0)
            self.last_command = action
            return {'state':'command_sent', 'action':action, 'feedback_available':False,
                    'message':'CAN command sent; physical completion is not measured'}

def build_plugins(config, namespace, ros2=None):
    if config.get('connection',{}).get('can_iface') != 'can1' or config.get('hands') != [{'side':'right','can_id':39}]:
        raise ValueError('Unexpected hand hardware configuration')
    if not config.get('plugins',{}).get('hand',{}).get('enabled', True):
        return []
    return [HandPlugin(config)]
