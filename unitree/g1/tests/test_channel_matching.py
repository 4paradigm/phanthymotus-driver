import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


G1_DIR = Path(__file__).resolve().parents[1]


class _Listener:
    def __init__(self, **callbacks):
        for name, callback in callbacks.items():
            setattr(self, name, callback)


class _DataReader:
    instances = []

    def __init__(self, _participant, _topic, _qos=None, listener=None):
        self.listener = listener
        self.samples = []
        self.instances.append(self)

    def take(self, _count):
        samples, self.samples = self.samples, []
        return samples


def _module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    return module


def _load_channel_module():
    class _DDSException(Exception):
        msg = ''

    class _DDSCTypes:
        class publication_matched_status:
            pass

    class _Singleton:
        pass

    class _BQueue:
        pass

    cyclonedds_modules = {
        'cyclonedds': _module('cyclonedds'),
        'cyclonedds.domain': _module(
            'cyclonedds.domain', Domain=type('Domain', (), {}),
            DomainParticipant=type('DomainParticipant', (), {}),
        ),
        'cyclonedds.internal': _module(
            'cyclonedds.internal', dds_c_t=_DDSCTypes,
            InvalidSample=type('InvalidSample', (), {}),
        ),
        'cyclonedds.pub': _module(
            'cyclonedds.pub', DataWriter=type('DataWriter', (), {}),
        ),
        'cyclonedds.sub': _module(
            'cyclonedds.sub', DataReader=_DataReader,
        ),
        'cyclonedds.topic': _module(
            'cyclonedds.topic', Topic=type('Topic', (), {}),
        ),
        'cyclonedds.qos': _module(
            'cyclonedds.qos', Qos=type('Qos', (), {}),
        ),
        'cyclonedds.core': _module(
            'cyclonedds.core', DDSException=_DDSException,
            Listener=_Listener,
        ),
        'cyclonedds.util': _module(
            'cyclonedds.util', duration=lambda **kwargs: kwargs,
        ),
    }
    package_modules = {
        'unitree_sdk2py': _module('unitree_sdk2py'),
        'unitree_sdk2py.core': _module('unitree_sdk2py.core'),
        'unitree_sdk2py.core.channel_config': _module(
            'unitree_sdk2py.core.channel_config',
            ChannelConfigAutoDetermine='', ChannelConfigHasInterface='',
        ),
        'unitree_sdk2py.utils': _module('unitree_sdk2py.utils'),
        'unitree_sdk2py.utils.singleton': _module(
            'unitree_sdk2py.utils.singleton', Singleton=_Singleton,
        ),
        'unitree_sdk2py.utils.bqueue': _module(
            'unitree_sdk2py.utils.bqueue', BQueue=_BQueue,
        ),
    }
    name = 'unitree_sdk2py.core.channel_under_test'
    spec = importlib.util.spec_from_file_location(
        name, G1_DIR / 'unitree_sdk2py/core/channel.py',
    )
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(
        sys.modules, {**cyclonedds_modules, **package_modules},
    ):
        spec.loader.exec_module(module)
    return module


class ChannelMatchingTests(unittest.TestCase):
    def setUp(self):
        _DataReader.instances.clear()
        self.channel = _load_channel_module()

    def test_reader_reports_publisher_matching_and_direct_data(self):
        received = []
        matches = []
        reader_wrapper = self.channel.Channel._Channel__Reader()
        reader_wrapper.Init(
            object(), object(), handler=received.append, queueLen=0,
            matchHandler=matches.append,
        )
        reader = _DataReader.instances[-1]

        reader.listener.on_subscription_matched(
            reader, types.SimpleNamespace(current_count=1),
        )
        sample = types.SimpleNamespace(data='{"play_state":1}')
        reader.samples.append(sample)
        reader.listener.on_data_available(reader)

        self.assertTrue(reader_wrapper.WaitForPublisher(timeout=0))
        self.assertEqual(matches, [1])
        self.assertEqual(received, [sample])

        reader.listener.on_subscription_matched(
            reader, types.SimpleNamespace(current_count=0),
        )
        self.assertFalse(reader_wrapper.WaitForPublisher(timeout=0))
        self.assertEqual(matches, [1, 0])


if __name__ == '__main__':
    unittest.main()
