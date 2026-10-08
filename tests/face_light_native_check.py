"""Run in network-disabled Linux with verified SDK_DIR and REAL_ADAPTER paths.
Uses the real SDK only for readiness and expected offline send failures.
Native success tests use a fake SDK and Unix-domain sockets, never robot IO.
"""
import os
from pathlib import Path
import subprocess
import socket
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'unitree/go1'))
from ext_devices import FaceLightPlugin

# Some kernels expose inactive tunnel devices even with Docker --network none.
active = {name for _, name in socket.if_nameindex()
          if int(Path('/sys/class/net', name, 'flags').read_text(), 16) & 1}
if active != {'lo'} or len(Path('/proc/net/route').read_text().splitlines()) > 1:
    raise RuntimeError('Native real-SDK tests require --network none: only lo active, no IPv4 routes')

sdk = Path(os.environ['SDK_DIR'])
real = os.environ['REAL_ADAPTER']
with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    mock_library = tmp / 'libfake_face.so'
    mock = tmp / 'mock_adapter'
    subprocess.run(['g++', '-std=c++11', '-fPIC', '-shared', '-I', str(sdk / 'include'),
                    str(ROOT / 'tests/fixtures/face_light/fake_sdk.cpp'), '-o', str(mock_library)], check=True)
    subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-I', str(sdk / 'include'),
                    str(ROOT / 'unitree/go1/deploy/face_light/adapter.cpp'), str(mock_library),
                    '-Wl,--export-dynamic', '-Wl,-rpath,' + str(tmp), '-ldl', '-o', str(mock)], check=True)
    os.environ['FACE_TEST_LOG'] = str(tmp / 'frames')
    plugin = FaceLightPlugin({'backend': 'sdk', 'sdk_executable': str(mock), 'sdk_exclusive': True}, '', None, None)
    assert plugin.start()['ok']
    process = plugin._backend.process
    colors = [[i, 100 + i, 255 - i] for i in range(12)]
    result = plugin.dispatch('set_leds', {'colors': colors})
    assert result['ok'], result
    logged = [int(v) for v in (tmp / 'frames').read_text().split()]
    assert logged == [v for rgb in colors for v in rgb]
    assert plugin.dispatch('set_led', {'index': 11, 'g': 77})['ok']
    assert plugin.stop()['ok'] and process.poll() is not None
    print('PASS: native 12-LED SDK calls, RGB order, socket-send tracking and process cleanup (fake SDK)')
    bad = subprocess.run([str(mock)], input='1 2 3\n' + '0 ' * 35 + '999\n',
                         text=True, capture_output=True, check=True)
    assert bad.stdout.count('ERROR expected exactly 36') == 2, bad.stdout
    print('PASS: native invalid frames rejected before SDK send')
    os.environ['FACE_TEST_ERROR'] = '1'
    plugin = FaceLightPlugin({'backend': 'sdk', 'sdk_executable': str(mock), 'sdk_exclusive': True}, '', None, None)
    assert plugin.start()['ok']
    process = plugin._backend.process
    result = plugin.dispatch('set_color', {'r': 2})
    assert not result['ok'] and 'Bad file descriptor' in result['message'], result
    assert process.poll() is not None
    plugin.stop()
    print('PASS: native SDK send failure returns explicit error and reaps process')
    del os.environ['FACE_TEST_ERROR']
    # Run this ONLY in a --network none container. Real SDK owns its destination.
    plugin = FaceLightPlugin({'backend': 'sdk', 'sdk_executable': real, 'sdk_exclusive': True, 'sdk_dir': str(sdk)}, '', None, None)
    assert plugin.start()['ok']
    process = plugin._backend.process
    result = plugin.dispatch('set_color', {'r': 0, 'g': 0, 'b': 0})
    assert not result['ok'] and 'SDK UDP send failed' in result['message'], result
    assert process.poll() is not None
    plugin.stop()
    print('PASS: actual ARM64 vendor library loads; network-unavailable error observed without robot connection')
