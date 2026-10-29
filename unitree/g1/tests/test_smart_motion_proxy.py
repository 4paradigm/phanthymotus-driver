"""Offline coverage of subprocess logging initialization."""
from pathlib import Path


def test_spawned_rpc_worker_installs_logsafe_before_dds():
    source = Path(__file__).resolve().parents[1].joinpath('rpc_proxy.py').read_text(encoding='utf-8')
    worker = source[source.index('def _rpc_worker'):source.index('class RpcProxy')]
    assert worker.index('logsafe.install(check_fd=False)') < worker.index('ChannelFactoryInitialize(0, network_iface)')
