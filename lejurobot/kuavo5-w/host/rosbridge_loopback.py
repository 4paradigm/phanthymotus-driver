#!/usr/bin/env python3
"""Process-local workaround for rosbridge 0.11.17's omitted bind interface."""
import runpy
import sys
from autobahn.twisted import websocket

_listen = websocket.listenWS


def listen_loopback(factory, contextFactory=None, backlog=50, interface=''):
    return _listen(factory, contextFactory, backlog, interface='127.0.0.1')


if __name__ == '__main__':
    websocket.listenWS = listen_loopback
    if len(sys.argv) < 2:
        raise SystemExit('Expected the rosbridge node executable')
    sys.argv = sys.argv[1:]
    runpy.run_path(sys.argv[0], run_name='__main__')
