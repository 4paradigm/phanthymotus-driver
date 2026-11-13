"""Offline coverage for mainline RPC request correlation."""
import queue
import threading

import rpc_proxy


def test_concurrent_calls_keep_replies_bound_to_their_request():
    proxy = rpc_proxy.RpcProxy.__new__(rpc_proxy.RpcProxy)
    proxy._lock = threading.Lock()
    proxy._req_counter = 0
    proxy._cmd_q = queue.Queue()
    proxy._result_q = queue.Queue()
    responses = {}

    def responder():
        for _ in range(2):
            cmd = proxy._cmd_q.get(timeout=2)
            proxy._result_q.put({'_req_id': cmd['_req_id'], 'result': cmd['method']})

    worker = threading.Thread(target=responder)
    callers = [threading.Thread(target=lambda name=name: responses.update({name: proxy._call(name, timeout=2)}))
               for name in ('first', 'second')]
    worker.start()
    for caller in callers:
        caller.start()
    for caller in callers:
        caller.join(timeout=3)
    worker.join(timeout=3)
    assert responses == {'first': 'first', 'second': 'second'}
    assert proxy._req_counter == 2
