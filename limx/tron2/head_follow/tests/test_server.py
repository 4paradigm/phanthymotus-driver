import io
import os
from email.message import Message
from types import SimpleNamespace
import json
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from engine import Engine
from server import Handler
from test_follow import Fake


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.f=Fake();self.e=Engine(self.f,lambda:self.f.t,lambda:self.f.t)
        self.s=None
        if os.environ.get('FOLLOW_HTTP_TEST')=='1':
            self.s=ThreadingHTTPServer(('127.0.0.1',0),Handler);self.s.engine=self.e
            self.t=threading.Thread(target=self.s.serve_forever,daemon=True);self.t.start()

    def tearDown(self):
        if self.s:
            self.s.shutdown();self.s.server_close();self.t.join()

    def rpc(self,method,params=None):
        if self.s is None:
            raw=json.dumps({'jsonrpc':'2.0','id':1,'method':method,'params':params or {}}).encode()
            h=Handler.__new__(Handler);h.path='/mcp';h.headers=Message();h.headers['Content-Length']=str(len(raw))
            h.rfile=io.BytesIO(raw);h.wfile=io.BytesIO();h.server=SimpleNamespace(engine=self.e)
            h.request_version='HTTP/1.1';h.requestline='POST /mcp HTTP/1.1';h.command='POST'
            h.do_POST()
            return json.loads(h.wfile.getvalue().split(b'\r\n\r\n',1)[1])
        req=urllib.request.Request('http://127.0.0.1:%d/mcp'%self.s.server_port,
            data=json.dumps({'jsonrpc':'2.0','id':1,'method':method,'params':params or {}}).encode(),
            headers={'Content-Type':'application/json'})
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req,timeout=3) as r:return json.load(r)

    @unittest.skipUnless(os.environ.get('FOLLOW_HTTP_TEST')=='1','真实TCP/HTTP在Jetson隔离容器强制执行')
    def test_real_http_listener(self):
        self.assertEqual(self.rpc('initialize')['result']['serverInfo']['version'],'0.1.3')

    def test_schema_exposes_only_follow(self):
        tool=self.rpc('tools/list')['result']['tools'][0]
        self.assertEqual(tool['name'],'head_visual_follow')
        self.assertEqual([a for a in tool['inputSchema']['properties']['action']['enum'] if a not in ('stop','info','start')],['follow'])
        self.assertNotIn('status',tool['inputSchema']['properties']['action']['enum'])
        self.assertEqual(self.f.calls,[])

    def test_lifecycle_start_never_arms(self):
        r=self.rpc('tools/call',{'name':'head_visual_follow','arguments':{'action':'start'}})
        self.assertFalse(r['result']['structuredContent']['active'])

    def test_manual_enable_then_lifecycle_stop(self):
        p={'name':'head_visual_follow','arguments':{'action':'follow'}}
        self.assertTrue(self.rpc('tools/call',p)['result']['structuredContent']['active'])
        p['arguments']['action']='stop'
        self.assertFalse(self.rpc('tools/call',p)['result']['structuredContent']['active'])
        self.assertEqual(self.f.calls,[])

    def test_unknown_action_has_tool_error(self):
        r=self.rpc('tools/call',{'name':'head_visual_follow','arguments':{'action':'reset'}})
        self.assertTrue(r['result']['isError']);self.assertEqual(self.f.calls,[])


if __name__=='__main__':unittest.main(verbosity=2)
