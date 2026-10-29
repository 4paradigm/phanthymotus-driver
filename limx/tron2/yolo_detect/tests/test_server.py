"""MCP protocol tests; optional real loopback only in isolated Docker proof."""
import io,json,os,threading,unittest,urllib.request
from types import SimpleNamespace
from server import Handler,ThreadingHTTPServer
from engine import Engine,tool
from test_engine import Bus,Model,Clock

class Tests(unittest.TestCase):
    def setUp(self):self.e=Engine(Bus(),Model(),Clock(),Clock())
    def request(self,method='tools/list',params=None,rid=1):
        body=json.dumps({'jsonrpc':'2.0','id':rid,'method':method,'params':params or {}}).encode()
        h=Handler.__new__(Handler);h.path='/mcp';h.headers={'Content-Length':str(len(body))};h.rfile=io.BytesIO(body);h.server=SimpleNamespace(engine=self.e);out=[]
        h.send=lambda status,value,ctype='application/json':out.append((status,value,ctype));h.do_POST();return out[0]
    def test_initialize_and_tools(self):
        self.assertEqual(self.request('initialize')[1]['result']['serverInfo']['name'],'TRON2 YOLO-World Vision')
        self.assertEqual(self.request()[1]['result']['tools'],[tool()])
    def test_live_call_and_stop(self):
        r=self.request('tools/call',{'name':'yolo_detect','arguments':{'action':'start'}})[1]['result'];self.assertFalse(r['isError']);self.assertTrue(self.e.active)
        self.request('tools/call',{'name':'yolo_detect','arguments':{'action':'stop'}});self.assertFalse(self.e.active)
    def test_unknown_or_malformed_tools_fail_without_binding(self):
        for params in ({'name':'arm_control','arguments':{}},{'name':'yolo_detect','arguments':{'action':'move'}},{'name':'yolo_detect','arguments':{'action':'start','input_topic':'/unknown'}}):
            self.request('tools/call',params);self.assertFalse(self.e.active)
    def test_unknown_method_and_resources(self):
        self.assertEqual(self.request('resources/list')[1]['result'],{'resources':[]})
        self.assertEqual(self.request('unrecognized')[1]['error']['code'],-32601)
    def test_info_is_read_only(self):
        self.request('tools/call',{'name':'yolo_detect','arguments':{'action':'info'}});self.assertFalse(self.e.active)
    def test_chunked_requests_rejected(self):
        h=Handler.__new__(Handler);h.path='/mcp';h.headers={'Transfer-Encoding':'chunked'};h.server=SimpleNamespace(engine=self.e);out=[];h.send=lambda *a:out.append(a);h.do_POST()
        self.assertEqual(out[0][0],400);self.assertFalse(self.e.active)
    @unittest.skipUnless(os.environ.get('YOLO_HTTP_TEST')=='1','Actual socket exercised inside isolated deployment test container')
    def test_real_loopback_http(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.engine=self.e;t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
        try:
            opener=urllib.request.build_opener(urllib.request.ProxyHandler({}));base='http://127.0.0.1:'+str(server.server_port)
            with opener.open(base+'/health') as r:self.assertEqual(json.load(r)['service'],'tron2-yolo-card')
            with opener.open(base+'/latest.jpg') as r:self.assertTrue(r.read().startswith(b'\xff\xd8'))
            data=json.dumps({'jsonrpc':'2.0','id':2,'method':'tools/list','params':{}}).encode()
            with opener.open(urllib.request.Request(base+'/mcp',data=data,headers={'Content-Type':'application/json'})) as r:self.assertEqual(json.load(r)['result']['tools'][0]['name'],'yolo_detect')
        finally:server.shutdown();server.server_close();t.join()
if __name__=='__main__':unittest.main(verbosity=2)
