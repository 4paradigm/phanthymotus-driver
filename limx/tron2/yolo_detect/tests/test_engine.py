"""Fake model/ROS only; no network, GPU or robot commands."""
import io,json,unittest
from PIL import Image
from engine import *

class Clock:
    def __init__(self):self.t=100.
    def __call__(self):return self.t
class Bus:
    def __init__(self):self.callback=None;self.sent=[];self.bound=None
    def bind(self,topic,callback):self.bound=topic;self.callback=callback
    def unbind(self):self.bound=None;self.callback=None
    def publish(self,raw,result,stamp):self.sent.append((raw,result,stamp))
class Model:
    def __init__(self):self.images=[];self.callback=None;self.fail=False;self.health_calls=0
    def health(self):self.health_calls+=1;return {'model':MODEL,'model_loaded':True,'status':'ok'}
    def detect(self,raw):
        self.images.append(raw)
        if self.callback:self.callback()
        if self.fail:raise RuntimeError('GPU busy')
        im=decode_jpeg(raw)
        return {'image_width':im.width,'image_height':im.height,'box_coordinate_system':'original_image_pixels','detections':[
            {'label':'book','confidence':.9,'box_xyxy':[10.,15.,80.,85.]},
            {'label':'bottle','confidence':.3,'box_xyxy':[90.,10.,130.,70.]}]}
def jpeg(color='white',size=(160,120)):
    out=io.BytesIO();Image.new('RGB',size,color).save(out,format='JPEG');return out.getvalue()
class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.bus=Bus();self.model=Model();self.e=Engine(self.bus,self.model,self.clock,self.clock)
    def start(self,**kwargs):return self.e.call(dict(action='start',**kwargs))
    def frame(self,raw=None):self.bus.callback(jpeg() if raw is None else raw,{'sec':10,'nanosec':20,'frame_id':'head'})
    def test_schema_ports_and_no_actuator(self):
        t=tool();self.assertEqual(t['type'],'processor');self.assertFalse(t['multiInstance'])
        self.assertEqual(t['topic_in'],[{'format':'image/jpeg','desc':'摄像头图像'}]);self.assertEqual(t['topic_out'][0]['format'],'image/jpeg')
        self.assertEqual(t['topic_out'][1]['format'],'data/json');self.assertNotIn('x-resource',t['inputSchema'])
    def test_dead_worker_reported_as_error_not_starting(self):
        from types import SimpleNamespace
        self.start();self.e.thread=SimpleNamespace(is_alive=lambda:False)
        info=self.e.info()
        self.assertEqual(info['state'],'error');self.assertFalse(info['available'])
        self.assertFalse(info['worker_alive']);self.assertIn('线程',info['last_error'])
    def test_live_worker_metadata_does_not_hide_valid_result(self):
        from types import SimpleNamespace
        self.start();self.frame();self.e.step();self.e.thread=SimpleNamespace(is_alive=lambda:True)
        info=self.e.info();self.assertTrue(info['worker_alive']);self.assertTrue(info['available'])
    def test_no_manual_parameter_fields(self):
        schema=tool()['inputSchema']
        self.assertEqual(set(schema['properties']),{'action','input_topic'})
        self.assertEqual(schema['x-action-params']['start']['params'],[])
        for key in ('confidence','fps','labels'):
            self.assertNotIn(key,schema['properties'])
    def test_low_score_results_are_not_filtered_by_card(self):
        d={'image_width':160,'image_height':120,'box_coordinate_system':'original_image_pixels','detections':[{'label':'item','confidence':.01,'box_xyxy':[0,0,20,20]}]}
        self.assertEqual(validate_detections(d,160,120,Settings())[0]['confidence'],.01)
    def test_legacy_tuning_does_not_change_runtime_settings(self):
        self.start(confidence=.99,fps=0,labels='nonexistent')
        self.assertEqual(self.e.settings.fps,2.)
        self.assertFalse(any(k in self.e.info() for k in ('confidence','labels_filter','requested_fps')))
    def test_info_stop_do_not_call_model_or_bind(self):
        self.assertFalse(self.e.call({'action':'info'})['active']);self.e.call({'action':'stop'})
        self.assertEqual(self.model.health_calls,0);self.assertEqual(self.model.images,[]);self.assertIsNone(self.bus.bound)
    def test_latest_frame_only_and_association(self):
        self.start();first=jpeg('red');latest=jpeg('blue');self.frame(first);self.frame(latest)
        self.assertEqual(self.e.overwritten,1);self.assertTrue(self.e.step());self.assertEqual(self.model.images,[latest])
        result=self.e.info()['latest_result'];self.assertEqual(result['source_jpeg_sha256'],hashlib.sha256(latest).hexdigest())
        self.assertEqual(result['frame_sequence'],2);self.assertEqual(result['source_header_stamp']['sec'],10)
        self.assertEqual(result['detections'][0]['label'],'book');self.assertFalse(result['measurement_time_verified'])
        self.assertNotEqual(self.bus.sent[-1][0],latest)
    def test_no_duplicate_inference_without_new_frame(self):
        self.start();self.frame();self.e.step();self.e.step();self.assertEqual(len(self.model.images),1)
    def test_legacy_filters_are_ignored_and_all_results_preserved(self):
        self.start(confidence=.5,labels='book');self.frame();self.e.step();r=self.e.info()['latest_result']
        self.assertEqual([x['label'] for x in r['detections']],['book','bottle'])
    def test_stop_discards_late_result(self):
        self.start();self.frame();self.model.callback=self.e.stop;self.assertFalse(self.e.step())
        self.assertFalse(self.e.info()['active']);self.assertFalse(any(m[1].get('available') for m in self.bus.sent))
    def test_camera_switch_discards_old_callback_and_old_result(self):
        self.start();old=self.bus.callback;self.frame()
        self.model.callback=lambda:self.start(input_topic=INPUTS[1])
        self.assertFalse(self.e.step());old(jpeg(),{})
        self.assertIsNone(self.e.pending);self.assertEqual(self.e.info()['input_topic'],INPUTS[1]);self.assertFalse(self.e.info()['available'])
    def test_invalid_input_and_settings_do_not_bind(self):
        for args in [dict(input_topic='/arm_control'),dict(input_topics=list(INPUTS)),dict(url='http://example.com')]:
            with self.subTest(args=args),self.assertRaises(ValueError):self.start(**args)
        self.assertIsNone(self.bus.bound)
    def test_failed_initial_publish_does_not_leave_active(self):
        def failure(*args):raise RuntimeError('DDS unavailable')
        self.bus.publish=failure
        with self.assertRaises(RuntimeError):self.start()
        self.assertFalse(self.e.active);self.assertIsNone(self.bus.bound)
    def test_active_start_idempotent(self):
        self.start();generation=self.e.generation;self.start();self.assertEqual(self.e.generation,generation);self.assertEqual(self.model.health_calls,1)
    def test_source_stale_is_not_inferred(self):
        self.start();self.frame();self.clock.t+=4;self.assertFalse(self.e.step());self.assertFalse(self.model.images)
    def test_result_stale_is_not_published(self):
        self.start();self.frame();self.model.callback=lambda:setattr(self.clock,'t',104.)
        self.assertFalse(self.e.step());self.assertFalse(any(m[1].get('available') for m in self.bus.sent))
    def test_busy_error_clears_boxes_and_recovers(self):
        self.start();self.frame();self.e.step();self.model.fail=True;self.frame();self.e.step()
        self.assertIsNone(self.e.info()['latest_result']);self.assertFalse(self.bus.sent[-1][1]['available'])
        self.model.fail=False;self.frame();self.assertTrue(self.e.step());self.assertTrue(self.e.info()['available'])
    def test_camera_loss_clears_last_boxes(self):
        self.start();self.frame();self.e.step();self.clock.t+=4;self.e.step()
        self.assertIsNone(self.e.info()['latest_result']);self.assertEqual(self.bus.sent[-1][1]['detections'],[])
    def test_bad_jpeg_never_reaches_model(self):
        self.start();self.frame(b'not a JPEG');self.assertFalse(self.e.step());self.assertEqual(self.model.images,[])
    def test_dimension_and_box_validation(self):
        settings=Settings();valid={'image_width':160,'image_height':120,'box_coordinate_system':'original_image_pixels','detections':[]}
        for patch in [dict(image_width=159),dict(box_coordinate_system='scaled'),dict(detections=[{'label':'x','confidence':.9,'box_xyxy':[0,0,200,30]}]),dict(detections=[{'label':'x','confidence':float('nan'),'box_xyxy':[0,0,20,30]}])]:
            with self.assertRaises(ValueError):validate_detections(dict(valid,**patch),160,120,settings)
    def test_oversized_source_is_dropped(self):
        self.start();self.frame(b'x'*(MAX_JPEG+1));self.assertIsNone(self.e.pending)
    def test_network_endpoint_cannot_be_changed(self):
        with self.assertRaises(ValueError):ModelClient('http://127.0.0.1:5000')
if __name__=='__main__':unittest.main(verbosity=2)
