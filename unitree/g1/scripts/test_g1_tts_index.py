"""Exercise the real SDK method without importing DDS or sending robot commands."""
import ast
import json
from pathlib import Path
import unittest


class TtsIndexTests(unittest.TestCase):
    def test_consecutive_requests_have_distinct_increasing_indices(self):
        path = Path(__file__).resolve().parents[1] / 'unitree_sdk2py/g1/audio/g1_audio_client.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'AudioClient')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'TtsMaker')
        scope = {'json': json, 'ROBOT_API_ID_AUDIO_TTS': 1001}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
        sent = []

        class Client:
            tts_index = 0

            def _Call(self, api_id, parameter):
                sent.append((api_id, json.loads(parameter)))
                return 0, None

        client = Client()
        for text in ('第一条', '第二条', '第三条'):
            self.assertEqual(scope['TtsMaker'](client, text, 0), 0)
        self.assertEqual([p['index'] for _, p in sent], [1, 2, 3])
        self.assertEqual([p['text'] for _, p in sent], ['第一条', '第二条', '第三条'])
        self.assertTrue(all(api == 1001 and p['speaker_id'] == 0 for api, p in sent))


if __name__ == '__main__':
    unittest.main()
