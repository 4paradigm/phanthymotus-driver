"""Preview discovers chassis maps without loading or relocalizing the robot."""
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from spatial_map import SpatialMap


class PreviewCatalogTests(unittest.TestCase):
    def test_list_matches_chassis_union_and_selection_only_syncs(self):
        with tempfile.TemporaryDirectory() as directory:
            syncs = []
            archive = types.SimpleNamespace(root=Path(directory),
                client=types.SimpleNamespace(maps=lambda:[{'fileName':'new.pgm'}, {'fileName':'old.pgm'}]),
                sync=syncs.append)
            preview = SpatialMap(None, 'test', archive, None, None)
            with patch.object(preview, 'snapshots', return_value={'local':None, 'old':None}):
                self.assertEqual(preview.dispatch('list_maps', {})['maps'], ['local','new','old'])
                self.assertEqual(syncs, [])
                result = preview.dispatch('select_map', {'map_name':'new'})
                self.assertEqual(result, {'state':'completed', 'selected_map':'new'})
                self.assertEqual(syncs, ['new'])
                self.assertEqual(preview.selected, 'new')

    def test_sync_failure_is_not_misreported_as_a_missing_map(self):
        with tempfile.TemporaryDirectory() as directory:
            def sync(name):
                raise InterruptedError('mapping_in_progress')
            archive = types.SimpleNamespace(root=Path(directory),
                client=types.SimpleNamespace(maps=lambda:[{'fileName':'new.pgm'}]), sync=sync)
            preview = SpatialMap(None, 'test', archive, None, None)
            reply = preview.dispatch('select_map', {'map_name':'new'})
            self.assertEqual(reply['error'], 'map_preview_sync_failed')
            self.assertIsNone(preview.selected)
            self.assertEqual(preview.dispatch('list_maps', {})['maps'], ['new'])
