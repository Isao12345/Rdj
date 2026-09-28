import json
from pathlib import Path
import tempfile
import unittest

from robomaster_round2.Slam.map_viewer import main, render_map


class TestMapViewer(unittest.TestCase):
    def setUp(self):
        self.data = dict(rows=1, cols=2, start=[0, 0], goal=[0, 1],
                         maze=[[dict(N=True, E=False, S=None, W=True),
                                dict(N=True, E=True, S=None, W=False)]],
                         path=[[0, 0], [0, 1]], visited_cells=[[0, 0]], status='failed')

    def test_render_unknown_walls_path_and_escaped_error(self):
        self.data['error'] = '<script>alert(1)</script>'
        document = render_map(self.data)
        self.assertIn('stroke-dasharray="5 5"', document)
        self.assertIn('<polyline', document)
        self.assertIn('<circle', document)
        self.assertIn('&lt;script&gt;', document)
        self.assertNotIn('<script>', document)
        # Shared open edge must not be drawn as a wall.
        self.assertNotIn('<line x1="110" y1="30" x2="110" y2="110"', document)

    def test_map_before_first_scan(self):
        self.data.update(maze=None, path=[], visited_cells=[])
        document = render_map(self.data)
        self.assertNotIn('<circle', document)
        self.assertIn('0/2', document)

    def test_invalid_dimensions_and_coordinates(self):
        for changes in (dict(rows=0), dict(path=[[2, 0]]), dict(maze=[])):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                render_map(dict(self.data, **changes))

    def test_cli_preserves_json_and_creates_html(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'map.json'
            original = json.dumps(self.data)
            source.write_text(original)
            main([str(source)])
            self.assertTrue(source.with_suffix('.html').exists())
            self.assertEqual(source.read_text(), original)
            with self.assertRaises(SystemExit):
                main([str(source), '--output', str(source)])
            self.assertEqual(source.read_text(), original)
