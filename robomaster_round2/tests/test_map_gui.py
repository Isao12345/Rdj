import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import Mock

from robomaster_round2.Slam.map_gui import draw_map, load_map, MapWindow


class TestMapGUI(unittest.TestCase):
    def test_draw_shared_open_edge_and_last_confirmed_cell(self):
        data = dict(rows=1, cols=2, start=[0, 0], goal=[0, 1],
                    maze=[[dict(N=True, E=False, S=None, W=True),
                           dict(N=True, E=True, S=None, W=False)]],
                    path=[[0, 0], [0, 1]], visited_cells=[[0, 0]])
        canvas = Mock()
        draw_map(canvas, data, 200, 120)
        self.assertEqual(canvas.create_rectangle.call_count, 2)
        canvas.create_oval.assert_called_once()
        for call in canvas.create_line.call_args_list:
            self.assertNotEqual(call.args, (100.0, 20.0, 100.0, 100.0))
        self.assertTrue(any(c.kwargs.get('dash') for c in canvas.create_line.call_args_list))

    def test_unscanned_map_has_no_position_marker(self):
        data = dict(rows=1, cols=1, start=[0, 0], goal=[0, 0], maze=None, path=[])
        canvas = Mock()
        draw_map(canvas, data, 200, 200)
        canvas.create_oval.assert_not_called()
        self.assertEqual(canvas.create_line.call_count, 4)

    def test_read_error_keeps_previous_data(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.json'
            path.write_text('{')
            with self.assertRaises(ValueError):
                load_map(path)
            window = MapWindow.__new__(MapWindow)
            window.path, window.status = path, Mock()
            previous = {'status': 'running'}
            window.data = previous
            window.reload()
            self.assertIs(window.data, previous)
            self.assertIn('อ่าน', window.status.set.call_args.args[0])
