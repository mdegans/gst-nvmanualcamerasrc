import os
import sys
import unittest

PYMOD_PATH = os.environ["PYMOD_PATH"]
sys.path.append(PYMOD_PATH)

import nvmanual

# TODO(mdegans): these test cases can probably be programmatically created

class TestRectangle(unittest.TestCase):
    TYPES = (float, int)
    TEMPLATE_MAP = {
        f"{t.__class__.__name__.capitalize()}Rectangle": t for t in TYPES
    }

    def test_rectangle_values(self):
        for clsname, T in self.TEMPLATE_MAP.items():
            values = {
                'left': T(1),
                'top': T(2),
                'right': T(3),
                'bottom': T(4),
            }
            with self.subTest(clsname):
                cls = getattr(nvmanual.argus, clsname)
                rect = cls(**values)
                for k, v in values.items():
                    self.assertEqual(getattr(rect, k), v)


class TestRGBTuple(unittest.TestCase):
    TYPES = (float, int)
    TEMPLATE_MAP = {
        f"{t.__class__.__name__.capitalize()}RGBTuple": t for t in TYPES
    }

    def test_rgb_tuple(self):
        for clsname, T in self.TEMPLATE_MAP.items():
            values = {
                'r': T(1),
                'g': T(2),
                'b': T(3),
            }
            with self.subTest(clsname):
                cls = getattr(nvmanual.argus, clsname)
                rgb_tuple = cls(**values)
                for k, v in values.items():
                    self.assertEqual(getattr(rgb_tuple, k), v)


class TestBayerTuple(unittest.TestCase):
    TYPES = (float, int)
    TEMPLATE_MAP = {
        f"{t.__class__.__name__.capitalize()}BayerTuple": t for t in TYPES
    }

    def test_rgb_tuple(self):
        for clsname, T in self.TEMPLATE_MAP.items():
            values = {
                'r': T(1),
                'g_even': T(2),
                'g_odd': T(3),
                'b': T(4),
            }
            with self.subTest(clsname):
                cls = getattr(nvmanual.argus, clsname)
                bayer_tuple = cls(**values)
                for k, v in values.items():
                    self.assertEqual(getattr(bayer_tuple, k), v)
