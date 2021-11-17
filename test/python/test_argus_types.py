import os
import sys
import unittest

PYMOD_PATH = os.environ["PYMOD_PATH"]
sys.path.append(PYMOD_PATH)

import nvmanual

# TODO(mdegans): these test cases can probably be programmatically created


class TestRectangle(unittest.TestCase):
    TYPES = (float, int)
    TEMPLATE_MAP = {f"{t.__name__.capitalize()}Rectangle": t for t in TYPES}

    def test_rectangle_values(self):
        for clsname, T in self.TEMPLATE_MAP.items():
            values = {
                "left": T(1),
                "top": T(2),
                "right": T(3),
                "bottom": T(4),
            }
            with self.subTest(clsname):
                cls = getattr(nvmanual.argus, clsname)
                rect = cls(**values)
                for k, v in values.items():
                    self.assertEqual(getattr(rect, k), v)


class TestRGBTuple(unittest.TestCase):
    TYPES = (float, int)
    TEMPLATE_MAP = {f"{t.__name__.capitalize()}RGBTuple": t for t in TYPES}

    def test_rgb_tuple(self):
        for clsname, T in self.TEMPLATE_MAP.items():
            values = {
                "r": T(1),
                "g": T(2),
                "b": T(3),
            }
            with self.subTest(clsname):
                cls = getattr(nvmanual.argus, clsname)
                rgb_tuple = cls(**values)
                for k, v in values.items():
                    self.assertEqual(getattr(rgb_tuple, k), v)


class TestBayerTuple(unittest.TestCase):
    TYPES = (float, int)
    TEMPLATE_MAP = {f"{t.__name__.capitalize()}BayerTuple": t for t in TYPES}

    def test_rgb_tuple(self):
        for clsname, T in self.TEMPLATE_MAP.items():
            values = {
                "r": T(1),
                "g_even": T(2),
                "g_odd": T(3),
                "b": T(4),
            }
            with self.subTest(clsname):
                cls = getattr(nvmanual.argus, clsname)
                bayer_tuple = cls(**values)
                for k, v in values.items():
                    self.assertEqual(getattr(bayer_tuple, k), v)


class TestArray2D(unittest.TestCase):
    SUBTYPES = (nvmanual.argus.FloatBayerTupleArray2D,)

    def test_from_xy(self):
        # also tests __len__
        sz = 16
        for t in self.SUBTYPES:
            with self.subTest(t.__class__.__name__):
                i = t.from_xy(x=sz, y=sz)
                self.assertIsInstance(i, t)
                self.assertEqual(sz * sz, len(i))

    def test_at(self):
        sz = 16
        for t in self.SUBTYPES:
            with self.subTest(t.__class__.__name__):
                i = t.from_xy(x=sz, y=sz)
                e = i.at(x=0, y=0)
                e = i.at(x=sz - 1, y=sz - 1)
                with self.assertRaises(IndexError):
                    i.at(x=sz, y=0)
                with self.assertRaises(IndexError):
                    i.at(x=0, y=16)

    def test_getitem(self):
        sz = 16
        for t in self.SUBTYPES:
            with self.subTest(t.__class__.__name__):
                i = t.from_xy(x=sz, y=sz)
                e = i[0]
                e = i[sz * sz - 1]
                with self.assertRaises(IndexError):
                    i[sz * sz]

    def test_iter(self):
        sz = 16
        for t in self.SUBTYPES:
            with self.subTest(t.__class__.__name__):
                i = t.from_xy(x=sz, y=sz)
                for e in i:
                    self.assertIsInstance(e, nvmanual.argus.FloatBayerTuple)


if __name__ == "__main__":
    unittest.main()
