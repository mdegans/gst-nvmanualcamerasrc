import os
import sys
import unittest

PYMOD_PATH = os.environ["PYMOD_PATH"]
sys.path.append(PYMOD_PATH)

import nvmanual

class TestRectangle(unittest.TestCase):
    TEMPLATE_MAP = {
        "FloatRectangle": float,
        "IntRectangle": int,
    }

    PROPS = (
        'left',
        'top',
        'right',
        'bottom'
    )

    def test_rectangle_values(self):
        for clsname, T in self.TEMPLATE_MAP.items():
            values = {
                'left': T(1),
                'top': T(2),
                'right': T(3),
                'bottom': T(4),
            }
            with self.subTest(clsname):
                cls = getattr(nvmanual, clsname)
                rect = cls(**values)
                for k, v in values.items():
                    self.assertEqual(getattr(rect, k), v)

