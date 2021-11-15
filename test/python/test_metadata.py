# this should mirror the logic of test_metadata.cpp
import os
import sys

from typing import Optional, Any

import gi
gi.require_version("Gst", "1.0")

from gi.repository import Gst

PYMOD_PATH = os.environ["PYMOD_PATH"]
sys.path.append(PYMOD_PATH)

import nvmanual

def metadata_probe(pad: Gst.Pad, info: Gst.PadProbeInfo, data: Any):
    # todo: comprehensive checks of each
    meta = nvmanual.Metadata.from_buffer(info.get_buffer())
    assert meta is not None

    # b_hist = meta.bayer_histogram()
    # assert b_hist is not None

    # rgb_hist = meta.rgb_histogram()
    # assert rgb_hist is not None

    lux = meta.scene_lux()
    assert type(lux) is float
    print(f"scene lux: {lux}")

    # s_values = meta.sharpness_values()
    # assert s_values is not None

    # t_curve = meta.tonemap_curve()
    # assert t_curve is not None

    # ccx = meta.color_correction_matrix()
    # assert ccx is not None

    return Gst.PadProbeReturn.OK


def main():
    Gst.init(None)

    pipe = Gst.Pipeline.new()
    try:
        assert pipe is not None

        camera = Gst.ElementFactory.make("nvmanualcamerasrc", "camera")
        assert camera is not None

        fakesink = Gst.ElementFactory.make("fakesink", "fakesink")
        assert fakesink is not None

        assert pipe.add(camera)
        assert pipe.add(fakesink)

        assert camera.link(fakesink)

        camera.set_property("bayer-sharpness-map", True)
        camera.set_property("metadata", True)
        camera.set_property("num-buffers", 10)

        pad = fakesink.get_static_pad("sink")
        assert pad is not None

        pad.add_probe(Gst.PadProbeType.BUFFER, metadata_probe, None)

        assert pipe.set_state(Gst.State.PLAYING) != Gst.StateChangeReturn.FAILURE

        bus = pipe.get_bus()
        assert bus is not None

        msg = bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, (Gst.MessageType.ERROR | Gst.MessageType.EOS))
        assert msg.type == Gst.MessageType.EOS

    finally:
        assert pipe.set_state(Gst.State.NULL) != Gst.StateChangeReturn.FAILURE

    return 0

if __name__ == "__main__":
    import sys
    sys.exit(main())