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

# switched to 1 on error
retcode = 0


def metadata_probe(pad: Gst.Pad, info: Gst.PadProbeInfo, data: Any):
    # todo: comprehensive checks of each
    try:
        meta = nvmanual.Metadata.from_buffer(info.get_buffer())
        assert meta is not None

        b_hist = meta.bayer_histogram()
        assert b_hist is not None
        assert type(b_hist) is list
        assert len(b_hist) is 256
        bayer_tuple = b_hist[0]
        assert type(bayer_tuple.r) is int
        assert type(bayer_tuple.g_even) is int
        assert type(bayer_tuple.g_odd) is int
        assert type(bayer_tuple.b) is int
        print(f"got bayer histogram of length: {len(b_hist)}")

        rgb_hist = meta.rgb_histogram()
        assert rgb_hist is not None
        assert type(rgb_hist) is list
        assert len(rgb_hist) == 256
        rgb_tuple = rgb_hist[0]
        assert type(rgb_tuple.r) is int
        assert type(rgb_tuple.g) is int
        assert type(rgb_tuple.b) is int
        print(f"got rgb histogram of length: {len(rgb_hist)}")

        lux = meta.scene_lux
        assert type(lux) is float
        print(f"scene lux: {lux}")

        # sharpness using the default ROI (entire image)
        sharpness = meta.sharpness_score()
        assert type(sharpness) is float
        print(f"scene sharpness: {sharpness}")

        # sharpness using just a ROI crop
        roi = nvmanual.argus.FloatRectangle(0.4, 0.4, 0.6, 0.6)
        roi_sharpness = meta.sharpness_score(roi)
        assert type(roi_sharpness) is float
        print(f"roi sharpness: {roi_sharpness}")

        # t_curve = meta.tonemap_curve()
        # assert t_curve is not None

        # ccx = meta.color_correction_matrix()
        # assert ccx is not None
    except AssertionError as e:
        print(e)
        retcode = 1

    return Gst.PadProbeReturn.OK


def main():
    Gst.init(None)

    global retcode

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

    except AssertionError:
        retcode = 1

    finally:
        assert pipe.set_state(Gst.State.NULL) != Gst.StateChangeReturn.FAILURE

    return retcode

if __name__ == "__main__":
    import sys
    sys.exit(main())