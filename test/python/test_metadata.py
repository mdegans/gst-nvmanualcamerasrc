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
    try:
        # To get Metadata in python, use the `from_buffer` method and supply
        # it a Gst.Buffer. Using a probe is just one way of doing this. There is
        # possibility `from_buffer` can return None if no metadata is available
        # or something goes wrong.
        meta = nvmanual.Metadata.from_buffer(info.get_buffer())
        assert meta is not None

        # a Bayer histogram is normally available. This is a 256 element list
        # where a bayer value is the index and the returning tuple contains
        # counts for that value.
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

        # a RGB histogram is normally available. This is a 256 element list
        # where a RGB value is the index and the returning tuple contains
        # counts for that value.
        rgb_hist = meta.rgb_histogram()
        assert rgb_hist is not None
        assert type(rgb_hist) is list
        assert len(rgb_hist) == 256
        rgb_tuple = rgb_hist[0]
        assert type(rgb_tuple.r) is int
        assert type(rgb_tuple.g) is int
        assert type(rgb_tuple.b) is int
        print(f"got rgb histogram of length: {len(rgb_hist)}")

        # This is very approximate scene lux. It's useful mostly to cheaply find
        # the brightest frame (eg, the one where flash is brightest).
        lux = meta.scene_lux
        assert type(lux) is float
        print(f"scene lux: {lux}")

        # This calculates the sharpness score from an internal array of
        # sharpness values. This is very cheap since the GPU/ISP has already
        # done the hard work.
        sharpness = meta.sharpness_score()
        assert type(sharpness) is float
        print(f"scene sharpness: {sharpness}")

        # This does the same as the above, only within a ROI. It's important to
        # note that the internal sharpness map is 64x64 always so a very small
        # ROI will not work as expected.
        roi = nvmanual.argus.FloatRectangle(0.4, 0.4, 0.6, 0.6)
        roi_sharpness = meta.sharpness_score(roi)
        assert type(roi_sharpness) is float
        print(f"roi sharpness: {roi_sharpness}")

        # These are tonemap curves. They're not currently available and this
        # is a bug. Whose fault that is, IDK yet.
        t_curve = meta.tonemap_curves()
        # FIXME(mdegans): not very important for client's purpose but in the
        # future it might be.
        # assert t_curve is not None
        print(f"tonemap curves: {t_curve}")

        # ccx = meta.color_correction_matrix()
        # assert ccx is not None
    except Exception as e:
        print(e, flush=True)
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