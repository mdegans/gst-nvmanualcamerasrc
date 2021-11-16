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

        # Capture ID is like the frame number but not quite. It starts count at
        # 1 and is always positive. Internally it's a u32
        assert type(meta.capture_id) is int
        print(f"capture id: {meta.capture_id}")

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

        # these are the raw sharpness values. Prefer using or modifying
        # sharpness_score as operating on this from python is slow because
        # python objects are being constantly created.
        s_values = meta.sharpness_values()
        print(f"sharpness values length: {len(s_values)}")
        assert type(s_values) is nvmanual.argus.FloatBayerTupleArray2D
        for v in s_values:
            assert type(v) is nvmanual.argus.FloatBayerTuple
        assert type(s_values.at(x=63, y=63)) is nvmanual.argus.FloatBayerTuple
        try:
            s_values.at(64, 0)
        except IndexError as e:
            print("sharpness values x index check ok")
            pass
        try:
            s_values.at(0, 64)
        except IndexError as e:
            print("sharpness values y index check ok")
            pass

        # These are tonemap curves. They're not currently available and this
        # is a bug. Whose fault that is, IDK yet.
        t_curve = meta.tonemap_curves()
        # FIXME(mdegans): not very important for client's purpose but in the
        # future it might be.
        # assert t_curve is not None
        print(f"tonemap curves: {t_curve}")

        assert type(meta.ae_locked) is bool

        # see fixme in pymod.cpp
        # assert type(meta.ae_state) is nvmanual.argus.AeState

        assert type(meta.awb_cct) is int
        print(f"AWB color temperature: {meta.awb_cct}")

        awb_gains = meta.awb_gains
        assert type(awb_gains) is nvmanual.argus.FloatBayerTuple
        print(f"AWB gains: ({awb_gains.r}, {awb_gains.g_even}, {awb_gains.g_odd}, {awb_gains.b})")

        awb_estimate = meta.awb_estimate()
        assert type(awb_estimate) is list
        assert type(awb_estimate[0]) is float
        print(f"AWB estimate: {awb_estimate}")

        ccx = meta.color_correction_matrix()
        print(f"color correction matrix: {ccx}")

        assert type(meta.color_correction_matrix_enable) is bool
        print(f"color correction matrix enabled: {meta.color_correction_matrix_enable}")

        assert type(meta.saturation) is float
        print(f"saturation: {meta.saturation}")

        # this is not applicable to most setups
        assert type(meta.focuser_position) is int
        print(f"focuser position: {meta.focuser_position}")

        assert type(meta.frame_duration) is int
        print(f"frame duration: {meta.frame_duration}")

        assert type(meta.frame_readout_time) is int
        print(f"frame readout time: {meta.frame_readout_time}")

        assert type(meta.isp_digital_gain) is float
        print(f"ISP digital gain: {meta.isp_digital_gain}")

        assert type(meta.sensor_analog_gain) is float
        print(f"sensor analog gain: {meta.sensor_analog_gain}")

        assert type(meta.sensor_exposure_time) is int
        print(f"sensor exposure time: {meta.sensor_exposure_time}")

        assert type(meta.sensor_sensitivity) is int
        print(f"ISO value: {meta.sensor_sensitivity}")

        assert type(meta.sensor_timestamp) is int
        print(f"sensor timestamp: {meta.sensor_timestamp}")
    except Exception as e:
        # This is always printing nothing. Not sure why. Prolly did a dumb or
        # printing can't work in this callback.
        print(e, file=sys.stderr, flush=True)
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