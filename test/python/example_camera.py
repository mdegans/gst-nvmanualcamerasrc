"""
This module contains an example `Camera` class for capturing images and metadata
"""
import contextlib
from typing import (
    Tuple,
    Iterator,
)

import numpy as np
import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstVideo, GstApp

import nvmanual



class CameraError(Exception):
    """camera goofed"""


class Camera():
    """
    Example camera class using `nvmanualcam` and Gstreamer.

    Use it as a context manager and the `capture` method returns a tuple of
    np.ndarray and nvmanual.Metadata

    Raises:
        CameraError: if something goes wrong

    Example Usage:

    >>> with Camera() as cam:
    ...     with cam.capture() as (img, meta):
    ...         # Do not use `img` outside this context without making a copy
    ...         # it is mapped to a Gst.Buffer's data.
    ...         img.shape
    ...         type(meta)
    (3040, 4032, 4)
    <class 'nvmanual.Metadata'>

    Known Issues:
        * The metadata and captured img **may** be off by a frame due to a race
          condition. There are multiple appsinks here but no syncronization
          guarantess between them. The pipeline design may change in the
          future to fix this.
    """

    PIPELINE = (
        # the leaky and buffer limits here is (hopefully) to reduce latency
        # as there isn't really any guarantee that the metadata will match
        # the frame exactly (race condition). Might be a frame off. It may
        # or may not be a problem depending on your usage. `Metadata` currently
        # does not survive a `nvvidconv` (or any buffer conversion). That
        # requires a GType and that is stupid levels of pointless boilerplate.
        # A better solution might be to just return an unconverted buffer and
        # convert it using the NvBuffer API after it leaves GStreamer (after
        # removing metadata).
        'nvmanualcamerasrc metadata=true bayer-sharpness-map=true gain=4'
        ' ! tee name=t '
        ' t. ! queue max-size-buffers=1 leaky=2'
        ' ! appsink async=false sync=false max-buffers=1 drop=true name=metasink'
        ' t. ! queue max-size-buffers=1 leaky=2'
        ' ! autovideosink'
        ' t. ! queue max-size-buffers=1 leaky=2'
        ' ! nvvidconv'
        ' ! appsink async=false sync=false max-buffers=1 drop=true caps=video/x-raw,format=RGBA name=pixelsink'
    )

    def __init__(self):
        Gst.init(None)
        self.pipe = Gst.parse_launch(Camera.PIPELINE)
        self.metasink = self.pipe.get_by_name("metasink")  # type: GstApp.AppSink
        self.pixelsink = self.pipe.get_by_name("pixelsink")  # type: GstApp.AppSink

    def __enter__(self) -> "Camera":
        # try to go to the playign state
        ret = self.pipe.set_state(Gst.State.PLAYING)
        # if we failed
        if ret == Gst.StateChangeReturn.FAILURE:
            raise CameraError("Could not go to playing state.")
        # if it's going to PLAYING but not ready yet
        elif ret == Gst.StateChangeReturn.ASYNC:
            # wait for ready
            _ = self.pipe.get_state(10)
        return self

    def __exit__(self, *_):
        self.pipe.set_state(Gst.State.NULL)

    @contextlib.contextmanager
    def capture(self, just_meta=False) -> Iterator[Tuple[np.ndarray, nvmanual.Metadata]]:
        # get metadata
        meta = nvmanual.Metadata.from_buffer(
            self.metasink.pull_sample().get_buffer())

        if just_meta:
            yield None, meta
            return

        # get pixel data
        # There might be a better way to do this, but getting n_components
        # from a caps structure is more trouble than it's worth and a read-only
        # map is low cost. a VideoFrame has what we need.
        sample = self.pixelsink.pull_sample()
        caps = sample.get_caps()
        if not caps:
            raise CameraError(f"Could not get Gst.Caps from sample: {sample}")
        buf = sample.get_buffer()
        if not buf:
            raise CameraError(f"Could not get Gst.Buffer from sample: {sample}")
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            raise CameraError("Could not map Gst.Buffer")
        info = GstVideo.VideoInfo.new()
        if not info.from_caps(caps):
            raise CameraError(f"Could not get info from caps: {caps}")
        frame = GstVideo.VideoFrame()
        if not frame.map(info, buf, Gst.MapFlags.READ):
            raise CameraError("Frame could not be mapped.")

        # Yes, numpy warns not to use this constructor but there isn't a lot of
        # choice here.  https://stackoverflow.com/a/58806157
        arr = np.ndarray(
            shape=(
                frame.info.height,
                frame.info.width,
                frame.info.finfo.n_components,
            ),
            dtype=np.uint8,
            buffer=map_info.data,
        )

        # If np.ndarray could specify a `free` callback, we could avoid this
        # context manager, but since we can't (cleanly), we have to yield it...
        yield arr, meta

        # ... then free it when control returns here. Coroutines rock!
        frame.unmap()
        buf.unmap(map_info)


if __name__ == "__main__":
    import doctest
    doctest.testmod()
