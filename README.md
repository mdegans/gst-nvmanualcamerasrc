# gst-nvmanualcamerasrc

This is a fork of nvarguscamerasrc. No effort is being made to maintain
compatability with the original source's properties. It's mostly oriented
towards people who want or need full manual control of capture settings.

Changes from `nvarguscamerasrc`:
* **Ranges are not supported**. In development it's been found this is confusing and there was no useful feedback if a set failed (eg. out of range because that varies from sensor to sensor). Also, this is `manual` src because it's assumed you don't want automatic stuff.
* **Exposure time is set in frames** (exposure in nanoseconds is is gettable through `exposurereal` property)
* **Metadata is now attached to the buffer** (sharpness measures, histograms, white balance values, ISO, and more.).
* As many **print statements** as possible have been **converted to standard GST logging macros**. The camera source will (mostly) no longer spam the console. Can't do anything about libargus itself, sorry.

Steps to compile the "gst-nvmanualcamerasrc" sources natively:

## Requirements

1) Install gstreamer related packages on target using the command:
```
sudo apt-get install \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  libegl1-mesa-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer1.0-dev
```

2) Install "jetson_multimedia_api" package from latest Jetpack release.
```
sudo apt-get install nvidia-l4t-jetson-multimedia-api
```

## Building

Run the following commands to build and install `nvmanualcamerasrc`:

using Meson and Ninja

```
meson builddir
cd builddir
ninja
(sudo) ninja install
```

## Python Bindings

`nvmanualcamerasrc` provides Python bindings for Argus metadata. If `python3-dev`
is installed on the system, they will be built and installed automatically.
Tests are included which double as usage examples.

These examples can be found under [`test/python/`](test/python/). All that's
needed to construct metadata is a `Gst.Buffer` with metadata attached by the
camera (`camera.set_property("metadata", True)`).

One way of getting a buffer is a Pad probe Pad probe as shown in the example.
Another is using `appsink` and `sample.get_buffer()` from it's `Gst.Sample`.
Once you have such a `buffer` just:

```python
import nvmanual
...
meta = nvmanual.Metadata.from_buffer(buffer)  # type: Optional[nvmanual.Metadata]
```

NOTE: if `meta` is `None`, `metadata` is probably not set to `True` on the
camera.

The `Metadata` interface is mostly the same as `nvmanualcam::Metadata` in C++
but may change in the future to become more "Pythonic" where appropriate.
Particularly, there are some bound Argus types that might be better as
`np.ndarray`. Also, `from_buffer` might do better as an init method.

For API documentation, use `help(...)` from within Python. `.pyi` stubs and html
docs are not yet provided (but are planned).

If you need to use `nvmanual` in a virtual environment, it can be copied (or
linked) from `/usr/local/lib/python3.6/dist-packages` to wherever you need it
(similar to how one might with `cv2`)
