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
