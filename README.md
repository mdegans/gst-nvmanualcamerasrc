# gst-nvarguscamerasrc

Steps to compile the "gst-nvarguscamera" sources natively:

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

Run the following commands to build and install "libgstnvarguscamerasrc.so":

using Gnu Make (`make`)

```
make
(sudo) make install
```
(or)
```
DEST_DIR=<dir> (sudo) make install
```

using Meson and Ninja

```
meson builddir
cd builddir
ninja
(sudo) ninja install
```

  Note: "sudo make install" will copy library "libgstnvarguscamerasrc.so"
  into "/usr/lib/aarch64-linux-gnu/gstreamer-1.0" directory.
