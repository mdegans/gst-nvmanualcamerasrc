# Copyright (c) 2020 Michael de Gans

# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# check for gstreamer and get our plugin install path
gst_dep = dependency('gstreamer-1.0')
gst_install_dir = gst_dep.get_variable(pkgconfig: 'pluginsdir')
# this is not actually a lib install dir, but i'm using the name from
# the Makefile for consistency
lib_install_dir = '/usr/lib/aarch64-linux-gnu/tegra/'

# mmapi incdirs
mmapi_incdirs = [
  include_directories('/usr/src/jetson_multimedia_api/include/'),
  include_directories('/usr/src/jetson_multimedia_api/argus/samples/utils/'),
]

# a list of our dependencies
gstnvmanualcamearsrc_deps = [
  gst_dep,
  dependency('gstreamer-base-1.0'),
  dependency('gstreamer-video-1.0'),
  dependency('gstreamer-allocators-1.0'),
  # glib-2.0 is implied by above
  cc.find_library('nvbuf_utils', dirs: lib_install_dir),
  cc.find_library('nvdsbufferpool', dirs: lib_install_dir),
  cc.find_library('nvargus', dirs: lib_install_dir),
  # https://mesonbuild.com/howtox.html
  # "Lots of people seem to do this manually with find_library('pthread') or 
  # something similar. Do not do that. It is not portable. Instead do this."
  dependency('threads'),
]

# a list of source files (globbing is verboten!)
gstnvmanualcameasrc_src = [
  'gstnvmanualcamera_utils.cpp',
  'gstnvmanualcamerasrc.cpp',
  'plugin.cpp',
]

# set this when you `meson configure` with `-D b_lundef=true`
# ldflags = ['-Wl,--no-undefined']

# our library target
libgstnvmanualcameasrc = library(meson.project_name(), gstnvmanualcameasrc_src,
  dependencies: gstnvmanualcamearsrc_deps,
  include_directories: [
    gstnvmanualcamerasrc_incdir,
    mmapi_incdirs,
  ],
  install: true,
  install_dir: gst_install_dir,
  cpp_args: gstnvmanualcamerasrc_cflags,
#  link_args: ldflags,
)