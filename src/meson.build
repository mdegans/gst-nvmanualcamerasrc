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
gstnvmanualcamerasrc_deps = [
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

# our metadata source
gstnvmanualcamera_metadata_src = [
  'metadata.cpp',
]

# metadata cflags
gstnvmanualcamera_metadata_cflags = []

# metadata deps
boost_json_dep = dependency('boost', modules: ['json'], required: false)
if boost_json_dep.found()
  gstnvmanualcamera_metadata_cflags += '-DHAS_BOOST_JSON'
endif
if gst_dep.found()
  gstnvmanualcamera_metadata_cflags += '-DHAS_GSTREAMER'
endif
gstnvmanualcamera_metadata_deps = [
  cc.find_library('nvargus', dirs: lib_install_dir),
  gst_dep,
  boost_json_dep,
]

# useful to point to the folder including the plugin
libgstnvmanualcamerasrc_dir = meson.current_build_dir()

# variables for use with dependency.get_variable(..)
gstnvmanualcamerameta_variables = [
  'pluginsdir_uninstalled=' + libgstnvmanualcamerasrc_dir,
  'pluginsdir=' + gst_install_dir,
]

# metadata library target
libgstnvmanualcamerameta = library('gstnvmanualcamerameta',
  gstnvmanualcamera_metadata_src,
  dependencies: gstnvmanualcamera_metadata_deps,
  include_directories: [gstnvmanualcamerasrc_incdirs, mmapi_incdirs],
  install: true,
  version: meson.project_version(),
  cpp_args: gstnvmanualcamera_metadata_cflags,
)

# a metadata dependency for use in subprojects
gstnvmanualcamerameta_dep = declare_dependency(
  compile_args: gstnvmanualcamera_metadata_cflags,
  dependencies: gstnvmanualcamera_metadata_deps,
  include_directories: [gstnvmanualcamerasrc_incdirs, mmapi_incdirs],
  link_with: libgstnvmanualcamerameta,
  version: meson.project_version(),
  variables: gstnvmanualcamerameta_variables,
)

# a pkg-config file to find the metadata library
pkg = import('pkgconfig')
pkg.generate(libgstnvmanualcamerameta,
  description: 'Metadata library for `nvmanualcamerasrc` Metadata.',
  extra_cflags: gstnvmanualcamera_metadata_cflags,
  libraries: gstnvmanualcamera_metadata_deps,
  subdirs: meson.project_name(),  # $PREFIX/include/$PROJECT_NAME
  variables: gstnvmanualcamerameta_variables,
)

# a list of source files (globbing is verboten!)
gstnvmanualcamerasrc_src = [
  'gstnvmanualcamera_utils.cpp',
  'gstnvmanualcamerasrc.cpp',
  'stoppable_thread.cpp',
  'consumer.cpp',
  'producer.cpp',
  'plugin.cpp',
]

# set this when you `meson configure` with `-D b_lundef=true`
# ldflags = ['-Wl,--no-undefined']

# our library target
libgstnvmanualcamerasrc = library(meson.project_name(), gstnvmanualcamerasrc_src,
  dependencies: gstnvmanualcamerasrc_deps,
  link_with: libgstnvmanualcamerameta,
  include_directories: [gstnvmanualcamerasrc_incdirs, mmapi_incdirs],
  install: true,
  install_dir: gst_install_dir,
  cpp_args: [gstnvmanualcamerasrc_cflags, gstnvmanualcamera_metadata_cflags],
)
