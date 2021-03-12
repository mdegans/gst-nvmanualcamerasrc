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

# fill out our template from the root meson.build and the VERSION source of
# truth

#define PROJ_VER "1.0.0"
#define PROJ_DESCRIPTION "nVidia MANUAL Camera Source"
#define BINARY_PACKAGE "NvMANUALCameraSrc"
#define PROJ_URL "https://github.com/mdegans/gst-nvmanualcamerasrc"
#define LICENSE "Proprietary"

gstnvmanualcamera_config_meson_h = configure_file(
  input: 'gstnvmanualcamera_config_meson.h.in',
  output: 'gstnvmanualcamera_config_meson.h',
  configuration: {
    'proj_ver': meson.project_version(),
    'proj_description': proj_description,
    'binary_package': binary_package,
    'proj_url': proj_url,
    'license': meson.project_license()[0],
  },
)

# this tells our config.h to use our completed template
gstnvmanualcamerasrc_cflags = ['-DMESON']
gstnvmanualcamerasrc_public_cflags = []

# a variable pointing to this dir
gstnvmanualcamerasrc_incdir = include_directories('.')
gstnvmanualcamerasrc_public_incdir = include_directories('public')
gstnvmanualcamerasrc_incdirs = [
  gstnvmanualcamerasrc_incdir,
  gstnvmanualcamerasrc_public_incdir,
]

# headers to install
subdir('public')