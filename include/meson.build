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
#define PROJ_DESCRIPTION "nVidia ARGUS Camera Source"
#define BINARY_PACKAGE "NvARGUSCameraSrc"
#define PROJ_URL "https://github.com/mdegans/gst-nvarguscamerasrc"
#define LICENSE "Proprietary"

gstnvarguscamera_config_meson_h = configure_file(
  input: 'gstnvarguscamera_config_meson.h.in',
  output: 'gstnvarguscamera_config_meson.h',
  configuration: {
    'proj_ver': meson.project_version(),
    'proj_description': proj_description,
    'binary_package': binary_package,
    'proj_url': proj_url,
    'license': meson.project_license()[0],
  },
)

# this tells our config.h to use our completed template
gstnvarguscamerasrc_cflags = ['-DMESON']

# a variable pointing to this dir
gstnvarguscamerasrc_incdir = include_directories('.')