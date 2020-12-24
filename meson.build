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

# setup the project's global options
project('gstnvarguscamerasrc', ['cpp'],
  version: run_command(
    'head', '-n', '1', files('VERSION'),
  ).stdout().strip(),
  meson_version: '>= 0.56.0',
  license: 'Proprietary',
  default_options: [
    'cpp_std=c++17',
    'warning_level=1',  # -Wall
    'werror=true',  # -Werror
  ],
)

# the c++ compiler object
cc = meson.get_compiler('cpp')

# split the project version
proj_ver = meson.project_version().split('.')
# version minus patch revision
proj_ver_short = proj_ver[0] + '.' + proj_ver[1]
proj_ver_major = proj_ver[0]
proj_ver_minor = proj_ver[1]
proj_ver_patch = proj_ver[2]
# all of this is used by configure_file on gstnvarguscamera_config_meson.h.in
proj_description = 'nVidia ARGUS Camera Source (meson)'
proj_url = 'https://github.com/mdegans/gst-nvarguscamerasrc'
binary_package = 'NvARGUSCameraSrc'
origin = 'github'
# change this if you fork this

# extra cflags:
# add_project_arguments(
#   '-fno-exceptions',
#   language : 'cpp'
# )

subdir('include')
subdir('src')
