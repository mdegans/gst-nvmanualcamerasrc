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
project('gstnvmanualcamerasrc', ['cpp'],
  version: run_command(
    'head', '-n', '1', files('VERSION'),
  ).stdout().strip(),
  meson_version: '>= 0.57.1',
  license: 'Proprietary',
  default_options: [
    'cpp_std=c++17',
    'warning_level=2',  # -Wall, -Wextra
    'werror=true',  # -Werror
  ],
)

# the c++ compiler object
cc = meson.get_compiler('cpp')

# get argus version (actually detect the JetPack version, but that ends up
# being more useful). It's used to define 'JETPACK_45'.
pymod = import('python')
python3 = pymod.find_installation('python3')
argus_version = run_command(
  python3, '-sSE', files('argus_version.py')).stdout().strip()
has_jetpack_45 = false
if argus_version == 'None'
  error('Argus not found')
else
  if argus_version.version_compare('>=32.5')
    message('JetPack 4.5 Detected. Enabling extra Metadata.')
    has_jetpack_45 = true
  endif
endif

# split the project version
proj_ver = meson.project_version().split('.')
# version minus patch revision
proj_ver_short = proj_ver[0] + '.' + proj_ver[1]
proj_ver_major = proj_ver[0]
proj_ver_minor = proj_ver[1]
proj_ver_patch = proj_ver[2]
# all of this is used by configure_file on gstnvmanualcamera_config_meson.h.in
proj_description = 'nVidia MANUAL Camera Source (meson)'
proj_url = 'https://github.com/mdegans/gst-nvmanualcamerasrc'
binary_package = 'NvMANUALCameraSrc'
origin = 'github'
# change this if you fork this

# extra cflags:
# add_project_arguments(
#   '-fno-exceptions',
#   language : 'cpp'
# )

subdir('include')
subdir('src')
subdir('test')

