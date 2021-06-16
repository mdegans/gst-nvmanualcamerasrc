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

test_envs = [
  'GST_DEBUG=4',
  'GST_PLUGIN_PATH=' + libgstnvmanualcamerasrc_dir,
  'GST_REGISTRY_UPDATE=yes',
  'G_DEBUG=fatal_warnings',
]

# this is to fix meson/gcc not loading/linking libasan
# clang works, but gcc needs this
# issue now is that the plugin does not load with asan enabled
if cc.get_id() == 'gcc' and get_option('b_sanitize') == 'address'
  test_envs += 'LD_PRELOAD=/usr/lib/gcc/aarch64-linux-gnu/7/libasan.so'
endif

subdir('googletest')
subdir('gst-check')
subdir('exitcode')
subdir('gst-launch')
