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

# this meson.build is for gst-check tests (mostly integration tests)

# a list of dicts describing tests
tests = [
  # {
  #   'description': 'test metadata',
  #   'filename': 'test_metadata',
  #   'sources': ['test_metadata.cpp'],
  #   'suites': ['fast', 'integration'],
  # },
]

# check for check (outside the loop to avoid printing twice)
gst_check_dep = dependency('gstreamer-check-1.0')

# build and run tests (on ninja test)
foreach t: tests
  exe = executable(t['filename'], t['sources'],
    dependencies: [gst_check_dep],
    link_with: libgstnvmanualcamerameta,
    include_directories: [gstnvmanualcamerasrc_incdirs, mmapi_incdirs],
  )
  test(t['description'], exe,
    is_parallel: false,
    suite: t['suites'],
    env: test_envs,
  )
endforeach