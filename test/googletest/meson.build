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


# this meson.build is for googletest tests (mostly unit tests)

# a list of dicts describing tests
tests = [
  # {
  #   'description': 'test camera',
  #   'filename': 'test_camera_fast',
  #   'sources': ['test_camera_fast.cpp'],
  #   'suites': ['fast', 'unit'],
  # },
  # {
  #   'description': 'test settings',
  #   'filename': 'test_settings',
  #   'sources': ['test_settings.cpp'],
  #   'suites': ['fast', 'unit'],
  # },
  # {
  #   'description': 'test config',
  #   'filename': 'test_config',
  #   'sources': ['test_config.cpp'],
  #   'suites': ['fast', 'unit'],
  # },
  # {
  #   'description': 'test utils_glib',
  #   'filename': 'test_utils_glib',
  #   'sources': ['test_utils_glib.cpp'],
  #   'suites': ['fast', 'unit'],
  # },
]

# check for check (outside the loop to avoid printing twice)
# googletest subproject stuff
gtest_proj = subproject('gtest')
gtest_dep = gtest_proj.get_variable('gtest_dep')

# build and run tests (on ninja test)
foreach t: tests
  exe = executable(t['filename'], t['sources'],
    dependencies: [gtest_dep],
    link_with: [libgstnvmanualcamerasrc, libgstnvmanualcamerameta],
    include_directories: [libgstnvmanualcamerasrc_incdir, mmapi_incdirs],
  )
  # skip running test if it's an interactive one
  if 'interactive' in t['suites']
    continue
  endif
  # run the rest of the tests in parallel
  test(t['description'], exe,
    protocol: 'gtest',
    is_parallel: true,
    suite: t['suites'],
    env: [
      'GST_DEBUG=4',
      'GST_PLUGIN_PATH=' + meson.current_build_dir() + '/../../src',
      'GST_REGISTRY_UPDATE=yes',
    ],
  )
endforeach