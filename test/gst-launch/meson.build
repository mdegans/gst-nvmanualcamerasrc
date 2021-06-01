# this tests the element's use with gst-launch-1.0 to help diagnose #12

gst_launch = find_program('gst-launch-1.0')

test('gstlaunch test', gst_launch,
  args: ['nvmanualcamerasrc', 'num-buffers=10', '!', 'autovideosink'],
  protocol: 'exitcode',
  is_parallel: false,
  suite: 'integration',
  timeout: 20,  # seconds
  env: [
    'GST_DEBUG=4',
    'GST_PLUGIN_PATH=' + libgstnvmanualcamerasrc_dir,
    'GST_REGISTRY_UPDATE=yes',
  ],
)

# note that it's also possible to wrap any test:
# https://stackoverflow.com/questions/57734973/how-to-use-valgrind-with-meson
valgrind = find_program('valgrind', required: false)
if valgrind.found()
  test('valgrind short', valgrind,
    args: [
      '--leak-check=full',
      '--show-reachable=yes',
      '--track-origins=yes',
      '--error-exitcode=1',
      '/usr/bin/gst-launch-1.0',
      'nvmanualcamerasrc',
      'num-buffers=10',
      'metadata=true',
      'bayer-sharpness-map=true',
      '!', 'fakesink',
    ],
    protocol: 'exitcode',
    is_parallel: false,
    suite: ['memory'],
    timeout: 60,  # seconds
    env: [
      'GST_DEBUG=4',
      'GST_PLUGIN_PATH=' + libgstnvmanualcamerasrc_dir,
      'GST_REGISTRY_UPDATE=yes',
      'G_DEBUG=gc-friendly',
      'G_SLICE=always-malloc',
    ],
  )
  test('valgrind short', valgrind,
    args: [
      '--leak-check=full',
      '--show-reachable=yes',
      '--track-origins=yes',
      '--error-exitcode=1',
      '/usr/bin/gst-launch-1.0',
      'nvmanualcamerasrc',
      'num-buffers=10000',
      'metadata=true',
      'bayer-sharpness-map=true',
      '!', 'fakesink',
    ],
    protocol: 'exitcode',
    is_parallel: false,
    suite: ['stress', 'memory'],
    timeout: 60000,  # seconds
    env: [
      'GST_DEBUG=4',
      'GST_PLUGIN_PATH=' + libgstnvmanualcamerasrc_dir,
      'GST_REGISTRY_UPDATE=yes',
      'G_DEBUG=gc-friendly',
      'G_SLICE=always-malloc',
    ],
  )
endif