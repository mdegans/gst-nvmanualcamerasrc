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