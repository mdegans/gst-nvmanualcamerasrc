// this test exist just to test whether the element works in a string based
// pipeline because of issue #12

// It really shouldn't matter but somehow it appears there's a slight chance it
// might.

#include <gst/gst.h>

GST_DEBUG_CATEGORY_STATIC(test_cat);
#define GST_CAT_DEFAULT test_cat

int main(int argc, char** argv) {
  GST_DEBUG_CATEGORY_INIT(test_cat, TEST_NAME, 0, TEST_NAME);

  g_autoptr(GError) err = nullptr;
  g_autoptr(GstElement) pipe = nullptr;
  g_autoptr(GstBus) bus = nullptr;
  g_autoptr(GstMessage) msg = nullptr;

  // parse command line
  gst_init(&argc, &argv);

  pipe = gst_parse_launch(
    "nvmanualcamerasrc num-buffers=10 ! autovideosink", &err);
  if (err) {
    GST_ERROR("Error creating pipeline: %s", err->message);
  }

  g_assert(GST_STATE_CHANGE_FAILURE !=
           gst_element_set_state(pipe, GST_STATE_PLAYING));
  bus = gst_element_get_bus(pipe);
  g_assert(bus);
  msg = gst_bus_timed_pop_filtered(
      bus, GST_CLOCK_TIME_NONE,
      (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  g_assert(msg->type == GST_MESSAGE_EOS);

  g_assert(GST_STATE_CHANGE_FAILURE !=
           gst_element_set_state(pipe, GST_STATE_NULL));
  return 0;
}