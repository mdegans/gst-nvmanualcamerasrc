#include <unistd.h>

#include <initializer_list>

#include <gst/gst.h>

#include "gst/gstclock.h"
#include "gst/gstelement.h"
#include "metadata.hpp"

GST_DEBUG_CATEGORY_STATIC(test_cat);
#define GST_CAT_DEFAULT test_cat

bool check_for_errors(GstBus* bus) {
  while (gst_bus_have_pending(bus)) {
    g_autoptr(GstMessage) msg = nullptr;
    msg = gst_bus_pop(bus);
    if (msg->type == GST_MESSAGE_ERROR) {
      g_autoptr(GError) err = nullptr;
      gst_message_parse_error(msg, &err, nullptr);
      GST_ERROR("%s", err->message);
      return false;
    }
  }
  return true;
}

int main(int argc, char** argv) {
  gst_init(&argc, &argv);
  GST_DEBUG_CATEGORY_INIT(test_cat, TEST_NAME, 0, TEST_NAME);

  g_autoptr(GstElement) pipe = nullptr;
  GstElement* camera = nullptr;
  GstElement* fakesink = nullptr;
  g_autoptr(GstBus) bus = nullptr;
  int retcode = 0;

  pipe = gst_pipeline_new("pipe");
  g_assert(pipe);
  camera = gst_element_factory_make("nvmanualcamerasrc", "camera");
  g_assert(camera);
  // change to "autovideosink" if you want to see the camera feed while testing.
  fakesink = gst_element_factory_make("fakesink", "fakesink");
  g_assert(fakesink);

  for (auto e : {camera, fakesink}) {
    g_assert(gst_bin_add(GST_BIN(pipe), e));
  }

  g_assert(gst_element_link(camera, fakesink));

  // we test with all our properties on
  g_object_set(camera, "bayer-sharpness-map", true, nullptr);
  g_object_set(camera, "metadata", true, nullptr);

  bus = gst_element_get_bus(pipe);
  g_assert(bus);

  for (int i = 10; i > 0; --i) {
    GST_INFO("Iteration %d", i);
    // switch to playing state
    g_assert(GST_STATE_CHANGE_FAILURE !=
             gst_element_set_state(pipe, GST_STATE_PLAYING));
    if (!check_for_errors(bus)) {
      retcode = -2;
      break;
    }
    // check we got there (state change to playing can return ASYNC)
    g_assert(
        GST_STATE_CHANGE_FAILURE !=
        gst_element_get_state(pipe, nullptr, nullptr, GST_CLOCK_TIME_NONE));
    if (!check_for_errors(bus)) {
      retcode = -3;
      break;
    }
    // switch back to READY
    g_assert(GST_STATE_CHANGE_FAILURE !=
             gst_element_set_state(pipe, GST_STATE_READY));
    if (!check_for_errors(bus)) {
      retcode = -4;
      break;
    }
  }

  // cleanup
  g_assert(GST_STATE_CHANGE_FAILURE !=
           gst_element_set_state(pipe, GST_STATE_NULL));

  return retcode;
}