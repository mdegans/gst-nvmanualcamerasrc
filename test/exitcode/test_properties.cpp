#include "metadata.hpp"

#include <gst/gst.h>

#ifdef HAS_BOOST_JSON
#include <boost/json/src.hpp>
#endif

#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>

GST_DEBUG_CATEGORY_STATIC(test_cat);
#define GST_CAT_DEFAULT test_cat

bool check_property_equal(GstElement* e, const gchar* propname, int value) {
  int actual;
  g_object_get(G_OBJECT(e), propname, &actual, nullptr);
  return value == actual;
}

bool check_property_equal(GstElement* e, const gchar* propname, uint value) {
  uint actual;
  g_object_get(G_OBJECT(e), propname, &actual, nullptr);
  return value == actual;
}

bool check_property_equal(GstElement* e, const gchar* propname, float value) {
  float actual;
  g_object_get(G_OBJECT(e), propname, &actual, nullptr);
  return value == actual;
}

int main(int argc, char** argv) {
  gst_init(&argc, &argv);
  GST_DEBUG_CATEGORY_INIT(test_cat, TEST_NAME, 0, TEST_NAME);

  g_autoptr(GstElement) pipe = nullptr;
  GstElement* camera = nullptr;
  GstElement* fakesink = nullptr;
  g_autoptr(GstBus) bus = nullptr;
  g_autoptr(GstMessage) msg = nullptr;

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

  // check properties set and get before playback
  g_object_set(G_OBJECT(camera), "aelock", true, nullptr);
  g_object_set(G_OBJECT(camera), "aeantibanding", 1, nullptr);
  g_object_set(G_OBJECT(camera), "awblock", true, nullptr);
  g_object_set(G_OBJECT(camera), "digitalgain", 10.0f, nullptr);
  g_object_set(G_OBJECT(camera), "ee-mode", 2, nullptr);
  g_object_set(G_OBJECT(camera), "ee-strength", 1.0f, nullptr);
  g_object_set(G_OBJECT(camera), "exposurecompensation", 0.1f, nullptr);
  g_object_set(G_OBJECT(camera), "exposuretime", 0.5f, nullptr);
  g_object_set(G_OBJECT(camera), "gain", 2.0f, nullptr);
  g_object_set(G_OBJECT(camera), "saturation", 2.0f, nullptr);
  g_object_set(G_OBJECT(camera), "sensor-id", 0, nullptr);
  g_object_set(G_OBJECT(camera), "sensor-mode", 0, nullptr);
  g_object_set(G_OBJECT(camera), "tnr-mode", 2, nullptr);
  g_object_set(G_OBJECT(camera), "wbmode", 0, nullptr);
  g_object_set(G_OBJECT(camera), "metadata", true, nullptr);
  g_object_set(G_OBJECT(camera), "bayer-sharpness-map", true, nullptr);

  g_object_set(G_OBJECT(camera), "num-buffers", 10, nullptr);

  g_assert(GST_STATE_CHANGE_FAILURE !=
           gst_element_set_state(pipe, GST_STATE_PLAYING));
  // actually wait until the playing state
  g_assert(GST_STATE_CHANGE_FAILURE !=
           gst_element_get_state(camera, nullptr, nullptr, -1));

  // check the propreties are still set after starting playback
  g_assert(check_property_equal(camera, "aelock", true));
  g_assert(check_property_equal(camera, "aeantibanding", 1));
  g_assert(check_property_equal(camera, "awblock", true));
  g_assert(check_property_equal(camera, "digitalgain", 10.0f));
  g_assert(check_property_equal(camera, "ee-mode", 2));
  g_assert(check_property_equal(camera, "ee-strength", 1.0f));
  g_assert(check_property_equal(camera, "exposurecompensation", 0.1f));
  g_assert(check_property_equal(camera, "exposuretime", 0.5f));
  g_assert(check_property_equal(camera, "gain", 2.0f));
  g_assert(check_property_equal(camera, "saturation", 2.0f));
  g_assert(check_property_equal(camera, "sensor-id", 0));
  g_assert(check_property_equal(camera, "sensor-mode", 0));
  g_assert(check_property_equal(camera, "tnr-mode", 2));
  g_assert(check_property_equal(camera, "wbmode", 0));
  g_assert(check_property_equal(camera, "metadata", true));
  g_assert(check_property_equal(camera, "bayer-sharpness-map", true));

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