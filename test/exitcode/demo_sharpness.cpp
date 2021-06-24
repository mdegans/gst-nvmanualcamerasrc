#include "metadata.hpp"

#include <gst/gst.h>

#ifdef HAS_BOOST_JSON
#include <boost/json/src.hpp>
#endif

#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>

// set how many frames you want for the demo here
static const int NUM_FRAMES = 1000;
// the argus sensor mode (0 is usually best)
static const int SENSOR_MODE = 0;

GST_DEBUG_CATEGORY_STATIC(test_cat);
#define GST_CAT_DEFAULT test_cat

static int meta_counter = NUM_FRAMES;

GstPadProbeReturn metadata_probe(GstPad* pad,
                                 GstPadProbeInfo* info,
                                 gpointer user_data) {
  (void)pad;
  (void)user_data;
  auto meta = nvmanualcam::Metadata::steal(info);

  auto score = meta->getSharpnessScore();
  if (score) {
    GST_INFO("Frame %d score:%.3f", meta_counter--, score.value());
  } else {
    GST_ERROR("Shapness score not available.");
    return GST_PAD_PROBE_REMOVE;
  }

  return GST_PAD_PROBE_OK;
}

int main(int argc, char** argv) {
  gst_init(&argc, &argv);
  GST_DEBUG_CATEGORY_INIT(test_cat, TEST_NAME, 0, TEST_NAME);

  g_autoptr(GstElement) pipe = nullptr;
  GstElement* camera = nullptr;
  GstElement* sink = nullptr;
  g_autoptr(GstPad) pad = nullptr;
  g_autoptr(GstBus) bus = nullptr;
  g_autoptr(GstMessage) msg = nullptr;

  pipe = gst_pipeline_new("pipe");
  g_assert(pipe);
  camera = gst_element_factory_make("nvmanualcamerasrc", "camera");
  g_assert(camera);
  sink = gst_element_factory_make("autovideosink", "sink");
  g_assert(sink);

  for (auto e : {camera, sink}) {
    g_assert(gst_bin_add(GST_BIN(pipe), e));
  }

  g_assert(gst_element_link(camera, sink));

  g_object_set(camera, "bayer-sharpness-map", true, nullptr);
  g_object_set(camera, "metadata", true, nullptr);
  g_object_set(camera, "num-buffers", NUM_FRAMES, nullptr);
  g_object_set(camera, "sensor-mode", SENSOR_MODE, nullptr);

  // setup pad probe callback
  pad = gst_element_get_static_pad(sink, "sink");
  g_assert(pad);
  g_assert(gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, metadata_probe,
                             nullptr, nullptr));  // returns nonzero on success

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