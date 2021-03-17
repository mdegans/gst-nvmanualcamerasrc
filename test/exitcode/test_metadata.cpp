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

static int meta_counter = 0;

GstPadProbeReturn metadata_probe(GstPad* pad,
                                 GstPadProbeInfo* info,
                                 gpointer user_data) {
  (void)pad;
  (void)user_data;
  auto metaa = nvmanualcam::Metadata::create(info);
  auto metab = nvmanualcam::Metadata::steal(info);

  if (!metaa) {
    GST_ERROR("no copied metadata found");
    return GST_PAD_PROBE_REMOVE;
  }

  if (!metab) {
    GST_ERROR("no moved metadata found");
    return GST_PAD_PROBE_REMOVE;
  }
  GST_INFO("got metadata from buffer (%d)", ++meta_counter);

  // sanity test for metaa and metab
  assert(metaa->getSceneLux() == metab->getSceneLux());  // a value matches

#ifdef HAS_BOOST_JSON
  auto json = metab->as_json();
  GST_INFO("json: %s", json.c_str());
#endif

  // test gains
  auto gains = metab->getAwbGains();
  GST_INFO("Bayer Gains: r:%.3f,gEven:%.3f,gOdd:%.3f,b:%.3f", gains.r(),
           gains.gEven(), gains.gOdd(), gains.b());
  // test sharpness scores
  auto scores = metab->getSharpnessScore();
  if (scores) {
    std::stringstream ss;
    for (auto val : scores.value()) {
      ss << val << ",";
    }
    GST_INFO("Sharpness scores:%s", ss.str().c_str());
  }
  // test scene lux
  auto lux = metab->getSceneLux();
  GST_INFO("Scene Lux: %.3f", lux);

  return GST_PAD_PROBE_OK;
}

int main(int argc, char** argv) {
  gst_init(&argc, &argv);
  GST_DEBUG_CATEGORY_INIT(test_cat, TEST_NAME, 0, TEST_NAME);

  g_autoptr(GstElement) pipe = nullptr;
  GstElement* camera = nullptr;
  GstElement* fakesink = nullptr;
  g_autoptr(GstPad) pad = nullptr;
  g_autoptr(GMainLoop) main_loop = nullptr;
  g_autoptr(GstBus) bus = nullptr;
  g_autoptr(GstMessage) msg = nullptr;

  pipe = gst_pipeline_new("pipe");
  g_assert(pipe);
  camera = gst_element_factory_make("nvmanualcamerasrc", "camera");
  g_assert(camera);
  fakesink = gst_element_factory_make("fakesink", "fakesink");
  g_assert(fakesink);

  for (auto e : {camera, fakesink}) {
    g_assert(gst_bin_add(GST_BIN(pipe), e));
  }

  g_assert(gst_element_link(camera, fakesink));

  g_object_set(camera, "metadata", true, nullptr);
  g_object_set(camera, "num-buffers", 10, nullptr);

  // setup pad probe callback
  pad = gst_element_get_static_pad(fakesink, "sink");
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