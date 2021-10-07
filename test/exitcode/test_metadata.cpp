#include "gst/gstpad.h"
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

#define TEST_FAIL(...)          \
  do {                          \
    ret = GST_PAD_PROBE_REMOVE; \
    --retcode;                  \
    GST_ERROR(__VA_ARGS__);     \
  } while (0)

static int meta_counter = 0;
static int retcode = 0;

GstPadProbeReturn metadata_probe(GstPad* pad,
                                 GstPadProbeInfo* info,
                                 gpointer user_data) {
  (void)pad;
  (void)user_data;
  auto metaa = nvmanualcam::Metadata::create(info);
  auto metab = nvmanualcam::Metadata::steal(info);
  auto ret = GST_PAD_PROBE_OK;

  if (!metaa) {
    TEST_FAIL("no copied metadata found");
  }

  if (!metab) {
    TEST_FAIL("no moved metadata found");
  }
  GST_INFO("got metadata from buffer (%d)", ++meta_counter);

  // test sensor timestamp
  auto timestamp = metab->getSensorTimestamp();
  if (timestamp) {
    GST_INFO("Sensor timestamp:%ld", timestamp);
  } else {
    TEST_FAIL("Could not getSensorTimestamp (return was 0)");
  }
  auto now_ns = (uint64_t)g_get_monotonic_time() * 1000;
  GST_INFO("Now Monotonic NS:%ld", now_ns);
  int64_t latency_ns = now_ns - timestamp;
  GST_INFO("Sensor to probe latency:%.4f ms", (double)latency_ns / 1000000.0);
  if (latency_ns < 0) {
    TEST_FAIL("Sensor timestamp is in the future (%lu ns). Offset is wrong?",
              -latency_ns);
  } else if (latency_ns > GST_SECOND) {
    TEST_FAIL(
        "Sensor timestamp latency greater than 1 second (%lu ns). Old JetPack "
        "version?",
        latency_ns);
  }

  // test sensor exposure time
  auto exp_time = metab->getSensorExposureTime();
  if (exp_time) {
    GST_INFO("Sensor exposure time:%ld", timestamp);
  } else {
    TEST_FAIL("Sensor exposure time is 0");
  }
  // TODO(mdegans): test exposure time is less than a frame (it isn't and
  // that's) not possible, so it appears libargus is producing bad meta on this.

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

  // test sharpness score
  auto score = metab->getSharpnessScore();
  if (score) {
    GST_INFO("Sharpness score:%.3f", score.value());
  } else {
    TEST_FAIL("Shapness score not available.");
  }

  // test scene lux
  auto lux = metab->getSceneLux();
  GST_INFO("Scene Lux: %.3f", lux);

  // test bayer-sharpness-map
  auto svalues = metab->getSharpnessValues();
  if (svalues) {
    GST_INFO("Sharpness values length:%d", svalues.value().size().area());
  } else {
    TEST_FAIL("Could not get SharpnessValues. `bayer-sharpness-map` broken.");
  }

  return ret;
}

int main(int argc, char** argv) {
  gst_init(&argc, &argv);
  GST_DEBUG_CATEGORY_INIT(test_cat, TEST_NAME, 0, TEST_NAME);

  g_autoptr(GstElement) pipe = nullptr;
  GstElement* camera = nullptr;
  GstElement* fakesink = nullptr;
  g_autoptr(GstPad) pad = nullptr;
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

  g_object_set(camera, "bayer-sharpness-map", true, nullptr);
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
  return retcode;
}