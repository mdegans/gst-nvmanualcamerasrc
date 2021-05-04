#ifndef E92A8243_BA44_4DAB_ACC2_4D74F6E5F878
#define E92A8243_BA44_4DAB_ACC2_4D74F6E5F878

#include <thread>

#include <Argus/Argus.h>
#include <EGLStream/FrameConsumer.h>

#include "gstnvmanualcamera_config.h"
#include "gstnvmanualcamera_utils.h"

using namespace Argus;

static bool list_sensor_modes(const std::vector<SensorMode*>& modes) {
  for (const auto& mode : modes) {
    const auto iMode = interface_cast<ISensorMode>(mode);
    if (!iMode) {
      return false;
    }
    auto analog_range = iMode->getAnalogGainRange();
    auto exp_range = iMode->getExposureTimeRange();
    GST_INFO(
        "%d x %d FR = %f fps Duration = %lu ; Analog Gain range min %f, max "
        "%f; Exposure Range min %ju, max %ju;",
        iMode->getResolution().width(), iMode->getResolution().height(),
        (1e9 / (iMode->getFrameDurationRange().min())),
        iMode->getFrameDurationRange().min(), analog_range.min(),
        analog_range.max(), exp_range.min(), exp_range.max());
  }
  return true;
}

struct ArgusCtx final {
  bool stop_requested = false;
  bool in_error = false;
  size_t total_sensor_modes = 0;

  UniqueObj<CameraProvider> provider;
  ICameraProvider* iProvider = nullptr;

  UniqueObj<Request> request;
  IRequest* iRequest = nullptr;

  UniqueObj<CaptureSession> session;
  ICaptureSession* iSession = nullptr;

  UniqueObj<OutputStreamSettings> oSSettings;
  IOutputStreamSettings* iOSSettings = nullptr;

  UniqueObj<OutputStream> oStream;
  IEGLOutputStream* iOStream = nullptr;

  UniqueObj<EGLStream::FrameConsumer> frameConsumer;
  EGLStream::IFrameConsumer iFrameConsumer = nullptr;

  ICameraProperties* iCameraProperties = nullptr;
  IAutoControlSettings* iAutoSettings = nullptr;
  IDenoiseSettings* iDenoiseSettings = nullptr;
  IEdgeEnhanceSettings* iEeSettings = nullptr;
  ISourceSettings* iSourceSettings = nullptr;

  static std::shared_ptr<ArgusCtx> create(
      const Argus::Size2D<uint32_t> resolution,
      const std::shared_ptr<ArgusControls> controls,
      const size_t dev_id = 0,
      const size_t mode_id = 0) {
    Argus::Status status;
    std::vector<CameraDevice*> devices;
    auto ctx = std::make_shared<ArgusCtx>();
    std::vector<SensorMode*> modes;

    // Create the CameraProvider
    ctx->provider.reset(CameraProvider::create(&status));
    ctx->iProvider = interface_cast<ICameraProvider>(provider);
    if (!ctx->iProvider) {
      GST_ERROR("Could not create CameraProvider (Status %d)", status);
      return nullptr;
    }

    // Get the camera devices.
    status = ctx->iProvider->getCameraDevices(&devices);
    if (status) {
      GST_ERROR("Could not getCameraDevices (Status %d)", status);
      return nullptr;
    }
    if (devices.size() == 0) {
      GST_ERROR("No cameras available.");
      return nullptr;
    }
    if (dev_id >= devices.size()) {
      GST_ERROR("Invalid camera device specified (%d specified, %d max index).",
                dev_id, static_cast<int32_t>(devices.size()) - 1);
      return nullptr;
    }

    // Create the capture session using the specified device.
    ctx->session.reset(
        iProvider->createCaptureSession(devices[dev_id], &status));
    ctx->iSession = interface_cast<ICaptureSession>(ctx->session);
    if (!ctx->iSession) {
      GST_ERROR("Could not create CaptureSessino (Status %d", status);
      return nullptr;
    }

    // src->iCaptureSession_ptr = iCaptureSession;
    GST_INFO("Creating output stream");
    ctx->oSSettings.reset(
        iSession->createOutputStreamSettings(STREAM_TYPE_EGL, &status));
    ctx->iOSSettings =
        interface_cast<IEGLOutputStreamSettings>(ctx->oSSettings);
    if (!iOSSettings) {
      GST_ERROR("Could not create IEGLOutputStreamSettings (Status %d)",
                status);
      return nullptr;
    }
    ctx->iOSSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    ctx->iOSSettings->setResolution(resolution);
    if (controls->getMetaEnabled()) {
      GST_INFO("Enabling metadata.");
      ctx->iOSSettings->setMetadataEnable(true);
    } else {
      GST_INFO("Metadata not enabled.");
    }

    ctx->oStream.reset(
        ctx->iSession->createOutputStream(ctx->oSSettings.get(), &status));
    ctx->iOStream = interface_cast<IEGLOutputStream>(ctx->oStream);
    if (!ctx->iOStream) {
      GST_ERROR("Could not createOutputStream (status %d)", status);
      return nullptr;
    }

    // Create the FrameConsumer.
    ctx->frameConsumer.reset(
        EGLStream::FrameConsumer::create(oStream.get(), &status));
    ctx->iFrameConsumer =
        interface_cast<EGLStream::IFrameConsumer>(ctx->frameConsumer.get());
    if (!ctx->iFrameConsumer) {
      GST_ERROR("Failed to create FrameConsumer (status %d)", status);
      return nullptr;
    }

    // Wait until the producer has connected to the stream.
    GST_INFO("Waiting until producer is connected...");
    if (ctx->iOStream->waitUntilConnected() != STATUS_OK) {
      GST_ERROR("Stream failed to connect.");
      return nullptr;
    }
    GST_INFO("Producer has connected; continuing.");

    // Create capture request and enable output stream.
    ctx->request.reset(ctx->iSession->createRequest());
    ctx->iRequest = interface_cast<IRequest>(request.get());
    if (!ctx->iRequest)
      ORIGINATE_ERROR("Failed to create Request");
    ctx->iRequest->enableOutputStream(ctx->oStream.get());

    ctx->iAutoSettings = interface_cast<IAutoControlSettings>(
        iRequest->getAutoControlSettings());
    ctx->iSourceSettings =
        interface_cast<ISourceSettings>(iRequest->getSourceSettings());

    // Get properites interface of our selected devices
    ctx->iCameraProperties = interface_cast<ICameraProperties>(devices[dev_id]);

    // Get all sensor modes, list them, and check the id is valid
    err = ctx->iCameraProperties->getAllSensorModes(&modes);
    if (err) {
      GST_ERROR("Could not get sensor modes.");
      return nullptr;
    }
    g_assert(list_sensor_modes(modes));
    ctx->total_sensor_modes = modes.size();
    if (mode_id >= ctx->total_sensor_modes) {
      GST_ERROR("Invalid sensor mode (%d selected, %d available).", mode_id,
                ctx->total_sensor_modes);
    }

    return ctx;
  }

  Argus::Status repeat_request() const {
    return this->iSession->repeat(this->request.get());
  }

 private:
  ~ArgusCtx() {
    if (iSession) {
      iSession->cancelRequests();
      // docs say stoprepeat is implicit with above
      iSession->waitForIdle();
      iSession = nullptr;
    }
  }
};

struct ArgusControls {
 public:
  ArgusControls() {
    gst_video_info_init(&info_);
    info_.width = nvmanualcam::defaults::DEFAULT_WIDTH;
    info_.height = nvmanualcam::defaults::DEFAULT_HEIGHT;
    info_.fps_n = nvmanualcam::defaults::DEFAULT_FPS;
    info_.fps_d = 1;
  }
  bool set_property(const Properties::ID id,
                    const GValue* value,
                    const GParamSpec* pspec) {
    updated_[static_cast<size_t>(id)] = true;

    switch (id) {
      case Properties::ID::SILENT:
        silent_ = g_value_get_boolean(value);
        break;
      case Properties::ID::TIMEOUT:
        timeout_ = g_value_get_boolean(value);
        break;
      case Properties::ID::WHITE_BALANCE:
        wbmode_ = (NvManualCamAwbMode)g_value_get_enum(value);
        break;
      case Properties::ID::SATURATION:
        saturation_ = g_value_get_float(value);
        break;
      case Properties::ID::SENSOR_ID:
        sensor_id_ = g_value_get_int(value);
        break;
      case Properties::ID::SENSOR_MODE:
        sensor_mode_ = g_value_get_int(value);
        break;
      case Properties::ID::EXPOSURE_TIME:
        exposure_time_ = g_value_get_float(value);
        exposure_real_ = exposure_time_ * frame_duration_;
        break;
      case Properties::ID::GAIN:
        gain_ = g_value_get_float(value);
        break;
      case Properties::ID::DIGITAL_GAIN:
        digital_gain_ = g_value_get_float(value);
        break;
      case Properties::ID::TNR_STRENGTH:
        tnr_strength_ = g_value_get_float(value);
        break;
      case Properties::ID::TNR_MODE:
        tnr_mode_ = (NvManualCamNoiseReductionMode)g_value_get_enum(value);
        break;
      case Properties::ID::EE_STRENGTH:
        ee_strength_ = g_value_get_float(value);
        break;
      case Properties::ID::EE_MODE:
        ee_mode_ = (NvManualCamEdgeEnhancementMode)g_value_get_enum(value);
        break;
      case Properties::ID::AEANTIBANDING_MODE:
        aeantibanding_mode_ =
            (NvManualCamAeAntibandingMode)g_value_get_enum(value);
        break;
      case Properties::ID::EXP_COMPENSATION:
        exp_compensation_ = g_value_get_float(value);
        break;
      case Properties::ID::AE_LOCK:
        ae_lock_ = g_value_get_boolean(value);
        break;
      case Properties::ID::AWB_LOCK:
        awb_lock_ = g_value_get_boolean(value);
        break;
      case Properties::ID::BAYER_SHARPNESS_MAP:
        bayer_sharpness_map_ = g_value_get_boolean(value);
        break;
      case Properties::ID::META_ENABLED:
        meta_enabled_ = g_value_get_boolean(value);
        break;
      case Properties::ID::BUFAPI:
        buf_api_ = g_value_get_boolean(value);
        break;

      default:
        GST_ERROR("CRITICAL! PROPERTY SET CODE BROKEN!");
        return false;
    }

    return true;
  }

  bool get_property(const Properties::ID id,
                    GValue* value,
                    const GParamSpec* pspec) {
    switch (id) {
      case Properties::ID::SILENT:
        g_value_set_boolean(value, silent_);
        break;
      case Properties::ID::TIMEOUT:
        g_value_set_uint(value, timeout_);
        break;
      case Properties::ID::WHITE_BALANCE:
        g_value_set_enum(value, wbmode_);
        break;
      case Properties::ID::SATURATION:
        g_value_set_float(value, saturation_);
        break;
      case Properties::ID::SENSOR_ID:
        g_value_set_int(value, sensor_id_);
        break;
      case Properties::ID::SENSOR_MODE:
        g_value_set_int(value, sensor_mode_);
        break;
      case Properties::ID::TOTAL_SENSOR_MODES:
        g_value_set_int(value, total_sensor_modes_);
        break;
      case Properties::ID::EXPOSURE_TIME:
        g_value_set_float(value, exposure_time_);
        break;
      case Properties::ID::EXPOSURE_REAL:
        g_value_set_uint64(value, exposure_real_);
        break;
      case Properties::ID::GAIN:
        g_value_set_float(value, gain_);
        break;
      case Properties::ID::DIGITAL_GAIN:
        g_value_set_float(value, digital_gain_);
        break;
      case Properties::ID::TNR_STRENGTH:
        g_value_set_float(value, tnr_strength_);
        break;
      case Properties::ID::TNR_MODE:
        g_value_set_enum(value, tnr_mode_);
        break;
      case Properties::ID::EE_MODE:
        g_value_set_enum(value, ee_mode_);
        break;
      case Properties::ID::EE_STRENGTH:
        g_value_set_float(value, ee_strength_);
        break;
      case Properties::ID::AEANTIBANDING_MODE:
        g_value_set_enum(value, aeantibanding_mode_);
        break;
      case Properties::ID::EXP_COMPENSATION:
        g_value_set_float(value, exp_compensation_);
        break;
      case Properties::ID::AE_LOCK:
        g_value_set_boolean(value, ae_lock_);
        break;
      case Properties::ID::AWB_LOCK:
        g_value_set_boolean(value, awb_lock_);
        break;
      case Properties::ID::BAYER_SHARPNESS_MAP:
        g_value_set_boolean(value, bayer_sharpness_map_);
        break;
      case Properties::ID::META_ENABLED:
        g_value_set_boolean(value, meta_enabled_);
        break;
      case Properties::ID::BUFAPI:
        g_value_set_boolean(value, buf_api_);
        break;

      default:
        // if we reach here, something is very broken.
        GST_ERROR("CRITICAL! PROPERTY GET CODE BROKEN!");
        return false;
        break;
    }
  }

  /** Check if a property has been updated **and reset the updated state** */
  bool ArgusControls::isUpdated(const Properties::ID id) const {
    bool ret = updated_.at(static_cast<size_t>(id));
    updated_[static_cast<size_t>(id)] = false;
    return ret;
  }

  uint getTimeout() const { return timeout_; }

  bool getSilent() const { return silent_; }

  NvManualCamAwbMode getWbmode() const { return wbmode_; }

  float getSaturation() const { return saturation_; }

  float getExposureTime() const { return exposure_time_; }

  uint64_t getExposureReal() const { return exposure_real_; }

  float getGain() const { return gain_; }

  float getDigitalGain() const { return digital_gain_; }

  gboolean getMetaEnabled() const { return meta_enabled_; }

  gboolean getBayerSharpnessMap() const { return bayer_sharpness_map_; }

  NvManualCamNoiseReductionMode getTnrMode() const { return tnr_mode_; }

  NvManualCamEdgeEnhancementMode getEeMode() const { return ee_mode_; }

  NvManualCamAeAntibandingMode getAeantibandingMode() const {
    return aeantibanding_mode_;
  }

  float getTnrStrength() const { return tnr_strength_; }

  float getEeStrength() const { return ee_strength_; }

  float getExpCompensation() const { return exp_compensation_; }

  bool getAeLock() const { return ae_lock_; }

  bool getAwbLock() const { return awb_lock_; }

  int getSensorId() const { return sensor_id_; }

  int getSensorMode() const { return sensor_mode_; }

  uint getTotalSensorModes() const { return total_sensor_modes_; }

  bool getBufApi() const { return buf_api_; }

  GstClockTime getFrameDuration() const { return frame_duration_; }
  void setFrameDuration(const GstClockTime frame_duration) {
    frame_duration_ = frame_duration;
    updated_[static_cast<size_t>(Properties::ID::EXPOSURE_REAL)] = true;
  }

  GstVideoInfo info() const { return info_; }

 private:
  GstVideoInfo info_;
  uint timeout_ = nvmanualcam::defaults::TIMEOUT;
  bool silent_ = nvmanualcam::defaults::SILENT;
  NvManualCamAwbMode wbmode_ = nvmanualcam::defaults::WB_MODE;
  float saturation_ = nvmanualcam::defaults::SATURATION;
  float exposure_time_ = nvmanualcam::defaults::EXPOSURE_TIME;  // in frames
  uint64_t exposure_real_ = 0;                                // in nanoseconds
  float gain_ = nvmanualcam::defaults::GAIN;                  // min: 1, max: 16
  float digital_gain_ = nvmanualcam::defaults::DIGITAL_GAIN;  // min 1, max: 256
  gboolean meta_enabled_ =
      nvmanualcam::defaults::METADATA;  // enable metadata generation
  gboolean bayer_sharpness_map_ =
      nvmanualcam::defaults::BAYER_SHARPNESS_MAP;  // enable BayerSharpnessMap
                                                   // metadata
  NvManualCamNoiseReductionMode tnr_mode_ = nvmanualcam::defaults::TNR_MODE;
  NvManualCamEdgeEnhancementMode ee_mode_ = nvmanualcam::defaults::EE_MODE;
  NvManualCamAeAntibandingMode aeantibanding_mode_ =
      nvmanualcam::defaults::AEANTIBANDING_MODE;
  float tnr_strength_ = nvmanualcam::defaults::TNR_STRENGTH;
  float ee_strength_ = nvmanualcam::defaults::EE_STRENGTH;
  float exp_compensation_ = nvmanualcam::defaults::EXP_COMPENSATION;
  bool ae_lock_ = nvmanualcam::defaults::AE_LOCK;
  bool awb_lock_ = nvmanualcam::defaults::AWB_LOCK;
  int sensor_id_ = nvmanualcam::defaults::SENSOR_ID;
  int sensor_mode_ = nvmanualcam::defaults::SENSOR_MODE_STATE;
  uint total_sensor_modes_ = nvmanualcam::defaults::TOTAL_SENSOR_MODES;
  bool buf_api_ = nvmanualcam::defaults::BUF_API;

  GstClockTime frame_duration_ = 0;

  std::array<bool, (size_t)Properties::ID::SENTINEL> updated_{false};
};

struct Properties {
  Properties() : pspecs_(create_pspecs()){};

  /** our properties enum for this element */
  enum class ID {
    AE_LOCK,
    AEANTIBANDING_MODE,
    AWB_LOCK,
    BAYER_SHARPNESS_MAP,
    BUFAPI,
    DIGITAL_GAIN,
    EE_MODE,
    EE_STRENGTH,
    EXP_COMPENSATION,
    EXPOSURE_REAL,
    EXPOSURE_TIME,
    GAIN,
    META_ENABLED,
    SATURATION,
    SENSOR_ID,
    SENSOR_MODE,
    SENTINEL,
    SILENT,
    TIMEOUT,
    TNR_MODE,
    TNR_STRENGTH,
    TOTAL_SENSOR_MODES,
    WHITE_BALANCE,
    SENTINEL,
  };

  /** install properties on a GObjectClass */
  static void install(GObjectClass* gobject_class) {
    auto pspecs = create_pspecs();
    for (guint i = 0; i != (guint)ID::SENTINEL; i++) {
      g_object_class_install_property(gobject_class, i, pspecs[i]);
    }
  }

 private:
  ArgusControls ctrl_;

  /** helper to initialize pspecs */
  static std::array<GParamSpec*, (size_t)ID::SENTINEL> create_pspecs() {
    std::array<GParamSpec*, (size_t)ID::SENTINEL> properties;

    for (size_t e = 0; e != (size_t)ID::SENTINEL; e++) {
      switch (static_cast<ID>(e)) {
        case ID::AE_LOCK:
          properties[e] = g_param_spec_boolean(
              "aelock", "AE Lock", "set or unset the auto exposure lock",
              nvmanualcam::defaults::AE_LOCK, (GParamFlags)G_PARAM_READWRITE);
          break;
        case ID::AEANTIBANDING_MODE:
          properties[e] = g_param_spec_enum(
              "aeantibanding", "Auto Exposure Antibanding Mode",
              "property to set the auto exposure antibanding mode",
              GST_TYPE_NVMANUALCAM_AEANTIBANDING_MODE,
              nvmanualcam::defaults::AEANTIBANDING_MODE,
              (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
          break;
        case ID::AWB_LOCK:
          properties[e] = g_param_spec_boolean(
              "awblock", "AWB Lock", "set or unset the auto white balance lock",
              nvmanualcam::defaults::AWB_LOCK, (GParamFlags)G_PARAM_READWRITE);
          break;
        case ID::BAYER_SHARPNESS_MAP:
          properties[e] = g_param_spec_boolean(
              "bayer-sharpness-map", "Bayer Sharpness Map",
              "Generate and attach IBayerSharpnessMap metadata.",
              nvmanualcam::defaults::BAYER_SHARPNESS_MAP,
              (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY));
          break;
        case ID::BUFAPI:
          properties[e] = g_param_spec_boolean(
              "bufapi-version", "Buffer API", "set to use new Buffer API",
              nvmanualcam::defaults::BUFAPI, (GParamFlags)G_PARAM_READWRITE);
          break;
        case ID::DIGITAL_GAIN:
          break;
        case ID::EE_MODE:
          break;
        case ID::EE_STRENGTH:
          break;
        case ID::EXP_COMPENSATION:
          break;
        case ID::EXPOSURE_REAL:
          break;
        case ID::EXPOSURE_TIME:
          break;
        case ID::GAIN:
          break;
        case ID::META_ENABLED:
          break;
        case ID::SATURATION:
          break;
        case ID::SENSOR_ID:
          break;
        case ID::SENSOR_MODE:
          break;
        case ID::SENTINEL:
          break;
        case ID::SILENT:
          break;
        case ID::TIMEOUT:
          break;
        case ID::TNR_MODE:
          break;
        case ID::TNR_STRENGTH:
          break;
        case ID::TOTAL_SENSOR_MODES:
          break;
        case ID::WHITE_BALANCE:
          break;
        case ID::SENTINEL:
          // something is broken
          [[fallthrough]];
        default:
          GST_ERROR("Property initialzation code broken.");
          std::terminate();
          break;
      }
    }
    return properties;
  }
};

#endif /* E92A8243_BA44_4DAB_ACC2_4D74F6E5F878 */
