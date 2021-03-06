/*
 * Copyright (c) 2021 Michael de Gans
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef AC5EBD7A_6EA4_46E9_980F_E6964D058DB4
#define AC5EBD7A_6EA4_46E9_980F_E6964D058DB4

#include <Argus/Argus.h>

#include <experimental/optional>

namespace nvmanualcam {

/** type representing RGB curves */
using RGBCurves =
    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>>;

/**
 * @brief More or less a copy of Argus::ICaptureMetadata with minor changes.
 */
class Metadata {
 public:
  // pattern from:
  // https://stackoverflow.com/a/38962230/11049585

  /**
   * @brief Create new Metadata if possible.
   *
   * @param meta directly from libargus
   *
   * @return Metadata on success, nullopt on failure
   */
  static std::experimental::optional<Metadata> create(
      Argus::CaptureMetadata* meta);
  Metadata(Metadata& other) = default;
  Metadata(Metadata&& other) = default;
  virtual ~Metadata() = default;

  Metadata& operator=(Metadata& other) = default;
  Metadata& operator=(Metadata&& other) = default;

  // TODO(mdegans)
  //   std::string as_json();
  //   std::string as_proto();
  //   static std::experimental::optional<Metadata> create(
  //       std::string meta);

  /**
   * @brief Get the status of the Auto Exposure lock.
   *
   * @return true if locked
   */
  virtual bool getAeLocked() const { return aeLocked_; }
  /**
   * @brief Get the Auto Exposure state (searching, converged...)
   *
   * @return Argus::AeState enum value
   */
  virtual Argus::AeState getAeState() const { return aeState_; }
  /**
   * @brief Get the active AE regions, if available.
   *
   * @return std::experimental::optional<std::vector<Argus::AcRegion>>
   */
  virtual std::experimental::optional<std::vector<Argus::AcRegion>>
  getAeRegions() const {
    return aeRegions_;
  }
  /**
   * @brief Get the Auto White Balance CCT. Bonus if you know what CCT is,
   * because Google is no help here.
   *
   * @return mystery value for mystery thing
   */
  virtual uint32_t getAwbCct() const { return awbCct_; }
  /**
   * @brief Get the per channel White Balance Gains.
   *
   * @return Argus::BayerTuple<float>
   */
  virtual Argus::BayerTuple<float> getAwbGains() const { return awbGains_; }
  /**
   * @brief Get the Ayto White Balance Mode (off, auto, incandescent...)
   *
   * @return Argus::AwbMode enum value
   */
  virtual Argus::AwbMode getAwbMode() const { return awbMode_; }
  /**
   * @brief Get the Auto White Balance Regions.
   *
   * @return std::experimental::optional<std::vector<Argus::AcRegion>>
   */
  virtual std::experimental::optional<std::vector<Argus::AcRegion>>
  getAwbRegions() const {
    return awbRegions_;
  }
  /**
   * @brief Get the Auto White Balance State (searching, converged...)
   *
   * @return Argus::AwbState
   */
  virtual Argus::AwbState getAwbState() const { return awbState_; }
  /**
   * @brief Get the Auto White Balance white balance estimate.
   *
   * @return std::experimental::optional<std::vector<float>>
   */
  virtual std::experimental::optional<std::vector<float>> getAwbWbEstimate()
      const {
    return awbWbEstimate_;
  }
  /**
   * @brief Get the Bayer Histogram if enabled and available.
   *
   * @return
   * std::experimental::optional<std::vector<Argus::BayerTuple<uint32_t>>>
   */
  virtual std::experimental::optional<std::vector<Argus::BayerTuple<uint32_t>>>
  getBayerHistogram() const {
    return bayerHistogram_;
  }
  /**
   * @brief Get the Bayer Histogram Region if enabled and available.
   *
   * @return
   * std::experimental::optional<std::vector<Argus::Rectangle<uint32_t>>>
   */
  virtual std::experimental::optional<std::vector<Argus::Rectangle<uint32_t>>>
  getBayerHistogramRegion() const {
    return bayerHistogramRegion_;
  }
  /**
   * @brief Return the capture id (return from ICaptureSession::capture())
   *
   * @return id of the capture call associated with this metadata
   */
  virtual uint32_t getCaptureId() const { return captureId_; }
  /**
   * @brief Argus docs: "Returns the clientData value for the Request used in
   * the capture that generated this metadata."
   *
   * @return id of the clentData assocaited with this metadata
   */
  virtual uint32_t getClientData() const { return clientData_; }
  /**
   * @brief Get the Color Correction Matrix if available.
   *
   * @return std::experimental::optional<std::vector<float>>
   */
  virtual std::experimental::optional<std::vector<float>>
  getColorCorrectionMatrix() const {
    return colorCorrectionMatrix_;
  }
  /**
   * @brief Get whether the color correction matrix is enabled or not.
   *
   * @return true if enabled
   */
  virtual bool getColorCorrectionMatrixEnable() const {
    return colorCorrectionMatrixEnabled_;
  }
  /**
   * @brief Get the Color Saturation.
   *
   * @return float value from -1.0 to 1.0
   */
  virtual float getColorSaturation() const { return colorSaturation_; }
  /**
   * @brief Get the Flicker reduction mode (off, 50hz, 60hz)
   *
   * @return Argus::AeFlickerState
   */
  virtual Argus::AeFlickerState getFlickerState() const {
    return flickerState_;
  }
  /**
   * @brief Get the Focuser Position
   *
   * @return int32_t
   */
  virtual int32_t getFocuserPosition() const { return focuserPosition_; }
  /**
   * @brief Get the duration it took to generate the associated frame.
   *
   * @return time in nanoseconds
   */
  virtual uint64_t getFrameDuration() const { return frameDuration_; }
  /**
   * @brief Get the duration it took for the sensor to capture.
   *
   * @return time in nanoseconds
   */
  virtual uint64_t getFrameReadoutTime() const { return frameReadoutTime_; }
  /**
   * @brief Get the Isp Digital Gain.
   *
   * @return float
   */
  virtual float getIspDigitalGain() const { return ispDigitalGain_; }
  /**
   * @brief Get the RGB histogram bin count if available.
   *
   * @return std::experimental::optional<uint32_t>
   */
  virtual std::experimental::optional<uint32_t> getRgbHistogramBinCount()
      const {
    return rbgHistogramBinCount_;
  }
  /**
   * @brief Get the Rgb Histogram if available
   *
   * @return std::experimental::optional<std::vector<Argus::RGBTuple<uint32_t>>>
   */
  virtual std::experimental::optional<std::vector<Argus::RGBTuple<uint32_t>>>
  getRgbHistogram() const {
    return rgbHistogram_;
  }
  /**
   * @brief Get the approximate scene illumination.
   *
   * @return float
   */
  virtual float getSceneLux() const { return sceneLux_; }
  /**
   * @brief Get the analog gain.
   *
   * @return float
   */
  virtual float getSensorAnalogGain() const { return sensorAnalogGain_; }
  /**
   * @brief Get the sensor exposure time.
   *
   * @return time in nanoseconds
   */
  virtual uint64_t getSensorExposureTime() const { return sensorExposureTime_; }
  /**
   * @brief Get sensitivity of the sensor.
   *
   * @return ISO value
   */
  virtual uint32_t getSensorSensitivity() const { return sensorSensitivity_; }
  /**
   * @brief Get the Sensor timestamp.
   *
   * @return time in nanoseconds
   */
  virtual uint64_t getSensorTimestamp() const { return sensorTimestamp_; }
  /**
   * @brief Get the sharpness score if available.
   *
   * @return std::experimental::optional<std::vector<float>>
   */
  virtual std::experimental::optional<std::vector<float>> getSharpnessScore()
      const {
    return sharpnessScore_;
  }
  /**
   * @brief Get tone map curves if available.
   *
   * @return an optional 3 tuple of float vectors in RGB order
   */
  virtual std::experimental::optional<RGBCurves> getToneMapCurves() const;

 private:
  Metadata(Argus::ICaptureMetadata* imeta);

  const bool aeLocked_;
  std::experimental::optional<std::vector<Argus::AcRegion>> aeRegions_;
  const Argus::AeState aeState_;
  const uint32_t awbCct_;
  const Argus::BayerTuple<float> awbGains_;
  const Argus::AwbMode awbMode_;
  std::experimental::optional<std::vector<Argus::AcRegion>> awbRegions_;
  const Argus::AwbState awbState_;
  std::experimental::optional<std::vector<float>> awbWbEstimate_;
  std::experimental::optional<uint32_t> bayerHistogramBinCount_;
  std::experimental::optional<std::vector<Argus::BayerTuple<uint32_t>>>
      bayerHistogram_;
  std::experimental::optional<std::vector<Argus::Rectangle<uint32_t>>>
      bayerHistogramRegion_;
  uint32_t captureId_;
  uint32_t clientData_;
  std::experimental::optional<std::vector<float>> colorCorrectionMatrix_;
  const bool colorCorrectionMatrixEnabled_;
  const float colorSaturation_;
  const Argus::AeFlickerState flickerState_;
  int32_t focuserPosition_;
  uint64_t frameDuration_;
  uint64_t frameReadoutTime_;
  float ispDigitalGain_;
  std::experimental::optional<uint32_t> rbgHistogramBinCount_;
  std::experimental::optional<std::vector<Argus::RGBTuple<uint32_t>>>
      rgbHistogram_;
  float sceneLux_;
  float sensorAnalogGain_;
  uint64_t sensorExposureTime_;
  uint32_t sensorSensitivity_;
  uint64_t sensorTimestamp_;
  std::experimental::optional<std::vector<float>> sharpnessScore_;
  // TODO(mdegans): stream metadata
  std::experimental::optional<std::vector<float>> toneMapCurveR_;
  std::experimental::optional<std::vector<float>> toneMapCurveG_;
  std::experimental::optional<std::vector<float>> toneMapCurveB_;
  bool toneMapCurveEnabled_;
};

}  // namespace nvmanualcam

#endif /* AC5EBD7A_6EA4_46E9_980F_E6964D058DB4 */
