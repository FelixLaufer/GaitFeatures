#ifndef _GAIT_FEATURE_EXTRACTOR_H_
#define _GAIT_FEATURE_EXTRACTOR_H_

#include "FSMProcessor.h"

#include "IMUData.h"
#include "RollPitchTracker.h"

enum GaitFeatureExtractorState
{
  Uncalibrated = 0,
  StaticCalibration = 1,
  DynamicCalibration = 2,
  Calibrated = 3
};

enum GaitFeatureExtractorEvent
{
  NoGaitSignalExtractorEvent = 0,
  StaticCalibrated = 1,
  DynamicCalibrated = 2,
};

class GaitFeatureExtractor : public FSMProcessor<IMUData, IMUData, Vector3, GaitFeatureExtractorState, GaitFeatureExtractorEvent>
{
public:
  struct Config
  {
    unsigned int minStaticFrames = 30; // 50
    unsigned int minDynamicFrames = 25; // 25
    ScalarType noAccNormThreshold = 0.2; // 0.1
    ScalarType noGyrNormThreshold = 0.05; // 0.1
    ScalarType someAccNormThreshold = 0.4; // 0.2
    ScalarType someGyrNormThreshold = 0.2; // 0.2
	ScalarType g0 = 9.809752; // local gravity in Kaiserslautern, Germany (source: ptb.de)
  };

  GaitFeatureExtractor(const unsigned int maxBufferSize = 1000, const Config config = Config());
  GaitFeatureExtractor(const std::vector<IMUData>& staticIMUData, const std::vector<IMUData>& dynamicIMUData, const unsigned int maxBufferSize = 1000);
  bool calibrated() const { return state_ == GaitFeatureExtractorState::Calibrated; }
  Vector3 process(const IMUData& imuData) override;

private:
  Matrix3x3 pcaR(const Matrix& data, const bool centered = false, const bool validRotation = true);
  void calibrate();
  Vector3 extract(const IMUData& imuData);

  Config config_;
  RollPitchTracker rollPitchTracker_;
  std::vector<IMUData> staticIMUData_;
  std::vector<IMUData> dynamicIMUData_;
  Matrix3x3 gyrR_;
  Matrix3x3 accR_;
};

#endif