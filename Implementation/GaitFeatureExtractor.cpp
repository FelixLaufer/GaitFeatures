#include "GaitFeatureExtractor.h"

GaitFeatureExtractor::GaitFeatureExtractor(const unsigned int maxBufferSize, const Config config)
  : FSMProcessor<IMUData, IMUData, Vector3, GaitFeatureExtractorState, GaitFeatureExtractorEvent>(maxBufferSize)
  , config_(config)
  , staticIMUData_(std::vector<IMUData>())
  , dynamicIMUData_(std::vector<IMUData>())
  , accR_(Matrix3x3::Identity())
  , gyrR_(Matrix3x3::Identity())
  , rollPitchTracker_(RollPitchTracker())
{}

GaitFeatureExtractor::GaitFeatureExtractor(const std::vector<IMUData>& staticIMUData, const std::vector<IMUData>& dynamicIMUData, const unsigned int maxBufferSize)
  : FSMProcessor<IMUData, IMUData, Vector3, GaitFeatureExtractorState, GaitFeatureExtractorEvent>(maxBufferSize)
  , config_(Config())
  , staticIMUData_(staticIMUData)
  , dynamicIMUData_(dynamicIMUData)
  , accR_(Matrix3x3::Identity())
  , gyrR_(Matrix3x3::Identity())
  , rollPitchTracker_(RollPitchTracker())
{
  calibrate();
  state_ = Calibrated;
}

Vector3 GaitFeatureExtractor::process(const IMUData& imuData)
{
  // Default event and return value
  GaitFeatureExtractorEvent event = NoGaitSignalExtractorEvent;
  Vector3 ret = Vector3::Zero();

  // Store data
  buffer_.push(imuData);

  // Give us a view on the recent data
  const std::vector<IMUData> view = recent(3, true);

  // State machine logic
  switch (state_)
  {
  case Uncalibrated:
  {
    if (view.size() >= 3
      && view[0].acc().norm() - config_.g0 < config_.noAccNormThreshold && view[0].gyr().norm() < config_.noGyrNormThreshold
      && view[1].acc().norm() - config_.g0 < config_.noAccNormThreshold && view[1].gyr().norm() < config_.noGyrNormThreshold
      && view[2].acc().norm() - config_.g0 < config_.noAccNormThreshold && view[2].gyr().norm() < config_.noGyrNormThreshold)
    {
      staticIMUData_.clear();
      state_ = StaticCalibration;
    }
    break;
  }
  case StaticCalibration:
  {
    if (view[0].acc().norm() - config_.g0 < config_.noAccNormThreshold && view[0].gyr().norm() < config_.noGyrNormThreshold)
      staticIMUData_.emplace_back(imuData);
    else
    {
      if (staticIMUData_.size() >= config_.minStaticFrames)
      {
        dynamicIMUData_.clear();
        state_ = DynamicCalibration;
        event = StaticCalibrated;
      }
      else
        staticIMUData_.clear();
    }
    break;
  }
  case DynamicCalibration:
  {
    if (view[0].acc().norm() - config_.g0 > config_.noAccNormThreshold&& view[0].gyr().norm() > config_.noGyrNormThreshold)
      dynamicIMUData_.emplace_back(imuData);
    else
    {
      if (view[0].acc().norm() - config_.g0 < config_.someAccNormThreshold && view[0].gyr().norm() < config_.someGyrNormThreshold)
      {
        if (dynamicIMUData_.size() >= config_.minDynamicFrames)
        {
          calibrate();

          state_ = Calibrated;
          event = DynamicCalibrated;
        }
        else
          dynamicIMUData_.clear();
      }
    }
    break;
  }
  case Calibrated:
  {
    ret = extract(imuData);
    break;
  }
  default:
    break;
  }

  events_.push(event);
  framesProcessed_++;

  return ret;
}

Matrix3x3 GaitFeatureExtractor::pcaR(const Matrix& data, const bool centered, const bool validRotation)
{
  Matrix pc = data;
  if (centered)
    pc = data.rowwise() - data.colwise().mean();

  Eigen::JacobiSVD<Matrix> svd(pc, Eigen::ComputeThinV);
  Matrix3x3 R = svd.matrixV().leftCols(3);
  if (validRotation && R.determinant() < 0)
    R.col(2) = R.col(0).cross(R.col(1));

  return R;
}

void GaitFeatureExtractor::calibrate()
{
  // Convert accelerations and angular velocities into matrices
  Matrix acc = Matrix::Zero(staticIMUData_.size(), 3);
  for (unsigned int f = 0; f < staticIMUData_.size(); ++f)
    acc.row(f) = staticIMUData_[f].acc();

  Matrix gyr = Matrix::Zero(dynamicIMUData_.size(), 3);
  for (unsigned int f = 0; f < dynamicIMUData_.size(); ++f)
    gyr.row(f) = dynamicIMUData_[f].gyr();

  // PCA on static accelerations
  const Matrix3x3 Racc = pcaR(acc, false, true);
  Vector3 gravity = Racc.col(0);
  if ((acc * Racc).col(0).mean() < 0)
    gravity *= -1;

  // PCA on dynamic angular velocities
  const Matrix3x3 Rgyr = pcaR(gyr, false, true);
  Vector3 sagittal = Rgyr.col(0);
  Eigen::Index minIdx, maxIdx;
  const Vector gyrSagittal = (gyr * Rgyr).col(0);
  // Correct the orientation if the first peak in sagittal angular velocity is not positive
  for (unsigned int i = 0; i < gyrSagittal.size(); ++i)
  {
    if (std::abs(gyrSagittal[i]) > 1)
    {
      if (gyrSagittal[i] < 0)
        sagittal *= -1;
      break;
    }
  }

  // Update acc rotation
  accR_.col(2) = gravity; // z
  accR_.col(0) = sagittal.cross(accR_.col(2)); // x
  accR_.col(1) = accR_.col(2).cross(accR_.col(0)); // y
  // Check determinant
  if (accR_.determinant() < 0)
    std::cerr << "Warning: det(accR_) < 0!" << std::endl;

  // Update gyr rotation
  gyrR_.col(1) = sagittal;
  gyrR_.col(0) = gyrR_.col(1).cross(gravity);
  gyrR_.col(2) = gyrR_.col(0).cross(gyrR_.col(1));
  // Check determinant
  if (gyrR_.determinant() < 0)
    std::cerr << "Warning: det(gyrR) < 0!" << std::endl;
}

Vector3 GaitFeatureExtractor::extract(const IMUData& imuData)
{
  // Raw acc and gyr signals
  const Vector3 rawAcc = imuData.acc();
  const Vector3 rawGyr = imuData.gyr();

  // Transform raw signals into accelerometer frame
  const Vector3 accRAcc = accR_.transpose() * rawAcc;
  const Vector3 accRGyr = accR_.transpose() * rawGyr;

  // Track roll and pitch in the accelerometer frame and build a stabilization matrix rotating the raw acceleration into a ground parallel accelerometer frame
  const Vector2 rollPitch = rollPitchTracker_.process(accRAcc, accRGyr);
  const Vector3 aa(rollPitch.x(), rollPitch.y(), 0);
  const Matrix3x3 accStabilizedR = AngleAxis(aa.norm(), aa.normalized()).toRotationMatrix();
  const Vector3 acc = accStabilizedR * accRAcc -Vector3(0, 0, config_.g0);
  const Vector3 gyr = gyrR_.transpose() * rawGyr;

  return Vector3(acc(0), gyr(1), acc(2));
}