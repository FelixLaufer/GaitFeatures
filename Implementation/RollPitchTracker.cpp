#include "RollPitchTracker.h"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>

RollPitchTracker::RollPitchTracker(const ScalarType dt, const ScalarType processNoise, const ScalarType measurementNoise)
  : A_(Matrix::Zero(0, 0))
  , B_(Matrix::Zero(0, 0))
  , C_(Matrix::Zero(0, 0))
  , P_(Matrix::Zero(0, 0))
  , Q_(Matrix::Zero(0, 0))
  , R_(Matrix::Zero(0, 0))
  , x_(Vector::Zero(0))
{
  A_.resize(4, 4);
  A_ << 1, -dt, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, -dt,
    0, 0, 0, 1;

  B_.resize(4, 2);
  B_ << dt, 0,
    0, 0,
    0, dt,
    0, 0;

  C_.resize(2, 4);
  C_ << 1, 0, 0, 0,
    0, 0, 1, 0;

  P_ = Matrix::Identity(4, 4);
  Q_ = Matrix::Identity(4, 4) * processNoise;
  R_ = Matrix::Identity(2, 2) * measurementNoise;

  x_ = Vector::Zero(4);
}

Matrix RollPitchTracker::pseudoInverse(const Matrix& m)
{
  if (m.cols() != m.rows())
  {
    if (m.cols() > m.rows())
      return m.transpose() * (m * m.transpose()).fullPivLu().inverse();
    else
      return (m.transpose() * m).fullPivLu().inverse() * m.transpose();
  }

  Eigen::JacobiSVD<Matrix> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Matrix ret(m.cols(), m.rows());
  ret.setZero();

  const Vector w = svd.singularValues();

  const ScalarType sum = w.sum();
  if (sum < 1e-15)
    return ret;

  Matrix diag(w.size(), w.size());
  diag.setZero();
  for (unsigned int i = 0; i < w.size(); i++)
    diag(i, i) = (fabs(w[i] / sum) < 1e-15) ? 0. : 1. / w[i];

  ret = svd.matrixV() * diag.transpose() * svd.matrixU().transpose();
  return ret;
}

Matrix RollPitchTracker::inverse(const Matrix& m)
{
  if (m.size() == 1)
    return (m.array().inverse()).eval();

  Eigen::FullPivLU<Matrix> lu(m);
  if (lu.isInvertible())
    return lu.inverse();
  
  return pseudoInverse(m);
}

Vector2 RollPitchTracker::process(const Vector3 acc, const Vector3 gyr)
{
  const ScalarType& roll = x_(0);
  const ScalarType& pitch = x_(2);

  // Predict
  const ScalarType gx = gyr.x();
  const ScalarType gy = gyr.y();
  const ScalarType gz = gyr.z();
  const ScalarType phiDot = gx + std::sin(roll) * std::tan(pitch) * gy + std::cos(roll) * std::tan(pitch) * gz;
  const ScalarType thetaDot = std::cos(roll) * gy - std::sin(roll) * gz;

  x_ = A_ * x_ + B_ * Vector2(phiDot, thetaDot);
  P_ = A_ * P_ * A_.transpose() + Q_;

  // Update
  const ScalarType ax = acc.x();
  const ScalarType ay = acc.y();
  const ScalarType az = acc.z();
  const ScalarType phiHatAcc = std::atan2(ay, std::sqrt(ax * ax + az * az));
  const ScalarType thetaHatAcc = std::atan2(-ax, std::sqrt(ay * ay + az * az));

  const Vector yTilde = Vector2(phiHatAcc, thetaHatAcc) - C_ * x_;
  const Matrix PCT = P_ * C_.transpose();
  const Matrix S = R_ + C_ * PCT;
  const Matrix K = PCT * MatrixUtils::safeInverse(S);
  x_ = x_ + K * yTilde;
  P_ = (Matrix::Identity(4, 4) - K * C_) * P_;

  return Vector2(roll, pitch);
}