#ifndef _ROLL_PITCH_TRACKER_H_
#define _ROLL_PITCH_TRACKER_H_

#include "EigenTypes.h"

class RollPitchTracker
{
public:
  RollPitchTracker(const ScalarType dt = 1.f / 60.f, const ScalarType processNoise = 0.0001f, const ScalarType measurementNoise = 5.f);
  Vector2 process(const Vector3 acc, const Vector3 gyr);

private:
  Matrix pseudoInverse(const Matrix& m);
  Matrix inverse(const Matrix& m);

  Matrix A_, B_, C_, P_, Q_, R_;
  Vector x_;
};

#endif