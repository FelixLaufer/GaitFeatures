#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_

#include "EigenTypes.h"

#include <string>

class IMUData
{
 public:
  IMUData() = default;
  IMUData(const Vector3& acc, const Vector3& gyr, const Vector3& mag = Vector3::Zero(), const uint32_t& t = 0, const std::string& id = "");

  inline const Vector3& acc() const { return acc_; }
  inline const Vector3& gyr() const { return gyr_; }
  inline const Vector3& mag() const { return mag_; }
  inline const uint32_t& getTime() const { return t_; }
  inline const std::string& getID() const { return id_; }

  inline void setAcc(const Vector3& acc) { acc_ = acc; }
  inline void setGyr(const Vector3& gyr) { gyr_ = gyr; }
  inline void setMag(const Vector3& mag) { mag_ = mag; }
  inline void setTime(const uint32_t& t) { t_ = t; }
  inline void setID(const std::string& id) { id_ = id; }

 private:
  Vector3 acc_ = Vector3::Zero();
  Vector3 mag_ = Vector3::Zero();
  Vector3 gyr_ = Vector3::Zero();
  uint32_t t_= 0;
  std::string id_ = "";
};

#endif