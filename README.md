# Expressive Gait Features using Lower Limb Inertial Sensor Data
Gait segmentation &amp; event detection based on raw inertial sensor data; paper and C++ feature extraction.
In this paper we trained a simple SVM classifier on top of the engineered features for detecting heel-strike (initial contact) and toe-off (terminal contact) gait events yielding a high accuracy and timeliness w.r.t. ground-truth data obtained from pressure insoles.

## Screencast presented at the IFAC 2020 conference Berlin

[![Screencast Video (15 minutes)](https://seafile.rlp.net/f/379b5bbccbe849d58a5b/?dl=1)](https://seafile.rlp.net/f/b93cc6c354cf4733ac6a/)
https://seafile.rlp.net/f/b93cc6c354cf4733ac6a/

## Paper
Laufer, Felix & Lorenz, Michael & Taetz, Bertram & Bleser, Gabriele. (2020). On Expressive Features for Gait Analysis using Lower Limb Inertial Sensor Data.
https://www.researchgate.net/publication/344807136_On_Expressive_Features_for_Gait_Analysis_using_Lower_Limb_Inertial_Sensor_Data

### Abstract
In this paper, we present a method to obtain explicit, expressive and interpretable
gait feature signals from an inertial sensor, mounted on any segment of the lower limbs.
The proposed method is invariant to the mounting orientation of the sensor, works without
magnetometer information, requires no prior knowledge and can be used in real-time scenarios.
Moreover, the constructed signals are robust for a wide variety of changing walking speeds
and directions. We investigate the informational content of our three feature signals lying in
the human sagittal plane with respect to the gait phase segmentation problem and compare
them to other commonly used signals, such as the sagittal angular velocity and the norms of
accelerations and angular velocities. To this end, we make use of the filter-based maximum
relevance minimum redundancy algorithm, which is a classifier-independent feature selection
method. For validating our approach, we consider gait data of twelve healthy subjects walking
straight and in curves at self-chosen speeds with inertial sensors attached to either the thigh,
shank or foot. Additionally, pressure measuring insoles are used to obtain ground truth toe-off
and heel-strike gait events for reference. With those events as the gait phase transitions, the
event detection is cast into a classification problem. To support the theoretical findings of the
feature selection and ranking, we finally evaluate different choices of feature sets with a simple
linear support vector machine classifier in an online fashion and obtain superior segmentation
results with our feature signals.

### Keywords
gait segmentation, bio-signals analysis and interpretation, human body motion
capture, information and sensor fusion, motion estimation, parameter and state estimation,
inertial sensors, signal processing, machine learning

## Code
Implementation of an online gait feature extractor
### Usage
```cpp
std::vector<IMUData> data = ... ; // some IMU data (accelerometer and gyroscope measurements)
GaitFeatureExtractor extractor;

for (size_t t = 0; t < data.size(); ++t)
{
  if (!extractor.calibrated())
    std::cout << "Warning: extractor is not yet calibrated" << std::endl;
  
  Vector3 feature = extractor.process(data[t]);
}

```

### Requires
- Eigen3
