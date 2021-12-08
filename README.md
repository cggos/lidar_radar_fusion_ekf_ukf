# Lidar & Radar Fusion

* based on [mithi/fusion-ekf](https://github.com/mithi/fusion-ekf), and its related python code is [mithi/fusion-ekf-python](https://github.com/mithi/fusion-ekf-python)

  - [Sensor Fusion and Object Tracking using an Extended Kalman Filter Algorithm — Part 1](https://medium.com/@mithi/object-tracking-and-fusing-sensor-measurements-using-the-extended-kalman-filter-algorithm-part-1-f2158ef1e4f0)

  - [Sensor Fusion and Object Tracking using an Extended Kalman Filter Algorithm — Part 2
  ](https://medium.com/@mithi/sensor-fusion-and-object-tracking-using-an-extended-kalman-filter-algorithm-part-2-cd20801fbeff)

* [udacity/CarND-Extended-Kalman-Filter-Project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project): Self-Driving Car Nanodegree Program Starter Code for the Extended Kalman Filter Project

* [Sensor Fusion (towardsautonomy)](http://www.towardsautonomy.com/sensor_fusion)

* PDF Doc: ***Self-Driving Car ND - Sensor Fusion - Extended Kalman Filters***

-----

[TOC]

## Introduction

This is an **extended Kalman Filter** implementation in C++ for **fusing lidar and radar sensor measurements**.

In this case, we have two 'noisy' sensors:
- A lidar sensor that measures our position in cartesian-coordinates `(x, y)`
- A radar sensor that measures our position and relative velocity (the velocity within line of sight) in polar coordinates `(rho, phi, drho)`

We want to predict our position, and how fast we are going in what direction at any point in time:
- In essence: the position and velocity of the system in cartesian coordinates: `(x, y, vx, vy)`
- NOTE: We are assuming a **constant velocity model (CV)** for this particular system

Note:
- The **measurement covariance matrices** `lidar_R` and `radar_R` are hard-coded in `src/fusionekf.cpp`.
- The **initial state covariance matrix** `P` is hard-coded in `src/fusionekf.cpp`.
- The **process 2d noise** `ax` and `ay` used to **update the process covariance matrix** `Q` is hard-coded in `include/fusionekf.h`.


## Build

```sh
make build & cd build
cmake .. & make -j3
```

## Run

* run with datafile
  ```
  ./ExtendedKF ../data/data-1.txt output.txt
  ```

* Please use the following format for your input file:
  ```
  L(for lidar) m_x m_y t r_x r_y r_vx r_vy
  R(for radar) m_rho m_phi m_drho t r_px r_py r_vx r_vy

  Where:
  (m_x, m_y) - measurements by the lidar
  (m_rho, m_phi, m_drho) - measurements by the radar in polar coordinates
  (t) - timestamp in unix/epoch time the measurements were taken
  (r_x, r_y, r_vx, r_vy) - the real ground truth state of the system

  Example:
  R 8.60363 0.0290616 -2.99903  1477010443399637  8.6 0.25  -3.00029  0
  L 8.45  0.25  1477010443349642  8.45  0.25  -3.00027  0
  ```

* The program outputs the predictions in the following format on the output file path you specified
  ```
  p_x p_y p_vx p_vy m_x m_y r_px r_py r_vx r_vy

  Where:
  (p_x, p_y, p_vx, p_vy) - the predicted state of the system by FusionEKF
  (m_x, m_y) - the position value as measured by the sensor converted to cartesian coordinates
  (r_x, r_y, r_vx, r_vy) - the real ground truth state of the system

  Example:
  4.53271 0.279 -0.842172 53.1339 4.29136 0.215312  2.28434 0.226323
  43.2222 2.65959 0.931181  23.2469 4.29136 0.215312  2.28434 0.226323
  ```

* draw the position data from `output.txt`
  
  - input `data-1.txt`

    <p align="center">
      <img src="./imgs/lidar_radar_fusion_output1.png" style="width:100%;"/>
    </p>

  - input `data-2.txt`

    <p align="center">
      <img src="./imgs/lidar_radar_fusion_output2.png" style="width:100%;"/>
    </p>  
