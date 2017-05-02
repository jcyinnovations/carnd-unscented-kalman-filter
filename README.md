# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./docs/UKF-Radar.png "RADAR Tracking on obj_pose-laser-radar-synthetic-input.txt"

## Notes
Current RMSE with RADAR data only: 
- [0.19, 0.29, 0.48, 0.55] - obj_pose-laser-radar-synthetic-input.txt
- [0.16, 0.21, 0.63, 0.73] - sample-laser-radar-measurement-data-1.txt

Current RMSE with LIDAR data only: 
- [0.12, 0.12, 0.75, 0.57] - obj_pose-laser-radar-synthetic-input.txt
- [0.03, 0.03, 0.59, 0.52] - sample-laser-radar-measurement-data-1.txt

Current RMSE with BOTH sensors: 
- [0.08, 0.09, 0.42, 0.40] - obj_pose-laser-radar-synthetic-input.txt
- [0.04, 0.06, 0.45, 0.51] - sample-laser-radar-measurement-data-1.txt

The filter operation was dramatically improved by setting the standard deviation values for longitudinal accelaration `a` and yaw acceleration `psi dot` using empirical values based om the ground truth data. 
The new values are:
- std `a` = 3.51
- std `psi dot` = 7.92

Further experiementation is required to refine the process noise parameters.
=======
![alt text][image1]

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`ukf.log >trace.log

