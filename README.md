# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./docs/UKF-Radar.png "RADAR Tracking on obj_pose-laser-radar-synthetic-input.txt"

## Notes
Current RMSE with just RADAR data used: 
- [0.19, 0.29, 0.48, 0.55] - obj_pose-laser-radar-synthetic-input.txt
- [0.16, 0.21, 0.63, 0.73] - sample-laser-radar-measurement-data-1.txt

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
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

