# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./docs/UKF-Radar.png "RADAR Tracking on obj_pose-laser-radar-synthetic-input.txt"
[image2]: ./docs/Position-Estimate-Fused.png "Position Estimate from Fused Sensors"
[image3]: ./docs/NIS-Lidar.png "Normalized Innovation for LIDAR estimates"
[image4]: ./docs/NIS-RADAR.png "Normalized Innovation for RADAR estimates"
[image5]: ./docs/Velocity-Estimate-Fused.png "Velocity Estimate from Fused Sensors"
[image6]: ./docs/UKF-Fused-Sample1.png "Fused Sensor Tracking on sample-laser-radar-measurement-data-1.txt"
[image7]: ./docs/NIS-RADAR-Sample1.png "NIS for RADAR on Sample 1"

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

## Update (5/2/2017)
The filter performance was improved by reviewing the ground truth data and eliminating outliers before re-evaluating its logits. The resulting logits are:
- std `a` (acceleration std) = 3.51
- std `psi dot` (yaw rate std) = 0.356

This resulted in lower overall RMSE as shown below:
- [0.08, 0.09, 0.37, 0.26] - obj_pose-laser-radar-synthetic-input.txt
- [0.12, 0.14, 0.65, 0.69] - sample-laser-radar-measurement-data-1.txt

As you can see, the first sample meets the required target whereas the second, older sample does not.

After tuning the `yaw rate std`, I was able to improve both error rates as follows:
- [0.07, 0.09, 0.36, 0.25] - obj_pose-laser-radar-synthetic-input.txt
- [0.08, 0.09, 0.59, 0.61] - sample-laser-radar-measurement-data-1.txt

The yaw rate STD value was increased to 0.55 to effect this result.

The filter performance was evaluated throughout this process to ensure good behaviour, using NIS. The final NIS plots are shown below:

![alt text][image3]
![alt text][image4]

The resulting position estimates are show below

### Obj Data (Newer Dataset)
![alt text][image2]

### Sample 1 Data (Older Dataset)
![alt text][image6]

## Conclusions
The target RMSE is met for both the old and new datasets.

Even though the RMSE meets targets for both the old and new datasets, the tracking can still be improved on Sample 1. 
The source of the drift is in the filter setup for RADAR as can be seen for the NIS for this sensor on the Sample 1 dataset (below).

![alt text][image7]

Improving RADAR tracking for Sample 1 would require further tuning of the process noise.

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

