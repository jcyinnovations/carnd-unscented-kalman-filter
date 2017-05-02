#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

	///* initially set to false, set to true in first call of ProcessMeasurement
	bool is_initialized_;

	///* if this is false, laser measurements will be ignored (except for init)
	bool use_lidar_;

	///* if this is false, radar measurements will be ignored (except for init)
	bool use_radar_;

	///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
	VectorXd x_;

	///* state covariance matrix
	MatrixXd P_;

	///* predicted sigma points matrix
	MatrixXd Xsig_pred_;

	///* time when the state is true, in us
	long long time_us_;

	///* Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a_;

	///* Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd_;

	///* Laser measurement noise standard deviation position1 in m
	double std_laspx_;

	///* Laser measurement noise standard deviation position2 in m
	double std_laspy_;

	///* Radar measurement noise standard deviation radius in m
	double std_radr_;

	///* Radar measurement noise standard deviation angle in rad
	double std_radphi_;

	///* Radar measurement noise standard deviation radius change in m/s
	double std_radrd_;

	///* Weights of sigma points
	VectorXd weights_;

	///* State dimension
	int n_x_;

	///* Meaurement dimension
	int n_z_;

	///* Augmented state dimension
	int n_aug_;

	///* Number of sigma points
	int n_sig_;

	///* Number of sigma points in augmented space
	int n_sig_aug_;

	///* Sigma point spreading factor for augmented space
	double lambda_aug_;

	///* Sigma point spreading parameter
	double lambda_;

	///* the current NIS for radar
	double NIS_radar_;

	///* the current NIS for laser
	double NIS_lidar_;

	Tools tools_;

	/**
	 * Constructor
	 */
	UKF();

	/**
	 * Destructor
	 */
	virtual ~UKF();

	/**
	 * ProcessMeasurement
	 * @param meas_package The latest measurement data of either radar or laser
	 */
	void ProcessMeasurement(MeasurementPackage meas_package);

	/**
	 * Prediction Predicts sigma points, the state, and the state covariance
	 * matrix
	 * @param delta_t Time between k and k+1 in s
	 */
	void Prediction(double delta_t);

	/**
	 * Updates the state and the state covariance matrix using a laser measurement
	 * @param meas_package The measurement at k+1
	 */
	void UpdateLidar(MeasurementPackage meas_package);

	/**
	 * Updates the state and the state covariance matrix using a radar measurement
	 * @param meas_package The measurement at k+1
	 */
	void UpdateRadar(MeasurementPackage meas_package);

	/**
	 * Set of functions to modularize the UKF process
	 */
private:
	/**
	 * PREDICTION STAGE
	 */

	/**
	 * Generate Sigma Points from the current state to estimate
	 * the non-linear state distribution in a normal distribution
	 * that is compatible with the Kalman Filter process.
	 */
	void GenerateSigmaPoints(MatrixXd* Xsig_out);

	/**
	 * Convert Sigma Points to Augmented Sigma Points to account
	 * for the non-linear process noise components
	 */
	void AugmentedSigmaPoints(MatrixXd* Xsig_out);

	/**
	 * Estimate a prior state in Sigma Points by feeding
	 * Augmented Sigma Points through the non-linear process model
	 */
	void SigmaPointPrediction(double delta_t, MatrixXd Xsig_aug);

	/**
	 * Convert the Sigma Point estimate to a current state estimate
	 */
	void PredictMeanAndCovariance();


	/**
	 * UPDATE STAGE
	 */

	/**
	 * Predict the posterior state based on the current RADAR measurement
	 */
	void PredictRadarMeasurement(VectorXd* z_out, MatrixXd* Zsig_out, MatrixXd* S_out);

	/**
	 * Predict the posterior state based on the current LIDAR measurement
	 */
	void PredictLidarMeasurement(VectorXd* z_out, MatrixXd* Zsig_out, MatrixXd* S_out);

	/**
	 * Update the state using the current prediction and measurement
	 */
	void UpdateState(MeasurementPackage::SensorType sensor, VectorXd z, MatrixXd Zsig, VectorXd z_pred, MatrixXd S);

	/**
	 * Update the state using the current prediction and measurement
	 */
	void UpdateStateLidar(VectorXd z, MatrixXd Zsig, VectorXd z_pred, MatrixXd S);

};

#endif /* UKF_H */
