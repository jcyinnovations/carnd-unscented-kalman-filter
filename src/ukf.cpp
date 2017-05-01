#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = false;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 3.51;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 7.92;

	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.3;

	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.03; //0.0175

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ = 0.3;   //0.1

	/**
	 TODO:

	 Complete the initialization. See ukf.h for other member properties.

	 Hint: one or more values initialized above might be wildly off...
	 */

	// process state size (number parameters)
	n_x_ = 5;

	// measurement dimension
	n_z_ = 3;

	// augmented process state size (includes process noise parameters)
	n_aug_ = n_x_ + 2;

	n_sig_ = 2 * n_x_ + 1;

	// number of Sigma Points in augmented space
	n_sig_aug_ = 2 * n_aug_ + 1;

	// Sigma Point spreading factor
	lambda_     = (double) (3 - n_x_);

	lambda_aug_ = (double) (3 - n_aug_);

	// UKF Weights for state prediction
	weights_ = VectorXd(n_sig_aug_);
	weights_(0) = lambda_aug_ / (lambda_aug_ + n_aug_);
	double w_n = 1 /(2* (n_aug_ + lambda_aug_));
	for (int i = 1; i < 2 * n_aug_ + 1; i++) {
		weights_(i) = w_n;
	}

	// process state prediction
	Xsig_pred_ = MatrixXd::Zero(n_x_, n_sig_aug_);
	// process state
	x_ = VectorXd(n_x_);
	x_.fill(0.0);

	// process covariance
	P_ = MatrixXd::Zero(n_x_, n_x_);

	tools_ = Tools();
}

UKF::~UKF() {
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	/**
	 TODO:

	 Complete this function! Make sure you switch between lidar and radar
	 measurements.
	 */
	if (!is_initialized_) {
		// first measurement
		time_us_ = meas_package.timestamp_;
		//Initial process noise covariance
		//for (int i=0; i < n_x_; i++) {
		//	P_(i,i) = 0.01;
		//}

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			 * Convert radar from polar to cartesian coordinates and initialize state.
			 */
			x_(0) = meas_package.raw_measurements_(0)
					* cos((double) meas_package.raw_measurements_(1));
			x_(1) = meas_package.raw_measurements_(0)
					* sin((double) meas_package.raw_measurements_(1));
			//x_(2) = meas_package.raw_measurements_(2);
			x_(3) = meas_package.raw_measurements_(1);
			//Rough estimate of vx, vy using components of rho_dot
		} else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			/**
			 * Initialize state.
			 */
			x_(0) = meas_package.raw_measurements_(0);
			x_(1) = meas_package.raw_measurements_(1);
			//x_(3) = atan2((double) meas_package.raw_measurements_(1),(double) meas_package.raw_measurements_(0));
		}

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}
	/**********************************************************
	 * Update the state transition matrix F according to the new
	 * elapsed time.
	 - Time is measured in seconds.
	 * Update the process noise covariance matrix.
	 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	 ************************************************************/
	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;
	Prediction(dt);

	/************************************************************
	 * UPDATE
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	 ************************************************************/
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		if (not (use_radar_))
			return;
		UpdateRadar(meas_package);
	} else {
		// Laser updates
		if (not (use_laser_))
			return;
		UpdateLidar(meas_package);
	}

	// print the output
	cout << "x_ = " << x_ << endl;
	cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	MatrixXd Xsig_out = MatrixXd(n_x_, n_sig_);
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_aug_);

	cout << endl << "____________________________________" << endl;
	//GenerateSigmaPoints(&Xsig_out);
	AugmentedSigmaPoints(&Xsig_aug);
	SigmaPointPrediction(delta_t, Xsig_aug);
	PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
	/**
	 TODO:

	 Complete this function! Use lidar data to update the belief about the object's
	 position. Modify the state vector, x_, and covariance, P_.

	 You'll also need to calculate the lidar NIS.
	 */
	VectorXd z = VectorXd(3);
	z << meas_package.raw_measurements_(0),
			meas_package.raw_measurements_(1),
			0;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	/**
	 TODO:

	 Complete this function! Use radar data to update the belief about the object's
	 position. Modify the state vector, x_, and covariance, P_.

	 You'll also need to calculate the radar NIS.
	 */
	VectorXd z = VectorXd(3);
	z << meas_package.raw_measurements_(0),
			meas_package.raw_measurements_(1),
			meas_package.raw_measurements_(2);
    VectorXd z_out = VectorXd(3);
    MatrixXd Zsig = MatrixXd(3, n_sig_aug_);
    MatrixXd S_out = MatrixXd(3, 3);
    PredictRadarMeasurement(&z_out, &Zsig, &S_out);
    UpdateState(z, Zsig, z_out, S_out);
}

void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

	//create sigma point matrix
	MatrixXd Xsig = MatrixXd::Zero(n_x_, n_sig_);

	//calculate square root of P
	MatrixXd A = P_.llt().matrixL();

	//calculate sigma points
	Xsig.col(0) = x_;
	for (int i = 1; i <= n_x_; i++) {
		MatrixXd factor = sqrt((lambda_ + n_x_)) * A;
		Xsig.col(i) = x_ + factor.col(i - 1);
		Xsig(3, i) = tools_.NormalizePhi((double) Xsig(3, i));

		Xsig.col(i+n_x_) = x_ - factor.col(i - 1);
		Xsig(3, i+n_x_) = tools_.NormalizePhi((double) Xsig(3, i+n_x_));
	}

	*Xsig_out = Xsig;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
	//create augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, n_sig_aug_);

	//create augmented mean state
	x_aug.head(n_x_) = x_;
	x_aug(5) = 0; //std_a_;
	x_aug(6) = 0; //std_yawdd_;

	//create augmented covariance matrix
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(5, 5) = std_a_*std_a_;
	P_aug(6, 6) = std_yawdd_*std_yawdd_;

	//create square root matrix
	MatrixXd A_aug = P_aug.llt().matrixL();
	MatrixXd factor = sqrt(lambda_aug_ + n_aug_) * A_aug;
	//create augmented sigma points
	Xsig_aug.col(0) = x_aug;
	for (int i = 1; i <= n_aug_; i++) {
		Xsig_aug.col(i) = x_aug + factor.col(i - 1);
		Xsig_aug.col(i+n_aug_) = x_aug - factor.col(i - 1);
	}

	*Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(double delta_t, MatrixXd Xsig_aug) {
	//predict sigma points
	//x_ka: px(0), py(1), v(2), psi(3), psi_dot(4), nu_a(5), nu_psi(6)
	for (int i = 0; i < n_sig_aug_; i++) {
		VectorXd x_ka = Xsig_aug.col(i);
		VectorXd x_k1 = VectorXd(5);
		double px = x_ka(0);
		double py = x_ka(1);
		double v = x_ka(2);
		double psi = x_ka(3);
		double psi_dot = x_ka(4);
		double nu_a = x_ka(5);
		double nu_psi = x_ka(6);
		double c1 = psi + psi_dot * delta_t;

		//avoid division by zero
		double delta_t_sq = delta_t*delta_t;
		if (fabs(psi_dot) > 0.001) {
			double p = c1 + delta_t_sq*nu_psi/2;
			//p = tools_.NormalizePhi(p);
			x_k1 <<
				px + (v/psi_dot)*(sin(c1) - sin(psi)) + delta_t_sq*cos(psi)*nu_a/2,
				py + (v/psi_dot)*(cos(psi)- cos(c1))  + delta_t_sq*sin(psi)*nu_a/2,
				v + delta_t*nu_a,
				p,
				psi_dot + delta_t*nu_psi;
		} else {
			double p = psi + delta_t_sq*nu_psi/2;
			//p = tools_.NormalizePhi(p);
			x_k1 <<
				px + v*cos(psi)*delta_t + delta_t_sq*cos(psi)*nu_a/2,
				py + v*sin(psi)*delta_t + delta_t_sq*sin(psi)*nu_a/2,
				v + delta_t*nu_a,
				p,
				psi_dot + delta_t*nu_psi;
		}
		//write predicted sigma points into right column
		Xsig_pred_.col(i) = x_k1;
	}
}

void UKF::PredictMeanAndCovariance() {
	//predict state mean
	//x_ = Xsig_pred_ * weights_;
	x_.fill(0.0);
	for (int i = 0; i < n_sig_aug_; i++) {
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	}

	//predict state covariance matrix
	P_.fill(0.0);
	MatrixXd diff = Xsig_pred_.colwise() - x_;
	for (int i = 0; i < n_sig_aug_; i++) {
		diff(3,i) = tools_.NormalizePhi((double) diff(3,i));
		P_ = P_ + weights_(i) * diff.col(i) * diff.col(i).transpose();
	}
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* Zsig_out, MatrixXd* S_out) {
	//set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 3;

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);

	MatrixXd Zsig = MatrixXd(n_z, n_sig_aug_);

	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_aug_; i++) {
		double px = Xsig_pred_(0, i);
		double py = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double psi = Xsig_pred_(3, i);

		double c1 = sqrt(px*px + py*py);
		Zsig.col(i) <<
				c1,
				atan2(py, px),
				(px*cos(psi)*v + py*sin(psi)*v) / c1;
	}
	//calculate mean predicted measurement
	z_pred.fill(0.0);
	for (int i=0; i < n_sig_aug_; i++) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}

	z_pred = Zsig * weights_;
	//z_pred(1) = tools_.NormalizePhi((double) z_pred(1));

	//calculate measurement covariance matrix S
	MatrixXd z_diff = Zsig.colwise() - z_pred;
	S.fill(0.0);
	//S(0, 0) = std_radr_*std_radr_;
	//S(1, 1) = std_radphi_*std_radphi_;
	//S(2, 2) = std_radrd_*std_radrd_;
	for (int i = 0; i < n_sig_aug_; i++) {
		//angle normalization
		z_diff(1,i) = tools_.NormalizePhi((double) z_diff(1,i));
		S = S + weights_(i)*z_diff.col(i)*z_diff.col(i).transpose();
	}

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z,n_z);
	R <<
		std_radr_*std_radr_, 0, 0,
		0, std_radphi_*std_radphi_, 0,
		0, 0,std_radrd_*std_radrd_;

	S = S + R;

	*z_out = z_pred;
	*S_out = S;
	*Zsig_out = Zsig;
}

void UKF::UpdateState(VectorXd z, MatrixXd Zsig, VectorXd z_pred, MatrixXd S) {

	//set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 3;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

	//calculate cross correlation matrix
	MatrixXd x_diff = Xsig_pred_.colwise() - x_;
	MatrixXd z_diff = Zsig.colwise() - z_pred;

	for (int i = 0; i < n_sig_aug_; i++) {
		//angle normalization
		z_diff(1, i) = tools_.NormalizePhi((double) z_diff(1, i));
		x_diff(3, i) = tools_.NormalizePhi((double) x_diff(3, i));

		Tc = Tc + weights_(i) * x_diff.col(i) * z_diff.col(i).transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//update state mean and covariance matrix
	MatrixXd y = (z - z_pred);
	y(1) = tools_.NormalizePhi((double) y(1));
	x_ = x_ + K * y;
	P_ = P_ - K * S * K.transpose();

	std::cout << "z: " << std::endl << z << std::endl;
	std::cout << "z_pred: " << std::endl << z_pred << std::endl;
	std::cout << "Zsig: " << std::endl << Zsig << std::endl;
	std::cout << "S: " << std::endl << S << std::endl;
	std::cout << "Xsig_pred: " << std::endl << Xsig_pred_ << std::endl;
}
