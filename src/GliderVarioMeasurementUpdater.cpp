/*
 * GliderVarioMeasurementMatrix.cpp
 *
 *  Created on: Feb 14, 2016
 *      Author: hor
 */

#include <GliderVarioMeasurementUpdater.h>

namespace openEV {

static void
GliderVarioMeasurementUpdater::gpsLatitudeUpd (
		FloatType measuredLatitude,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
FloatType measuredValue;
FloatType calculatedValue;
FloatType measurementVariance_R;
GliderVarioStatus::StatusVectorType measRowT;

	measRowT.setZero();

	// calculate and fill in local variables here.

	calcSingleMeasureUpdate (
			measuredValue,
			calculatedValue,
			measurementVariance_R,
			measRowT,
			varioStatus
			);
}

static void
GliderVarioMeasurementUpdater::gpsLongitudeUpd (
		FloatType measuredLongitude,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::gpsAltitudeUpd (
		FloatType measuredAltitudeMSL,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::gpsHeadingUpd (
		FloatType measuredCourseOverGround,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::gpsSpeedUpd (
		FloatType measuredSpeedOverGround,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}


static void
GliderVarioMeasurementUpdater::accelXUpd (
		FloatType measuredAccelX,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::accelYUpd (
		FloatType measuredAccelY,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::accelZUpd (
		FloatType measuredAccelZ,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::gyroXUpd (
		FloatType measuredRollRateX,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::gyroYUpd (
		FloatType measuredPitchRateY,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::gyroZUpd (
		FloatType measuredYawRateZ,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::compassUpd (
		FloatType measuredMagFlowX,
		FloatType measuredMagFlowY,
		FloatType measuredMagFlowZ,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}


static void
GliderVarioMeasurementUpdater::pressureAltUpd (
		FloatType measuredStaticPressure,
		FloatType measuredTemperature,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::trueAirSpeedUpd (
		FloatType measuredDynamicPressure,
		FloatType measuredTemperature,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType measuredValue;
	FloatType calculatedValue;
	FloatType measurementVariance_R;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.


		calcSingleMeasureUpdate (
				measuredValue,
				calculatedValue,
				measurementVariance_R,
				measRowT,
				varioStatus
				);
	}


void
GliderVarioMeasurementUpdater::calcSingleMeasureUpdate (
		FloatType measuredValue,
		FloatType calculatedValue,
		FloatType measurementVariance_R,
		GliderVarioStatus::StatusVectorType const &measRowT,
		GliderVarioStatus &varioStatus
		) {
	GliderVarioStatus::StatusCoVarianceType &coVariance_P = varioStatus.getErrorCovariance_P();
	GliderVarioStatus::StatusVectorType &statusVector_x = varioStatus.getStatusVector_x();

	GliderVarioStatus::StatusVectorType kalmanGain_K;
	FloatType denominator;

	// Intermediate because a term is used twice
	Eigen::Matrix<FloatType,1,GliderVarioStatus::STATUS_NUM_ROWS> hTimesP;
	// Intermediate storage to avoid side effects
	GliderVarioStatus::StatusVectorType newState;
	GliderVarioStatus::StatusCoVarianceType newCoVariance;

	hTimesP = measRowT.transpose() * coVariance_P;

	denominator = hTimesP * measRowT + measurementVariance_R;

	kalmanGain_K = coVariance_P * measRowT;
	kalmanGain_K /= denominator;

	// Use a temporary variable to avoid side effects when calculating the state based on the previous status.
	newState = statusVector_x + kalmanGain_K * (measuredValue - calculatedValue);
	statusVector_x.operator = (newState);

	newCoVariance = coVariance_P - kalmanGain_K * hTimesP;

	coVariance_P.operator = (newCoVariance);

}


} /* namespace openEV */
