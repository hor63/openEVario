/*
 * GliderVarioMeasurementUpdater.cpp
 *
 *  Created on: Feb 14, 2016
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016  Kai Horstmann
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License along
 *   with this program; if not, write to the Free Software Foundation, Inc.,
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "GliderVarioMeasurementUpdater.h"

namespace openEV {

static void
GliderVarioMeasurementUpdater::GPSLatitudeUpd (
		FloatType measuredLatitude,
		FloatType latitudeVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
FloatType calculatedValue;
GliderVarioStatus::StatusVectorType measRowT;

	measRowT.setZero();

	// calculate and fill in local variables here.
	measuredLatitude *= 3600.0f;
    measurementVector.gpsLatitude = measuredLatitude;
	measRowT(GliderVarioStatus::STATUS_IND_LATITUDE) = 1.0f;
	calculatedValue = varioStatus.latitude;

	calcSingleMeasureUpdate (
			measuredLatitude,
			calculatedValue,
			latitudeVariance,
			measRowT,
			varioStatus
			);
}

static void
GliderVarioMeasurementUpdater::GPSLongitudeUpd (
		FloatType measuredLongitude,
		FloatType longitudeVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measuredLongitude *= 3600.0f;
		measurementVector.gpsLongitude = measuredLongitude;
		measRowT(GliderVarioStatus::STATUS_IND_LONGITUDE) = 1.0f;
		calculatedValue = varioStatus.longitude;

		calcSingleMeasureUpdate (
				measuredLongitude,
				calculatedValue,
				longitudeVariance,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::GPSAltitudeUpd (
		FloatType measuredAltitudeMSL,
		FloatType altitudeVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;
	FloatType temp1,temp2;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.gpsSpeed = measuredAltitudeMSL;
		calculatedValue =
				sqrt(varioStatus.groundSpeedNorth * varioStatus.groundSpeedNorth +
				     varioStatus.groundSpeedEast * varioStatus.groundSpeedEast);

		// approximate the derivates
		temp1 = varioStatus.groundSpeedNorth*1.01;
		temp2 = sqrt(temp1 * temp1 +
				    varioStatus.groundSpeedEast * varioStatus.groundSpeedEast);
		measRowT(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = (temp2-calculatedValue)/(temp1-varioStatus.groundSpeedNorth);

		// approximate the derivates
		temp1 = varioStatus.groundSpeedEast*1.01;
		temp2 = sqrt(temp1 * temp1 +
				    varioStatus.groundSpeedNorth * varioStatus.groundSpeedNorth);
		measRowT(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = (temp2-calculatedValue)/(temp1-varioStatus.groundSpeedNorth);

		calcSingleMeasureUpdate (
				measuredAltitudeMSL,
				calculatedValue,
				altitudeVariance,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::GPSHeadingUpd (
		FloatType measuredCourseOverGround,
		FloatType courseOverGroundVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.

		calcSingleMeasureUpdate (
				measuredCourseOverGround,
				calculatedValue,
				courseOverGroundVariance,
				measRowT,
				varioStatus
				);
	}

static void
GliderVarioMeasurementUpdater::GPSSpeedUpd (
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
GliderVarioMeasurementUpdater::staticPressureUpd (
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
GliderVarioMeasurementUpdater::dynamicPressureUpd (
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
