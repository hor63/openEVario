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
#include "FastMath.h"
#include "RotationMatrix.h"

namespace openEV {

void
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

void
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

void
GliderVarioMeasurementUpdater::GPSAltitudeUpd (
		FloatType measuredAltitudeMSL,
		FloatType altitudeVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.gpsMSL = measuredAltitudeMSL;
		measRowT(GliderVarioStatus::STATUS_IND_ALT_MSL) = 1.0f;

		calcSingleMeasureUpdate (
				measuredAltitudeMSL,
				calculatedValue,
				altitudeVariance,
				measRowT,
				varioStatus
				);
	}

void
GliderVarioMeasurementUpdater::GPSHeadingUpd (
		FloatType measuredCourseOverGround,
		FloatType courseOverGroundVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;
	FloatType temp1;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.gpsHeading = measuredCourseOverGround;
		calculatedValue = FastMath::fastATan2(varioStatus.groundSpeedEast,varioStatus.groundSpeedNorth);

		// approximate the derivates
		// to avoid numeric issues use the same increment for both directions
		temp1 = (varioStatus.groundSpeedNorth + varioStatus.groundSpeedEast) / 100.0f;

		measRowT(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) =
				FastMath::fastATan2(varioStatus.groundSpeedEast,varioStatus.groundSpeedNorth + temp1) / temp1;
		measRowT(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E) =
				FastMath::fastATan2(varioStatus.groundSpeedEast + temp1,varioStatus.groundSpeedNorth) / temp1;

		calcSingleMeasureUpdate (
				measuredCourseOverGround,
				calculatedValue,
				courseOverGroundVariance,
				measRowT,
				varioStatus
				);
	}

void
GliderVarioMeasurementUpdater::GPSSpeedUpd (
		FloatType measuredSpeedOverGround,
		FloatType speedOverGroundVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;
	FloatType temp1, temp2, temp3;
	FloatType groundSpeedNSquare = varioStatus.groundSpeedNorth * varioStatus.groundSpeedNorth;
	FloatType groundSpeedESquare = varioStatus.groundSpeedEast  * varioStatus.groundSpeedEast;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measuredSpeedOverGround *= NM_TO_M / 3600.0f;
		measurementVector.gpsSpeed = measuredSpeedOverGround;
		calculatedValue = sqrt(groundSpeedNSquare + groundSpeedESquare);

		// approximate the derivates
		// use the same increment for both directions to avoid numerical resolution problems
		temp3 = calculatedValue / 100.0f;

		temp1 = varioStatus.groundSpeedNorth + temp3;
		temp2 = sqrt(temp1 * temp1 + groundSpeedESquare);
		measRowT(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = (temp2-calculatedValue) / temp3;

		temp1 = varioStatus.groundSpeedEast + temp3;
		temp2 = sqrt(groundSpeedNSquare + temp1 * temp1);
		measRowT(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = (temp2-calculatedValue) / temp3;


		calcSingleMeasureUpdate (
				measuredSpeedOverGround,
				calculatedValue,
				speedOverGroundVariance,
				measRowT,
				varioStatus
				);
	}


void
GliderVarioMeasurementUpdater::accelXUpd (
		FloatType measuredAccelX,
		FloatType accelXVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.accelX = measuredAccelX;
		calculatedValue = varioStatus.accelX;
		measRowT(GliderVarioStatus::STATUS_IND_ACC_X) = 1.0f;

		calcSingleMeasureUpdate (
				measuredAccelX,
				calculatedValue,
				accelXVariance,
				measRowT,
				varioStatus
				);
	}

void
GliderVarioMeasurementUpdater::accelYUpd (
		FloatType measuredAccelY,
		FloatType accelYVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;
	FloatType turnRateRad, turnRadius, bankAngleRot, staticAngle;

	    // First run: update of the Y acceleration
		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.accelY = measuredAccelY;
		calculatedValue = varioStatus.accelY;
		measRowT(GliderVarioStatus::STATUS_IND_ACC_Y) = 1.0f;

		calcSingleMeasureUpdate (
				measuredAccelY,
				calculatedValue,
				accelYVariance,
				measRowT,
				varioStatus
				);

	}

void
GliderVarioMeasurementUpdater::accelZUpd (
		FloatType measuredAccelZ,
		FloatType accelZVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.accelZ = measuredAccelZ;
		calculatedValue = varioStatus.accelZ;
		measRowT(GliderVarioStatus::STATUS_IND_ACC_Z) = 1.0f;

		calcSingleMeasureUpdate (
				measuredAccelZ,
				calculatedValue,
				accelZVariance,
				measRowT,
				varioStatus
				);
	}


void
GliderVarioMeasurementUpdater::gyroXUpd (
		FloatType measuredRollRateX,
		FloatType rollRateXVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.gyroRateX = measuredRollRateX;
		calculatedValue = varioStatus.rollRateX + varioStatus.gyroBiasX;
		measRowT(GliderVarioStatus::STATUS_IND_ROTATION_X) = 1.0f;
		measRowT(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = 1.0f;

		calcSingleMeasureUpdate (
				measuredRollRateX,
				calculatedValue,
				rollRateXVariance,
				measRowT,
				varioStatus
				);
	}

void
GliderVarioMeasurementUpdater::gyroYUpd (
		FloatType measuredPitchRateY,
		FloatType rollRateYVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.gyroRateY = measuredPitchRateY;
		calculatedValue = varioStatus.yawRateZ + varioStatus.gyroBiasY;
		measRowT(GliderVarioStatus::STATUS_IND_ROTATION_Y) = 1.0f;
		measRowT(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = 1.0f;

		calcSingleMeasureUpdate (
				measuredPitchRateY,
				calculatedValue,
				rollRateYVariance,
				measRowT,
				varioStatus
				);

	}

void
GliderVarioMeasurementUpdater::gyroZUpd (
		FloatType measuredYawRateZ,
		FloatType rollRateZVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.gyroRateZ = measuredYawRateZ;
		calculatedValue = varioStatus.yawRateZ + varioStatus.gyroBiasZ;
		measRowT(GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1.0f;
		measRowT(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = 1.0f;

		calcSingleMeasureUpdate (
				measuredYawRateZ,
				calculatedValue,
				rollRateZVariance,
				measRowT,
				varioStatus
				);


	}

void
GliderVarioMeasurementUpdater::compassUpd (
		FloatType measuredMagFlowX,
		FloatType measuredMagFlowY,
		FloatType measuredMagFlowZ,
		FloatType magFlowXVariance,
		FloatType magFlowYVariance,
		FloatType magFlowZVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	FloatType calculatedValue;
	GliderVarioStatus::StatusVectorType measRowT;
	FloatType temp;

	// Compensated magnetic flow, i.e. measurement - deviation flow vector
	FloatType magFlowCompensatedX = measuredMagFlowX - varioStatus.compassDeviationX;
	FloatType magFlowCompensatedY = measuredMagFlowY - varioStatus.compassDeviationY;
	FloatType magFlowCompensatedZ = measuredMagFlowZ - varioStatus.compassDeviationZ;

	// The absolute strength of the compensated magnetic flow, i.e. the vector length
	FloatType compensatedMagVectorLength = sqrtf(
			magFlowCompensatedX * magFlowCompensatedX +
			magFlowCompensatedY * magFlowCompensatedY +
			magFlowCompensatedZ * magFlowCompensatedZ
			);

	// The rotation matrix of the magnetic flow vector (unit vector turned by declination and inclination)
	RotationMatrix magRotMatrix              (varioStatus.magneticDeclination       ,varioStatus.magneticInclination       ,0.0f);
	// The same rotation matrix with incremments to declination and inclination to assess the Jacobian
	RotationMatrix magRotMatrixIncDeclination(varioStatus.magneticDeclination + 1.0f,varioStatus.magneticInclination       ,0.0f);
	RotationMatrix magRotMatrixIncInclination(varioStatus.magneticDeclination       ,varioStatus.magneticInclination + 1.0f,0.0f);

	// Rotation matrix of the plane attitude to project the magnetic vector to the plane measurements
	RotationMatrix attitudeRotMatrix         (varioStatus.heading       ,varioStatus.pitchAngle       ,varioStatus.rollAngle);
	// Increment plane attitude rotation matrixes to assess the Jacobians for the attitude angles
	RotationMatrix attitudeRotMatrixIncYaw   (varioStatus.heading + 1.0f,varioStatus.pitchAngle       ,varioStatus.rollAngle);
	RotationMatrix attitudeRotMatrixIncPitch (varioStatus.heading       ,varioStatus.pitchAngle + 1.0f,varioStatus.rollAngle);
	RotationMatrix attitudeRotMatrixIncRoll  (varioStatus.heading       ,varioStatus.pitchAngle       ,varioStatus.rollAngle + 1.0f);

	// The resulting rotation matrix from unit vector to the magnetic vector as seen in the plane
	RotationMatrix3DType compassMatrix = magRotMatrix.getMatrixPlaneToGlo() * attitudeRotMatrix.getMatrixGloToPlane();
	// The resulting rotation matrixes with the 5 increments
	RotationMatrix3DType compassMatrixIncDeclination = magRotMatrixIncDeclination.getMatrixPlaneToGlo() * attitudeRotMatrix.getMatrixGloToPlane();
	RotationMatrix3DType compassMatrixIncInclination = magRotMatrixIncInclination.getMatrixPlaneToGlo() * attitudeRotMatrix.getMatrixGloToPlane();
	RotationMatrix3DType compassMatrixIncYaw  = magRotMatrix.getMatrixPlaneToGlo() * attitudeRotMatrixIncYaw.getMatrixGloToPlane();
	RotationMatrix3DType compassMatrixIncPitch = magRotMatrix.getMatrixPlaneToGlo() * attitudeRotMatrixIncPitch.getMatrixGloToPlane();
	RotationMatrix3DType compassMatrixIncRoll  = magRotMatrix.getMatrixPlaneToGlo() * attitudeRotMatrixIncRoll.getMatrixGloToPlane();

	Vector3DType magVecLength (compensatedMagVectorLength,0.0f,0.0f);

	// The calculated vector of magnetic measurements.
	Vector3DType compassVector               = compassMatrix               * magVecLength;

	// Vector of compensated magnetic flows as calculated from the current attitude, inclination, and declination
	Vector3DType compassVectorIncDeclination = compassMatrixIncDeclination * magVecLength;
	// Variations of the vector with increments of the 5 paticipating factors.
	Vector3DType compassVectorIncInclination = compassMatrixIncInclination * magVecLength;
	Vector3DType compassVectorIncYaw         = compassMatrixIncYaw         * magVecLength;
	Vector3DType compassVectorIncPitch       = compassMatrixIncPitch       * magVecLength;
	Vector3DType compassVectorIncRoll        = compassMatrixIncRoll        * magVecLength;

		measurementVector.magX = measuredMagFlowX;
		measurementVector.magY = measuredMagFlowY;
		measurementVector.magZ = measuredMagFlowZ;

		measRowT.setZero();

		// calculate and fill in local variables here.
		temp = compassVector(0);
		calculatedValue = temp + varioStatus.compassDeviationX;
		measRowT(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X)  =  1.0f;
		measRowT(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION) = compassVectorIncDeclination(0) - temp;
		measRowT(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION) = compassVectorIncInclination(0) - temp;
		measRowT(GliderVarioStatus::STATUS_IND_HEADING)              = compassVectorIncYaw(0)         - temp;
		measRowT(GliderVarioStatus::STATUS_IND_PITCH)                = compassVectorIncPitch(0)       - temp;
		measRowT(GliderVarioStatus::STATUS_IND_ROLL)                 = compassVectorIncRoll(0)        - temp;

		calcSingleMeasureUpdate (
				measuredMagFlowX,
				calculatedValue,
				magFlowXVariance,
				measRowT,
				varioStatus
				);

		measRowT.setZero();

		// calculate and fill in local variables here.
		temp = compassVector(1);
		calculatedValue = temp + varioStatus.compassDeviationY;
		measRowT(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y)  =  1.0f;
		measRowT(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION) = compassVectorIncDeclination(1) - temp;
		measRowT(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION) = compassVectorIncInclination(1) - temp;
		measRowT(GliderVarioStatus::STATUS_IND_HEADING)              = compassVectorIncYaw(1)         - temp;
		measRowT(GliderVarioStatus::STATUS_IND_PITCH)                = compassVectorIncPitch(1)       - temp;
		measRowT(GliderVarioStatus::STATUS_IND_ROLL)                 = compassVectorIncRoll(1)        - temp;

		calcSingleMeasureUpdate (
				measuredMagFlowY,
				calculatedValue,
				magFlowYVariance,
				measRowT,
				varioStatus
				);

		measRowT.setZero();

		// calculate and fill in local variables here.
		temp = compassVector(2);
		calculatedValue = temp + varioStatus.compassDeviationZ;
		measRowT(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z)  =  1.0f;
		measRowT(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION) = compassVectorIncDeclination(2) - temp;
		measRowT(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION) = compassVectorIncInclination(2) - temp;
		measRowT(GliderVarioStatus::STATUS_IND_HEADING)              = compassVectorIncYaw(2)         - temp;
		measRowT(GliderVarioStatus::STATUS_IND_PITCH)                = compassVectorIncPitch(2)       - temp;
		measRowT(GliderVarioStatus::STATUS_IND_ROLL)                 = compassVectorIncRoll(2)        - temp;

		calcSingleMeasureUpdate (
				measuredMagFlowZ,
				calculatedValue,
				magFlowYVariance,
				measRowT,
				varioStatus
				);
	}


void
GliderVarioMeasurementUpdater::staticPressureUpd (
		FloatType measuredStaticPressure,
		FloatType measuredTemperature,
		FloatType staticPressureVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	GliderVarioStatus::StatusVectorType measRowT;

	static FloatType constexpr tempLapse = -0.01;      // -1C/100m
	static FloatType constexpr R         = 8.3144598f;   // 8.3144598  J /mol/K
	static FloatType constexpr Rspec     = 287.058f;     // Specific R for dry air
	static FloatType constexpr M         = 0.0289644f; // 0.0289644 kg/mol
	static FloatType constexpr ex        = GRAVITY * M / R / tempLapse;
	FloatType pFactor;
	FloatType p;
	FloatType p1;


		measRowT.setZero();

		// calculate and fill in local variables here.
		measurementVector.staticPressure = measuredStaticPressure;

		// to Kelvin
		measuredTemperature += 273.15f;

		// This is used to calculate the pressure and at the same time the derivate for Qff.
		pFactor = powf ((measuredTemperature - (tempLapse * varioStatus.altMSL) / measuredTemperature),ex);
		// The pressure at the height in the dry indifferent boundary layer.
		p = varioStatus.qff * pFactor;
		// The pressure 10m above to assess the derivate for altitude deviations
		p1 = varioStatus.qff * powf ((measuredTemperature - (tempLapse * (varioStatus.altMSL + 10)) / measuredTemperature),ex);

		measRowT(GliderVarioStatus::STATUS_IND_QFF) = pFactor;
		measRowT(GliderVarioStatus::STATUS_IND_ALT_MSL) = (p1 - p) / 10.0f;

		calcSingleMeasureUpdate (
				measuredStaticPressure,
				p,
				staticPressureVariance,
				measRowT,
				varioStatus
				);

		// store the calculated pressure for later use for determining the dynamic pressure from TAS
		varioStatus.lastPressure = p;
	}

void
GliderVarioMeasurementUpdater::dynamicPressureUpd (
		FloatType measuredDynamicPressure,
		FloatType measuredTemperature,
		FloatType dynamicPressureVariance,
		GliderVarioMeasurementVector const &measurementVector,
		GliderVarioStatus &varioStatus
		) {
	static FloatType constexpr RspecTimes2     = 287.058 * 2.0f;     // Specific R for dry air

	// This term is used repeatedly
	FloatType pressRspecTemp = varioStatus.lastPressure / RspecTimes2 / (measuredTemperature + 273.15f);
	FloatType tmp1;
	FloatType dynPressure;
	GliderVarioStatus::StatusVectorType measRowT;

		measRowT.setZero();

		// calculate and fill in local variables here.
		// Develop the dynamic pressure gradually to get the derivates of the variables most efficiently
		// dyn pressure = 0.5 * density * speed * speed
		// dyn pressure = 0.5 * (pressure / Rspec /temp) * speed * speed
		tmp1 = pressRspecTemp * varioStatus.trueAirSpeed;
		dynPressure = tmp1 * varioStatus.trueAirSpeed;
		// True derivate
		measRowT(GliderVarioStatus::STATUS_IND_TAS) = tmp1 * 0.5f;


		calcSingleMeasureUpdate (
				measuredDynamicPressure,
				dynPressure,
				dynamicPressureVariance,
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
