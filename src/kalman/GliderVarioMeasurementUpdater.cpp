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

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include "GliderVarioMeasurementUpdater.h"
#include "util/FastMath.h"

#if defined HAVE_LOG4CXX_H

static log4cxx::LoggerPtr logger = 0;

// Helper class to initiate the logger pointer since GliderVarioMeasurementUpdater is a pure static helper class which does not have a constructor.
class GliderVarioMeasurementUpdaterLogger {

private:

	GliderVarioMeasurementUpdaterLogger() {
		if (logger == 0) {
			logger = log4cxx::Logger::getLogger("openEV.Kalman.GliderVarioMeasurementUpdater");
		}
	}

	static GliderVarioMeasurementUpdaterLogger theOneAndOnly;
};

GliderVarioMeasurementUpdaterLogger GliderVarioMeasurementUpdaterLogger::theOneAndOnly;

#endif  // defined HAVE_LOG4CXX_H

namespace openEV {

// This stuff is used only for unit tests.
// These variables contain copies of the local variables used for the measurement updates.
// These variables are not declared and used in production code.
FloatType GliderVarioMeasurementUpdater::calculatedValueTst1 = 0.0f;
Eigen::SparseMatrix<FloatType> GliderVarioMeasurementUpdater::measRowTTst1;

FloatType GliderVarioMeasurementUpdater::calculatedValueTst2 = 0.0f;
Eigen::SparseMatrix<FloatType> GliderVarioMeasurementUpdater::measRowTTst2;

FloatType GliderVarioMeasurementUpdater::calculatedValueTst3 = 0.0f;
Eigen::SparseMatrix<FloatType> GliderVarioMeasurementUpdater::measRowTTst3;

bool GliderVarioMeasurementUpdater::unitTestMode = false;

void
GliderVarioMeasurementUpdater::GPSPositionUpd (
        double measuredLatitude,
        double measuredLongitude,
        FloatType latitudeVariance,
        FloatType longitudeVariance,
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
	enum ColIndex {
		latIndex = 0,
		lonIndex = 1,
	};
	Vector2DType calculatedValue;
	Matrix2DType varianceMatrix  {{latitudeVariance,0.0f},{0.0f,longitudeVariance}};
    Eigen::SparseMatrix<FloatType> measRowT(GliderVarioStatus::STATUS_NUM_ROWS,2);

    measRowT.reserve(GliderVarioStatus::STATUS_NUM_ROWS);

    // calculate and fill in local variables here.
    measuredLatitude *= 3600.0; // to arc seconds
    measurementVector.gpsLatitude = measuredLatitude;
    measuredLatitude = (measuredLatitude - double(varioStatus.getLatitudeBaseArcSec())) * LEN_LAT_ARC_SEC;
    measRowT.insert(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,latIndex) = 1.0f;
    calculatedValue(latIndex) = varioStatus.latitudeOffsC;

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = calculatedValue(latIndex);
    }

    measuredLongitude *= 3600.0; // to arc seconds
    measurementVector.gpsLongitude = measuredLongitude;
    measuredLongitude = (measuredLongitude - varioStatus.getLongitudeBaseArcSec()) * varioStatus.getLenLongitudeArcSec();
    measRowT.insert(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,lonIndex) = 1.0f;
    calculatedValue(lonIndex) = varioStatus.longitudeOffsC;


    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst2 = calculatedValue(lonIndex);
        measRowTTst1 = measRowT;
    }

    LOG4CXX_DEBUG(logger,"GPSLatitudeUpd: measured latitudeOffset(m) = " <<  measuredLatitude
    		<< ", calculated latitudeOffset = " << calculatedValue << ", variance = " << latitudeVariance);

	Vector2DType measuredValue (measuredLatitude,measuredLongitude);

    calc2DMeasureUpdate (
            measuredValue,
            calculatedValue,
			varianceMatrix,
            measRowT,
            varioStatus
    );
}

void
GliderVarioMeasurementUpdater::GPSAltitudeUpd (
        FloatType measuredAltitudeMSL,
        FloatType altitudeVariance,
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
    FloatType calculatedValue;
    Eigen::SparseMatrix<FloatType> measRowT(GliderVarioStatus::STATUS_NUM_ROWS,1);

    measRowT.reserve(GliderVarioStatus::STATUS_NUM_ROWS);

    // calculate and fill in local variables here.
    measurementVector.gpsMSL = measuredAltitudeMSL;
    measRowT.insert(GliderVarioStatus::STATUS_IND_ALT_MSL,0) = 1.0f;
    calculatedValue = varioStatus.altMSL;

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = calculatedValue;
        measRowTTst1 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredAltitude = " <<  measuredAltitudeMSL
    		<< ", calculated Altitude = " << calculatedValue << ", variance = " << altitudeVariance);

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
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
    FloatType calculatedValue;
    Eigen::SparseMatrix<FloatType> measRowT(GliderVarioStatus::STATUS_NUM_ROWS,1);
    FloatType temp1;

    measRowT.reserve(GliderVarioStatus::STATUS_NUM_ROWS);

    // calculate and fill in local variables here.
    measurementVector.gpsHeading = measuredCourseOverGround;
    calculatedValue = FastMath::fastATan2(varioStatus.groundSpeedEast,varioStatus.groundSpeedNorth);

    // approximate the derivates
    // to avoid numeric issues use the same increment for both directions
    temp1 = (fabsf(varioStatus.groundSpeedNorth) + fabsf(varioStatus.groundSpeedEast)) / 100.0f;

    measRowT.insert(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,0) =
            (FastMath::fastATan2(varioStatus.groundSpeedEast,varioStatus.groundSpeedNorth + temp1) - calculatedValue) / temp1;
    measRowT.insert(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,0) =
            (FastMath::fastATan2(varioStatus.groundSpeedEast + temp1,varioStatus.groundSpeedNorth) - calculatedValue) / temp1;

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = calculatedValue;
        measRowTTst1 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredCourseOverGround = " <<  measuredCourseOverGround
    		<< ", calculated Course = " << calculatedValue << ", variance = " << courseOverGroundVariance);

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
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
    FloatType calculatedValue;
    Eigen::SparseMatrix<FloatType> measRowT(GliderVarioStatus::STATUS_NUM_ROWS,1);
    FloatType temp1, temp2, temp3;
    FloatType groundSpeedNSquare = varioStatus.groundSpeedNorth * varioStatus.groundSpeedNorth;
    FloatType groundSpeedESquare = varioStatus.groundSpeedEast  * varioStatus.groundSpeedEast;

    measRowT.reserve(GliderVarioStatus::STATUS_NUM_ROWS);

    // calculate and fill in local variables here.
    measuredSpeedOverGround *= NM_TO_M / 3600.0f;
    measurementVector.gpsSpeed = measuredSpeedOverGround;
    calculatedValue = sqrtf(groundSpeedNSquare + groundSpeedESquare);

    // approximate the derivates
    // use the same increment for both directions to avoid numerical resolution problems
    temp3 = calculatedValue / 100.0f;

    temp1 = varioStatus.groundSpeedNorth + temp3;
    temp2 = sqrtf(temp1 * temp1 + groundSpeedESquare);
    measRowT.insert(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,0) = (temp2-calculatedValue) / temp3;

    temp1 = varioStatus.groundSpeedEast + temp3;
    temp2 = sqrtf(groundSpeedNSquare + temp1 * temp1);
    measRowT.insert(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,0) = (temp2-calculatedValue) / temp3;


    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = calculatedValue;
        measRowTTst1 = measRowT;
        }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredSpeedOverGround = " <<  measuredSpeedOverGround
    		<< ", calculated speed = " << calculatedValue << ", variance = " << speedOverGroundVariance);

    calcSingleMeasureUpdate (
            measuredSpeedOverGround,
            calculatedValue,
            speedOverGroundVariance,
            measRowT,
            varioStatus
    );
}


void
GliderVarioMeasurementUpdater::accelUpd (
        FloatType measuredAccelX,
        FloatType accelXVariance,
        FloatType measuredAccelY,
        FloatType accelYVariance,
        FloatType measuredAccelZ,
        FloatType accelZVariance,
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
    FloatType turnRateRad = varioStatus.yawRateZ * FastMath::degToRad;
    Eigen::SparseMatrix<FloatType> measRowT(GliderVarioStatus::STATUS_NUM_ROWS,3);
    RotationMatrix rotMat, rotMatIncX, rotMatIncY;
    Vector3DType modelAccelVector;
    Vector3DType calcAccelVector;
    Vector3DType calcAccelVectorIncX;
    Vector3DType calcAccelVectorIncY;
    Vector3DType measuredAccelVector {measuredAccelX,measuredAccelY,measuredAccelZ};

    measRowT.reserve(GliderVarioStatus::STATUS_NUM_ROWS*3);

    // calculate and fill in local variables here.
    measurementVector.accelX = measuredAccelX;
    measurementVector.accelY = measuredAccelY;
    measurementVector.accelZ = measuredAccelZ;

    rotMat.setYaw(0.0f); // Remember, model coordinate system X axis points to heading direction?
    rotMat.setPitch(varioStatus.pitchAngle);
    rotMat.setRoll(varioStatus.rollAngle);
    rotMatIncX.setYaw(0.0f);
    rotMatIncX.setPitch(varioStatus.pitchAngle);
    rotMatIncX.setRoll(varioStatus.rollAngle + 1.0f);
    rotMatIncY.setYaw(0.0f);
    rotMatIncY.setPitch(varioStatus.pitchAngle + 1.0f);
    rotMatIncY.setRoll(varioStatus.rollAngle);

    modelAccelVector(0) = varioStatus.accelHeading;
    modelAccelVector(1) = turnRateRad * varioStatus.trueAirSpeed ; // + varioStatus.accelCross; Ignore cross acceleration
    modelAccelVector(2) = varioStatus.accelVertical - varioStatus.gravity;

    RotationMatrix3DType &rotMatGloToPlane = rotMat.getMatrixGloToPlane();
    rotMat.calcWorldVectorToPlaneVector(modelAccelVector,calcAccelVector);
    rotMatIncX.calcWorldVectorToPlaneVector(modelAccelVector,calcAccelVectorIncX);
    rotMatIncY.calcWorldVectorToPlaneVector(modelAccelVector,calcAccelVectorIncY);

    FloatType TasTimesDegToRad = varioStatus.trueAirSpeed * FastMath::degToRad;

    // The linear factors
    measRowT.insert(GliderVarioStatus::STATUS_IND_ACC_HEADING,0) = rotMatGloToPlane(0,0);
    // measRowT.insert(GliderVarioStatus::STATUS_IND_ACC_CROSS,0) = rotMatGloToPlane(0,1);
    measRowT.insert(GliderVarioStatus::STATUS_IND_TAS,0) = rotMatGloToPlane(0,1) * turnRateRad;
    measRowT.insert(GliderVarioStatus::STATUS_IND_ROTATION_Z,0) = rotMatGloToPlane(0,1) * TasTimesDegToRad;
    measRowT.insert(GliderVarioStatus::STATUS_IND_ACC_VERTICAL,0) = rotMatGloToPlane(0,2);
    measRowT.insert(GliderVarioStatus::STATUS_IND_GRAVITY,0) = -(rotMatGloToPlane(0,2));

    // Now the non-linear factors by approximation (the attitude angles)
    measRowT.insert(GliderVarioStatus::STATUS_IND_ROLL,0) = calcAccelVectorIncX(0) - calcAccelVector(0);
    measRowT.insert(GliderVarioStatus::STATUS_IND_PITCH,0) = calcAccelVectorIncY(0) - calcAccelVector(0);

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = calcAccelVector(0);
        measRowTTst1 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredAccelX = " <<  measuredAccelX
    		<< ", calcAccel = " << calcAccelVector(0) << ", variance = " << accelXVariance);

    // The linear factors
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ACC_HEADING,1) = rotMatGloToPlane(1,0);
    // measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ACC_CROSS,1) = rotMatGloToPlane(1,1);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_TAS,1) = rotMatGloToPlane(1,1) * turnRateRad;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_Z,1) = rotMatGloToPlane(1,1) * TasTimesDegToRad;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ACC_VERTICAL,1) = rotMatGloToPlane(1,2);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_GRAVITY,1) = -(rotMatGloToPlane(1,2));

    // Now the non-linear factors by approximation (the attitude angles)
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,1) = calcAccelVectorIncX(1) - calcAccelVector(1);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_PITCH,1) = calcAccelVectorIncY(1) - calcAccelVector(1);

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst2 = calcAccelVector(1);
        measRowTTst2 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredAccelY = " <<  measuredAccelY
    		<< ", calcAccel = " << calcAccelVector(1) << ", variance = " << accelYVariance);

    // The linear factors
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ACC_HEADING,2) = rotMatGloToPlane(2,0);
    // measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ACC_CROSS,2) = rotMatGloToPlane(2,1);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_TAS,2) = rotMatGloToPlane(2,1) * turnRateRad;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_Z,2) = rotMatGloToPlane(2,1) * TasTimesDegToRad;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ACC_VERTICAL,2) = rotMatGloToPlane(2,2);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_GRAVITY,2) = -(rotMatGloToPlane(2,2));

    // Now the non-linear factors by approximation (the attitude angles)
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,2) = calcAccelVectorIncX(2) - calcAccelVector(2);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_PITCH,2) = calcAccelVectorIncY(2) - calcAccelVector(2);

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst3 = calcAccelVector(2);
        measRowTTst3 = measRowT;
    }

#if defined HAVE_LOG4CXX_H
    for (decltype(measRowT.rows()) i = 0; i < measRowT.rows() ; ++i){
    	if (measRowT.coeff(i,0)!= 0.0f ||measRowT.coeff(i,1)!= 0.0f ||measRowT.coeff(i,2)!= 0.0f) {
        	LOG4CXX_TRACE(logger, "measRowT["<< GliderVarioStatus::StatusComponentIndex(i) <<"] = {"
        			<< measRowT.coeff(i,0)<<",\t"<< measRowT.coeff(i,1)<<",\t"<< measRowT.coeff(i,2)<<'}'
					);
    	}
    }

    {
		GliderVarioStatus::StatusCoVarianceType & covPMatrix = varioStatus.getErrorCovariance_P();
		for (decltype(covPMatrix.rows()) i = 0; i < covPMatrix.rows() ; ++i){
			if (
					covPMatrix.coeff(i,GliderVarioStatus::STATUS_IND_ACC_HEADING) != 0.0f ||
					covPMatrix.coeff(i,GliderVarioStatus::STATUS_IND_ACC_CROSS) != 0.0f ||
					covPMatrix.coeff(i,GliderVarioStatus::STATUS_IND_ACC_VERTICAL) != 0.0f
					) {
				LOG4CXX_TRACE(logger, "covPMatrix["<<GliderVarioStatus::StatusComponentIndex(i)<<",{Head/Cross/Vert}] = "
						<< covPMatrix.coeff(i,GliderVarioStatus::STATUS_IND_ACC_HEADING) << '\t'
						<< covPMatrix.coeff(i,GliderVarioStatus::STATUS_IND_ACC_CROSS) << '\t'
						<< covPMatrix.coeff(i,GliderVarioStatus::STATUS_IND_ACC_VERTICAL)
						);
			}
		}
    }
#endif

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredAccelZ = " <<  measuredAccelZ
    		<< ", calcAccel = " << calcAccelVector(2) << ", variance = " << accelZVariance);

    LOG4CXX_TRACE(logger,__FUNCTION__ << ": ErrorCovariance_P before Accel update = " <<
        		printCovMatrix(varioStatus.getErrorCovariance_P()));

    RotationMatrix3DType measureVariance;

    measureVariance.setZero();
    measureVariance(0,0) = accelXVariance;
    measureVariance(1,1) = accelYVariance;
    measureVariance(2,2) = accelZVariance;

    calc3DMeasureUpdate (
            measuredAccelVector,
            calcAccelVector,
            measureVariance,
            measRowT,
            varioStatus
    );

    LOG4CXX_TRACE(logger,__FUNCTION__ << ": ErrorCovariance_P after Accel update = " <<
    		printCovMatrix(varioStatus.getErrorCovariance_P()));

}


void
GliderVarioMeasurementUpdater::gyroUpd (
        FloatType measuredRollRateX,
        FloatType rollRateXVariance,
        FloatType measuredPitchRateY,
        FloatType pitchRateYVariance,
        FloatType measuredYawRateZ,
        FloatType yawRateZVariance,
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
    Eigen::SparseMatrix<FloatType> measRowT(GliderVarioStatus::STATUS_NUM_ROWS,3);
    RotationMatrix rotMat;
    Vector3DType modelRotVector;
    Vector3DType calcRotVector;
    Vector3DType measuredRotVector {measuredRollRateX,measuredPitchRateY,measuredYawRateZ};

    measRowT.reserve(GliderVarioStatus::STATUS_NUM_ROWS*3);

    // calculate and fill in local variables here.
    measurementVector.gyroRateX = measuredRollRateX;
    measurementVector.gyroRateY = measuredPitchRateY;
    measurementVector.gyroRateZ = measuredYawRateZ;

    rotMat.setYaw(0.0f); // Remember, model coordinate system X axis points to heading direction.
    rotMat.setPitch(varioStatus.pitchAngle);
    rotMat.setRoll(varioStatus.rollAngle);

    modelRotVector(0) = varioStatus.rollRateX;
    modelRotVector(1) = varioStatus.pitchRateY;
    modelRotVector(2) = varioStatus.yawRateZ;

    RotationMatrix3DType const &rotMatGloToPlane = rotMat.getMatrixGloToPlane();
    rotMat.calcWorldVectorToPlaneVector(modelRotVector,calcRotVector);

    calcRotVector(0) += varioStatus.gyroBiasX;
    calcRotVector(1) += varioStatus.gyroBiasY;
    calcRotVector(2) += varioStatus.gyroBiasZ;

    // Now the update for the X-Axis measurement

    // The linear factors
    measRowT.insert(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,0)    = 1.0f;
    measRowT.insert(GliderVarioStatus::STATUS_IND_ROTATION_X,0)     = rotMatGloToPlane(0,0);
    measRowT.insert(GliderVarioStatus::STATUS_IND_ROTATION_Y,0)     = rotMatGloToPlane(0,1);
    measRowT.insert(GliderVarioStatus::STATUS_IND_ROTATION_Z,0) = rotMatGloToPlane(0,2);

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = calcRotVector(0);
        measRowTTst1 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredRollRateX = " <<  measuredRollRateX
    		<< ", calcRotationX = " << calcRotVector(0) << ", variance = " << rollRateXVariance);

    // Now the update for the Y-Axis measurement

    // The linear factors
    measRowT.insert(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,1)    = 1.0f;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_X,1)     = rotMatGloToPlane(1,0);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_Y,1)     = rotMatGloToPlane(1,1);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_Z,1) = rotMatGloToPlane(1,2);

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst2 = calcRotVector(1);
        measRowTTst2 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredPitchRateY = " <<  measuredPitchRateY
    		<< ", calcRotationY = " << calcRotVector(1) << ", variance = " << pitchRateYVariance);

    // Now the update for the Z-Axis measurement

    // The linear factors
    measRowT.insert(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,2)    = 1.0f;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_X,2)     = rotMatGloToPlane(2,0);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_Y,2)     = rotMatGloToPlane(2,1);
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_Z,2) = rotMatGloToPlane(2,2);

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst3 = calcRotVector(2);
        measRowTTst3 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredYawRateZ = " <<  measuredYawRateZ
    		<< ", calcRotationZ = " << calcRotVector(2) << ", variance = " << yawRateZVariance);

    RotationMatrix3DType measureVariance;
    measureVariance.setZero();
    measureVariance(0,0) = rollRateXVariance;
    measureVariance(1,1) = pitchRateYVariance;
    measureVariance(2,2) = yawRateZVariance;

    calc3DMeasureUpdate (
            measuredRotVector,
            calcRotVector,
			measureVariance,
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
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
    Eigen::SparseMatrix<FloatType> measRowT(GliderVarioStatus::STATUS_NUM_ROWS,3);

    measRowT.reserve(GliderVarioStatus::STATUS_NUM_ROWS);

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
    // The same rotation matrix with increments to declination and inclination to assess the Jacobian
    RotationMatrix magRotMatrixIncDeclination(varioStatus.magneticDeclination + 1.0f,varioStatus.magneticInclination       ,0.0f);
    RotationMatrix magRotMatrixIncInclination(varioStatus.magneticDeclination       ,varioStatus.magneticInclination + 1.0f,0.0f);

    // Rotation matrix of the plane attitude to project the magnetic vector to the plane measurements
    RotationMatrix attitudeRotMatrix         (varioStatus.heading       ,varioStatus.pitchAngle       ,varioStatus.rollAngle);
    // Increment plane attitude rotation matrixes to assess the Jacobians for the attitude angles
    RotationMatrix attitudeRotMatrixIncYaw   (varioStatus.heading + 1.0f,varioStatus.pitchAngle       ,varioStatus.rollAngle);
    RotationMatrix attitudeRotMatrixIncPitch (varioStatus.heading       ,varioStatus.pitchAngle + 1.0f,varioStatus.rollAngle);
    RotationMatrix attitudeRotMatrixIncRoll  (varioStatus.heading       ,varioStatus.pitchAngle       ,varioStatus.rollAngle + 1.0f);

    // The resulting rotation matrix from unit vector to the magnetic vector as seen in the plane
    RotationMatrix3DType compassMatrix = attitudeRotMatrix.getMatrixGloToPlane() * magRotMatrix.getMatrixPlaneToGlo();
    // The resulting rotation matrixes with the 5 increments
    RotationMatrix3DType compassMatrixIncDeclination = attitudeRotMatrix.getMatrixGloToPlane() * magRotMatrixIncDeclination.getMatrixPlaneToGlo() ;
    RotationMatrix3DType compassMatrixIncInclination = attitudeRotMatrix.getMatrixGloToPlane() * magRotMatrixIncInclination.getMatrixPlaneToGlo();
    RotationMatrix3DType compassMatrixIncYaw  = attitudeRotMatrixIncYaw.getMatrixGloToPlane() * magRotMatrix.getMatrixPlaneToGlo();
    RotationMatrix3DType compassMatrixIncPitch = attitudeRotMatrixIncPitch.getMatrixGloToPlane() * magRotMatrix.getMatrixPlaneToGlo();
    RotationMatrix3DType compassMatrixIncRoll  = attitudeRotMatrixIncRoll.getMatrixGloToPlane() * magRotMatrix.getMatrixPlaneToGlo();

    Vector3DType magVecLength (compensatedMagVectorLength,0.0f,0.0f);

    // The calculated vector of magnetic measurements.
    // This is the vector of compensated magnetic flows as calculated from the current attitude, inclination, and declination
    Vector3DType compassVector               = compassMatrix               * magVecLength;

    // Variations of the vector with increments of the 5 participating factors.
    Vector3DType compassVectorIncDeclination = compassMatrixIncDeclination * magVecLength;
    Vector3DType compassVectorIncInclination = compassMatrixIncInclination * magVecLength;
    Vector3DType compassVectorIncYaw         = compassMatrixIncYaw         * magVecLength;
    Vector3DType compassVectorIncPitch       = compassMatrixIncPitch       * magVecLength;
    Vector3DType compassVectorIncRoll        = compassMatrixIncRoll        * magVecLength;

    measurementVector.magX = measuredMagFlowX;
    measurementVector.magY = measuredMagFlowY;
    measurementVector.magZ = measuredMagFlowZ;

    auto tempX = compassVector(0);
    compassVector(0) = tempX + varioStatus.compassDeviationX;

    auto tempY = compassVector(1);
    compassVector(1) = tempY + varioStatus.compassDeviationY;

    auto tempZ = compassVector(2);
    compassVector(2) = tempZ + varioStatus.compassDeviationZ;


    // calculate and fill in local variables here.
    measRowT.insert(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X,0)  =  1.0f;
    measRowT.insert(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION,0) = compassVectorIncDeclination(0) - tempX;
    measRowT.insert(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION,0) = compassVectorIncInclination(0) - tempX;
    measRowT.insert(GliderVarioStatus::STATUS_IND_HEADING,0)              = compassVectorIncYaw(0)         - tempX;
    measRowT.insert(GliderVarioStatus::STATUS_IND_PITCH,0)                = compassVectorIncPitch(0)       - tempX;
    measRowT.insert(GliderVarioStatus::STATUS_IND_ROLL,0)                 = compassVectorIncRoll(0)        - tempX;

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = compassVector(0);
        measRowTTst1 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredMagFlowX = " <<  measuredMagFlowX
    		<< ", calculatedValueX = " << compassVector(0) << ", variance = " << magFlowXVariance);

    // // measRowT.setZero(); Just reset the previous deviation factor. All others are re-used

    // calculate and fill in local variables here.
    measRowT.insert  (GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y,1)  =  1.0f;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION,1) = compassVectorIncDeclination(1) - tempY;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION,1) = compassVectorIncInclination(1) - tempY;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_HEADING,1)              = compassVectorIncYaw(1)         - tempY;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_PITCH,1)                = compassVectorIncPitch(1)       - tempY;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,1)                 = compassVectorIncRoll(1)        - tempY;

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst2 = compassVector(1);
        measRowTTst2 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredMagFlowY = " <<  measuredMagFlowY
    		<< ", calculatedValueY = " << compassVector(1) << ", variance = " << magFlowYVariance);

    // calculate and fill in local variables here.
    measRowT.insert  (GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z,2)  =  1.0f;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION,2) = compassVectorIncDeclination(2) - tempZ;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION,2) = compassVectorIncInclination(2) - tempZ;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_HEADING,2)              = compassVectorIncYaw(2)         - tempZ;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_PITCH,2)                = compassVectorIncPitch(2)       - tempZ;
    measRowT.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,2)                 = compassVectorIncRoll(2)        - tempZ;

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst3 = compassVector(2);
        measRowTTst3 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredMagFlowZ = " <<  measuredMagFlowZ
    		<< ", calculatedValueZ = " << compassVector(2) << ", variance = " << magFlowZVariance);

    Vector3DType measuredVector {measuredMagFlowX,measuredMagFlowY,measuredMagFlowZ};
    RotationMatrix3DType varianceMatrix;
    varianceMatrix.setZero();
    varianceMatrix(0,0) = magFlowXVariance;
    varianceMatrix(1,1) = magFlowYVariance;
    varianceMatrix(2,2) = magFlowZVariance;

    calc3DMeasureUpdate (
            measuredVector,
			compassVector,
			varianceMatrix,
            measRowT,
            varioStatus
    );
}


void
GliderVarioMeasurementUpdater::staticPressureUpd (
        FloatType measuredStaticPressure,
        FloatType measuredTemperature,
        FloatType staticPressureVariance,
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
    Eigen::SparseMatrix<FloatType> measRowT1(GliderVarioStatus::STATUS_NUM_ROWS,1);
    Eigen::SparseMatrix<FloatType> measRowT2(GliderVarioStatus::STATUS_NUM_ROWS,1);

    FloatType pFactor;
    FloatType p;
    FloatType p1;

    // calculate and fill in local variables here.
    measurementVector.staticPressure = measuredStaticPressure;

    // This is used to calculate the pressure and at the same time the derivate for Qff.
    pFactor = calcBarometricFactor(varioStatus.altMSL,measuredTemperature);
    // The pressure at the height in the dry indifferent boundary layer.
    p = varioStatus.qff * pFactor;
    // The pressure 10m above to assess the derivate for altitude deviations
    p1 = varioStatus.qff * calcBarometricFactor(varioStatus.altMSL + 10.0f,measuredTemperature);

    measRowT1.insert(GliderVarioStatus::STATUS_IND_ALT_MSL,0) = (p1 - p) / 10.0f;

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = p;
        measRowTTst1 = measRowT1;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measured static pressure = " <<  measuredStaticPressure
    		<< ", measured temperature = " <<  (measuredTemperature)
    		<< ", calculated pressure = " << p << ", variance = " << staticPressureVariance);


#if defined HAVE_LOG4CXX_H
    if (logger->isTraceEnabled()) {
    	std::ostringstream str;
    	GliderVarioStatus::StatusCoVarianceType &errCov = varioStatus.getErrorCovariance_P();
   		for (GliderVarioStatus::StatusCoVarianceType::InnerIterator it(errCov,GliderVarioStatus::STATUS_IND_QFF);it;++it) {
			str << "\n	errCov[" << GliderVarioStatus::StatusComponentIndex(it.row())
					<< ',' << GliderVarioStatus::StatusComponentIndex(it.col())
					<< "= " << it.value();
   		}
    	LOG4CXX_TRACE(logger,"  Error Covariance for QFF before update:" << str.str());

    	str = std::ostringstream();
		for (GliderVarioStatus::StatusCoVarianceType::InnerIterator it(errCov,GliderVarioStatus::STATUS_IND_LAST_PRESSURE);it;++it) {
			str << "\n	errCov[" << GliderVarioStatus::StatusComponentIndex(it.row())
				<< ',' << GliderVarioStatus::StatusComponentIndex(it.col())
				<< "= " << it.value();
		}
    	LOG4CXX_TRACE(logger,"  Error Covariance for LAST_PRESSURE before update:" << str.str());

    	str = std::ostringstream();
		for (GliderVarioStatus::StatusCoVarianceType::InnerIterator it(errCov,GliderVarioStatus::STATUS_IND_ALT_MSL);it;++it) {
			str << "\n	errCov[" << GliderVarioStatus::StatusComponentIndex(it.row())
				<< ',' << GliderVarioStatus::StatusComponentIndex(it.col())
				<< "= " << it.value();
		}
    	LOG4CXX_TRACE(logger,"  Error Covariance for ALT_MSL: before update" << str.str());
    }
    LOG4CXX_TRACE(logger,__FUNCTION__ << ": ErrorCovariance_P before static pressure update = " <<
    		printCovMatrix(varioStatus.getErrorCovariance_P()));
#endif

    LOG4CXX_DEBUG(logger,"		Altitude derivate = " << measRowT1.coeff(GliderVarioStatus::STATUS_IND_ALT_MSL,0));

    calcSingleMeasureUpdate (
            measuredStaticPressure,
            p,
            staticPressureVariance,
            measRowT1,
            varioStatus
    );

    LOG4CXX_TRACE(logger,__FUNCTION__ << ": ErrorCovariance_P after static pressure update = " <<
    		printCovMatrix(varioStatus.getErrorCovariance_P()));

    measRowT2.insert(GliderVarioStatus::STATUS_IND_QFF,0) = pFactor;
    LOG4CXX_DEBUG(logger,"		QFF derivate = " << pFactor );

    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst2 = p;
        measRowTTst2 = measRowT2;
    }

    calcSingleMeasureUpdate (
            measuredStaticPressure,
            p,
            staticPressureVariance,
            measRowT2,
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
        GliderVarioMeasurementVector &measurementVector,
        GliderVarioStatus &varioStatus
) {
    static FloatType constexpr RspecTimes2     = Rspec * 2.0f;     // Specific R for dry air

    // This term is used repeatedly
    // Note, pressure must be in Pascal
    FloatType pressRspecTemp = varioStatus.lastPressure * (100.0f / RspecTimes2) / (measuredTemperature + CtoK);
    FloatType tmp2;
    FloatType dynPressure;
    Eigen::SparseMatrix<FloatType> measRowT(GliderVarioStatus::STATUS_NUM_ROWS,1);

    if (varioStatus.pitchAngle > 60.0f || varioStatus.pitchAngle < -60.0f) {
    	// If the pitch angle becomes too large the vertical component of TAS becomes just too much
    	// for a reliable calculation.
    	return;
    }

    // Convert the measured pressure to Pa
    measuredDynamicPressure *= 100.0f;
    dynamicPressureVariance *= 10000.0f;

    measRowT.reserve(GliderVarioStatus::STATUS_NUM_ROWS);

    // calculate and fill in local variables here.
    // Develop the dynamic pressure gradually to get the derivates of the variables most efficiently
    // dyn pressure = 0.5 * density * speed * speed
    // dyn pressure = 0.5 * (pressure / Rspec /temp) * speed * speed

    // // At negative speeds force the calculated pressure to go negative. Otherwise an increase in pressure will
    // // result in a further *decrease* of the calculated speed. Therefore use the absolute value of speed once.
    // // The results are horribly inaccurate at higher speeds, but this will occur only close to 0.
    // // In actual flight this is a non-issue because the speed is always positive except for some hard-core
    // // aerobatics, but that is way beyond the design envelope of this instrument.
    // dynPressure = pressRspecTemp * fabs(varioStatus.trueAirSpeed) * varioStatus.trueAirSpeed;
    auto cosPitch = FastMath::fastCos(varioStatus.pitchAngle);
    auto trueAirSpeed = varioStatus.trueAirSpeed;
    // Assume I fly also vertical with a speed according to horizontal TAS, and my pitch angle.
    auto totalSpeed = trueAirSpeed / cosPitch;
    dynPressure = pressRspecTemp * fabs(totalSpeed) * totalSpeed;

	tmp2 = totalSpeed + 1.0f;
    tmp2 = pressRspecTemp * fabs(tmp2) * tmp2;
    tmp2 = (tmp2 - dynPressure);
    // At negative speeds force the derivate to negative. Otherwise an increase in pressure will
    // result in a further *decrease* of the calculated speed. Therefore use the sign of the value.
    measRowT.insert(GliderVarioStatus::STATUS_IND_TAS,0) = tmp2 * cosPitch;
    measRowT.insert(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,0) = tmp2 * FastMath::fastSin(varioStatus.pitchAngle);


    if (unitTestMode) {
        // Save internal statuses for unit tests
        calculatedValueTst1 = dynPressure;
        measRowTTst1 = measRowT;
    }

    LOG4CXX_DEBUG(logger,__FUNCTION__ << ": measuredDynamicPressure = " <<  measuredDynamicPressure
    		<< "Pa, calculated dynPressure = " << dynPressure << "Pa, variance = " << dynamicPressureVariance
			<< ": derivative X = " << measRowT.coeff(GliderVarioStatus::STATUS_IND_TAS,0)
			<< ", Z = " << measRowT.coeff(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,0));

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
        Eigen::SparseMatrix<FloatType> const &measRowT,
        GliderVarioStatus &varioStatus
) {
    GliderVarioStatus::StatusCoVarianceType &coVariance_P = varioStatus.getErrorCovariance_P();
    GliderVarioStatus::StatusVectorType &statusVector_x = varioStatus.getStatusVector_x();

    Eigen::SparseMatrix <FloatType> kalmanGain_K(GliderVarioStatus::STATUS_NUM_ROWS,1);
    FloatType denominator;
    Eigen::SparseMatrix<FloatType> denominatorMatrix;

    kalmanGain_K.reserve(GliderVarioStatus::STATUS_NUM_ROWS);
    denominatorMatrix.reserve(4);

    // Intermediate because a term is used twice
    Eigen::SparseMatrix <FloatType> hTimesP(1,GliderVarioStatus::STATUS_NUM_ROWS);
    hTimesP.reserve(GliderVarioStatus::STATUS_NUM_ROWS);

    FloatType valueDiff = measuredValue - calculatedValue;

    hTimesP = measRowT.transpose() * coVariance_P;
    denominatorMatrix = hTimesP * measRowT;

    denominator = denominatorMatrix.coeff(0,0) + measurementVariance_R;

    kalmanGain_K = coVariance_P * measRowT;
    kalmanGain_K /= denominator;

    LOG4CXX_DEBUG(logger ,"calcSingleMeasureUpdate: valueDiff = " << valueDiff
    		<< ", denominator = " << denominator);
#if HAVE_LOG4CXX_H
    if (logger->isDebugEnabled()) {
    	for (Eigen::SparseMatrix <FloatType>::InnerIterator it(kalmanGain_K,0); it; ++it) {
    		LOG4CXX_DEBUG(logger ,"    kalmanGain_K[" << GliderVarioStatus::StatusComponentIndex(it.row()) << "] = " << it.value());
    	}
    }
#endif // HAVE_LOG4CXX_H

    // substitute direct assignment by iterating over the sparse kalman gain vector, and perform the correct element wise.
    // Eigen does not take mixing dense and sparse matrixes lightly.
    GliderVarioStatus::StatusComponentIndex index;
    FloatType kalmanGain;
    FloatType val;
    for (Eigen::SparseMatrix<FloatType>::InnerIterator iter(kalmanGain_K,0); iter ; ++iter){
        index = GliderVarioStatus::StatusComponentIndex(iter.row());
        kalmanGain = iter.value();
        kalmanGain *= valueDiff;
        val = statusVector_x(index);
        statusVector_x(index) = val + kalmanGain;
        LOG4CXX_DEBUG(logger ,"Update " << index
        		<< ": New value = " << statusVector_x(index)
				<< ", Correction value = " << kalmanGain
				<< ", variance before = " << coVariance_P.coeff(index,index));
    }

    coVariance_P -=  (kalmanGain_K * hTimesP);

#if HAVE_LOG4CXX_H
    for (Eigen::SparseMatrix<FloatType>::InnerIterator iter(kalmanGain_K,0); iter ; ++iter){
        index = GliderVarioStatus::StatusComponentIndex(iter.row());
        LOG4CXX_DEBUG(logger ,"Update " << index
				<< ", variance after = " << coVariance_P.coeff(index,index));
    }
#endif
}

void GliderVarioMeasurementUpdater::calc2DMeasureUpdate (
        Vector2DType const &measuredValue,
		Vector2DType const &calculatedValue,
		Matrix2DType const &measurementVariance_R,
        Eigen::SparseMatrix<FloatType> const &measRowT,
        GliderVarioStatus &varioStatus
) {
    GliderVarioStatus::StatusCoVarianceType &coVariance_P = varioStatus.getErrorCovariance_P();
    GliderVarioStatus::StatusVectorType &statusVector_x = varioStatus.getStatusVector_x();

    Eigen::SparseMatrix <FloatType> kalmanGain_K(GliderVarioStatus::STATUS_NUM_ROWS,2);
    Eigen::SparseMatrix <FloatType> denominator(2,2);
    Eigen::SparseMatrix <FloatType> denominatorInv(2,2);

    kalmanGain_K.reserve(GliderVarioStatus::STATUS_NUM_ROWS * 2);
    denominator.reserve(4);
    denominatorInv.reserve(4);

    // Intermediate because a term is used twice
    Eigen::SparseMatrix <FloatType> hTimesP(2,GliderVarioStatus::STATUS_NUM_ROWS);
    hTimesP.reserve(GliderVarioStatus::STATUS_NUM_ROWS * 2);

    Vector2DType valueDiff = measuredValue - calculatedValue;

    hTimesP = measRowT.transpose() * coVariance_P;
    denominator = hTimesP * measRowT + measurementVariance_R;

    calcInverse2D (denominatorInv, denominator);

    kalmanGain_K = coVariance_P * measRowT * denominatorInv;

    LOG4CXX_DEBUG(logger ,"calc2DMeasureUpdate: valueDiff = \n" << valueDiff
    		<< "\n, denominator = " << printSparseMatrixSimple(denominator)
    		<< ", denominatorInv = " << printSparseMatrixSimple(denominatorInv));
#if HAVE_LOG4CXX_H

    for (int i = 0; i<2; ++i) {
		for (Eigen::SparseMatrix <FloatType>::InnerIterator it(kalmanGain_K,i); it; ++it) {
			LOG4CXX_DEBUG(logger ,"    kalmanGain_K[" << GliderVarioStatus::StatusComponentIndex(it.row()) << " , " << i <<"] = " << it.value());
		}
    }

#endif // HAVE_LOG4CXX_H

    // substitute direct assignment by iterating over the sparse kalman gain vector, and perform the correct element wise.
    // Eigen does not take mixing dense and sparse matrixes lightly.
    GliderVarioStatus::StatusComponentIndex index;
    FloatType kalmanGain;
    FloatType val;

    for (int i = 0; i<2; ++i) {
        for (Eigen::SparseMatrix<FloatType>::InnerIterator iter(kalmanGain_K,i); iter ; ++iter){
            index = GliderVarioStatus::StatusComponentIndex(iter.row());
            kalmanGain = iter.value();
            kalmanGain *= valueDiff(i);
            val = statusVector_x(index);
            statusVector_x(index) = val + kalmanGain;
            LOG4CXX_DEBUG(logger ,"Update " << index << ":" << i
            		<< ": New value = " << statusVector_x(index)
					<< ", Correction value = " << kalmanGain
					<< ", Variance = " << coVariance_P.coeff(index,index));
        }
    }

    // Put the co-variance calculation here. Then the results are available for debug prints below.
    coVariance_P -=  (kalmanGain_K * hTimesP);

#if HAVE_LOG4CXX_H
    for (int i = 0; i<2; ++i) {
		for (Eigen::SparseMatrix<FloatType>::InnerIterator iter(kalmanGain_K,i); iter ; ++iter){
			index = GliderVarioStatus::StatusComponentIndex(iter.row());
			LOG4CXX_DEBUG(logger ,"Update " << index << ":" << i
					<< ", variance after = " << coVariance_P.coeff(index,index));
		}
    }
#endif

}

void GliderVarioMeasurementUpdater::calc3DMeasureUpdate (
        Vector3DType const &measuredValue,
		Vector3DType const &calculatedValue,
		RotationMatrix3DType const &measurementVariance_R,
        Eigen::SparseMatrix<FloatType> const &measRowT,
        GliderVarioStatus &varioStatus
) {
    GliderVarioStatus::StatusCoVarianceType &coVariance_P = varioStatus.getErrorCovariance_P();
    GliderVarioStatus::StatusVectorType &statusVector_x = varioStatus.getStatusVector_x();

    Eigen::SparseMatrix <FloatType> kalmanGain_K(GliderVarioStatus::STATUS_NUM_ROWS,3);
    Eigen::SparseMatrix <double> denominator(3,3);
    Eigen::SparseMatrix <double> denominatorInvers(3,3);

    kalmanGain_K.reserve(GliderVarioStatus::STATUS_NUM_ROWS * 3);
    denominator.reserve(9);
    denominatorInvers.reserve(9);

    // Intermediate because a term is used twice
    Eigen::SparseMatrix <FloatType> hTimesP(3,GliderVarioStatus::STATUS_NUM_ROWS);
    hTimesP.reserve(GliderVarioStatus::STATUS_NUM_ROWS * 3);

    Vector3DType valueDiff = measuredValue - calculatedValue;

    hTimesP = measRowT.transpose() * coVariance_P;
    denominator = (hTimesP * measRowT + measurementVariance_R).cast <double>();

    calcInverse3D (denominatorInvers, denominator);

    kalmanGain_K = ((coVariance_P * measRowT).cast<double>() * denominatorInvers).cast<FloatType>();

    LOG4CXX_DEBUG(logger ,"calc3DMeasureUpdate: valueDiff = \n" << valueDiff
    		<< ", denominator = " << printSparseMatrixSimple(denominator)
			<< ", denominatorInvers = " << printSparseMatrixSimple(denominatorInvers));
#if HAVE_LOG4CXX_H

    if (logger->isDebugEnabled()) {
        for (int i = 0; i<3; ++i) {
			for (Eigen::SparseMatrix <FloatType>::InnerIterator it(kalmanGain_K,i); it; ++it) {
				LOG4CXX_DEBUG(logger ,"    kalmanGain_K[" << GliderVarioStatus::StatusComponentIndex(it.row()) << " , " << i <<"] = " << it.value());
			}
        }
    }

#endif // HAVE_LOG4CXX_H

    // substitute direct assignment by iterating over the sparse kalman gain vector, and perform the correct element wise.
    // Eigen does not take mixing dense and sparse matrixes lightly.
    GliderVarioStatus::StatusComponentIndex index;
    FloatType kalmanGain;
    FloatType val;

    for (int i = 0; i<3; ++i) {
        for (Eigen::SparseMatrix<FloatType>::InnerIterator iter(kalmanGain_K,i); iter ; ++iter){
            index = GliderVarioStatus::StatusComponentIndex(iter.row());
            kalmanGain = iter.value();
            kalmanGain *= valueDiff(i);
            val = statusVector_x(index);
            statusVector_x(index) = val + kalmanGain;
            LOG4CXX_DEBUG(logger ,"Update " << index << ":" << i
            		<< ": New value = " << statusVector_x(index)
					<< ", Correction value = " << kalmanGain
					<< ", Variance = " << coVariance_P.coeff(index,index));
        }
    }

    coVariance_P -= kalmanGain_K * hTimesP;

#if HAVE_LOG4CXX_H
	for (int i = 0; i<3; ++i) {
		for (Eigen::SparseMatrix<FloatType>::InnerIterator iter(kalmanGain_K,i); iter ; ++iter){
			index = GliderVarioStatus::StatusComponentIndex(iter.row());
			LOG4CXX_DEBUG(logger ,"Update " << index << ":" << i
					<< ", variance after = " << coVariance_P.coeff(index,index));
		}
	}
#endif


}

void GliderVarioMeasurementUpdater::setUnitTestMode(bool unitTestMode ) {
    GliderVarioMeasurementUpdater::unitTestMode = unitTestMode;
}

bool GliderVarioMeasurementUpdater::getUnitTestMode() {
    return GliderVarioMeasurementUpdater::unitTestMode;
}

FloatType GliderVarioMeasurementUpdater::calcAltFromPressure(
		double pressure,
		double QFF,
		double temperatureAlt
		) {

	temperatureAlt += CtoK;

	return (temperatureAlt -(pow ((pressure / QFF),(1.0/BarometricFormulaExponent)) * temperatureAlt)) / TempLapseIndiffBoundLayer;
}

} /* namespace openEV */
