/*
 * GliderVarioTransitionMatrix.cpp
 *
 *  Created on: Dec 8, 2015
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

#include "GliderVarioTransitionMatrix.h"
#include "GliderVarioStatus.h"
#include "util/RotationMatrix.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;
#endif


namespace openEV
{


FloatType GliderVarioTransitionMatrix::staticRollTimeConstant = 2.0f;
FloatType GliderVarioTransitionMatrix::dynamicRollTimeConstant = 0.5f;

GliderVarioTransitionMatrix::GliderVarioTransitionMatrix ()
:transitionMatrix{GliderVarioStatus::STATUS_NUM_ROWS,GliderVarioStatus::STATUS_NUM_ROWS}
{

#if defined HAVE_LOG4CXX_H
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Kalman.GliderVarioTransitionMatrix");
	}
#endif /* HAVE_LOG4CXX_H */


}


GliderVarioTransitionMatrix::~GliderVarioTransitionMatrix ()
{

}

void
GliderVarioTransitionMatrix::updateStatus (
        GliderVarioStatus const &oldStatus ,
        GliderVarioStatus &newStatus,
        FloatType timeDiff
){
        calcTransitionMatrixAndStatus(timeDiff,oldStatus,newStatus);

        newStatus.getErrorCovariance_P() = transitionMatrix * oldStatus.getErrorCovariance_P() * transitionMatrix.transpose()
			          //+ oldStatus.getSystemNoiseCovariance_Q()
			          ;

        // Addition has to be added as a seperate statement. Eigen 3.3.4 did not like adding the noise covariance in the statement above.
        newStatus.getErrorCovariance_P() += oldStatus.getSystemNoiseCovariance_Q();

        newStatus.getSystemNoiseCovariance_Q() = oldStatus.getSystemNoiseCovariance_Q();

        newStatus.normalizeStatus();

    	LOG4CXX_DEBUG(logger,"GliderVarioTransitionMatrix::updateStatus"
    			<< "\n--------------------------------------------------------------------");
    	LOG4CXX_DEBUG(logger,"latitude = " << newStatus.latitude());
    	LOG4CXX_DEBUG(logger,"longitude = " << newStatus.longitude());
    	LOG4CXX_DEBUG(logger,"StatusVector = \n" << printStatusVector(newStatus.getStatusVector_x()));
    	LOG4CXX_TRACE(logger,"ErrorCovariance = \n" << printCovMatrix(newStatus.getErrorCovariance_P()));
    	LOG4CXX_TRACE(logger,"transitionMatrix = \n" << printCovMatrix(transitionMatrix));
    	LOG4CXX_TRACE(logger,"SystemNoiseCovariance = \n" << printCovMatrix(newStatus.getSystemNoiseCovariance_Q()));


}


void
GliderVarioTransitionMatrix::calcTransitionMatrixAndStatus (
        FloatType                timeDiff,
        GliderVarioStatus const &lastStatus,
        GliderVarioStatus       &newStatus)
{
    // I need the square of the time multiple times when calculating distance from acceleration
    //FloatType timeDiffSquare = timeDiff * timeDiff;

    // I need a conversion from the plane coordinates into the world coordinates
    //RotationMatrix rotMatrix (lastStatus.heading,lastStatus.pitchAngle,lastStatus.rollAngle);
    //RotationMatrix3DType &rotMatrixPlaneToWorld = rotMatrix.getMatrixPlaneToGlo();

    // For the EKF I need an approximate derivation of the rotation matrix for the roll, pitch and yaw angles.
    // For practical reasons I approximate the derivation by an increment of 1 degree.
    //RotationMatrix rotMatrixIncX (lastStatus.heading,lastStatus.pitchAngle,lastStatus.rollAngle + 1.0f);
    //RotationMatrix3DType &rotMatrixPlaneToWorldIncX = rotMatrixIncX.getMatrixPlaneToGlo();
    //RotationMatrix rotMatrixIncY (lastStatus.heading,lastStatus.pitchAngle + 1.0f,lastStatus.rollAngle);
    //RotationMatrix3DType &rotMatrixPlaneToWorldIncY = rotMatrixIncY.getMatrixPlaneToGlo();
    //RotationMatrix rotMatrixIncZ (lastStatus.heading + 1.0f,lastStatus.pitchAngle,lastStatus.rollAngle);
    //RotationMatrix3DType &rotMatrixPlaneToWorldIncZ = rotMatrixIncZ.getMatrixPlaneToGlo();

    // I need a conversion from the plane coordinates into the heading coordinates, i.e. I factor in pitch and roll, but not heading
    // RotationMatrix rotMatrixHeading (0.0f,lastStatus.pitchAngle,lastStatus.rollAngle);
    // RotationMatrix3DType &rotMatrixPlaneToHeading = rotMatrixHeading.getMatrixPlaneToGlo();

    // For the EKF I need an approximate derivation of the rotation matrix for the roll, pitch and yaw.
    // For practical reasons I approximate the derivation by an increment of 1 degree.
    //RotationMatrix rotMatrixHeadingIncX (0.0f,lastStatus.pitchAngle,lastStatus.rollAngle + 1.0f);
    //RotationMatrix3DType &rotMatrixPlaneToHeadingIncX = rotMatrixHeadingIncX.getMatrixPlaneToGlo();
    //RotationMatrix rotMatrixHeadingIncY (0.0f,lastStatus.pitchAngle + 1.0f,lastStatus.rollAngle);
    //RotationMatrix3DType &rotMatrixPlaneToHeadingIncY = rotMatrixHeadingIncY.getMatrixPlaneToGlo();
    //RotationMatrix rotMatrixHeadingIncZ (1.0f,lastStatus.pitchAngle,lastStatus.rollAngle);
    //RotationMatrix3DType &rotMatrixPlaneToHeadingIncZ = rotMatrixHeadingIncY.getMatrixPlaneToGlo();

    // I need half of time square for distance calculations based on acceleration here and there :)

    // FloatType timeSquareHalf  = timeDiff*timeDiff / 2.0f;

    // I am using a number of temporary variables to store factors used for new status calculation, and to store in the transition matrix.
    FloatType temp1;
    FloatType sinHeading = FastMath::fastSin(lastStatus.heading);
    FloatType cosHeading = FastMath::fastCos(lastStatus.heading);

    // First copy the coordinate bases and conversion of longitude sec to m to the new status
    newStatus.latitudeBaseArcSec = lastStatus.latitudeBaseArcSec;
    newStatus.longitudeBaseArcSec = lastStatus.longitudeBaseArcSec;
    newStatus.lenLongitudeArcSec = lastStatus.lenLongitudeArcSec;

    // OK, now systematically propagate the status based on previous status, and the elapsed time
    // Constant factors in comments have been moved to the class constructor. They will not change, and have to be set only once.

    // STATUS_IND_GRAVITY
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 1.0f;
    newStatus.gravity = lastStatus.gravity;

    // STATUS_IND_LATITUDE
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_LATITUDE_OFFS) = 1.0f;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = timeDiff;

    /// \todo Re-activate when possible
    // transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp2 = timeSquareHalf * cosHeading;

    // The angles have an indirect effect on the new status by means of the rotation matrix with the accelerations
    // Do a direct derivation of cos.
    /// \todo Re-activate when possible
    //transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_HEADING) =
    //        timeSquareHalf * lastStatus.accelHeading * (-sinHeading * FastMath::degToRad);

    newStatus.latitudeOffs = lastStatus.latitudeOffs
    		+ timeDiff * lastStatus.groundSpeedNorth
		    /// \todo Re-activate when possible
            // + temp2 * lastStatus.accelHeading
			;

    // STATUS_IND_LONGITUDE
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS) = 1.0f;

    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_SPEED_GROUND_E) = timeDiff;

    /// \todo Re-activate when possible
    //transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp2 = timeSquareHalf * sinHeading;

    // The angles have an indirect effect on the new status by means of the rotation matrix with the accelerations
    // I calculate the derivate of sin as cos.
    /// \todo Re-activate when possible
    //transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_HEADING) =
    //        timeSquareHalf * lastStatus.accelHeading * (cosHeading * FastMath::degToRad);

    /// \todo Re-activate when possible
    newStatus.longitudeOffs =
            lastStatus.longitudeOffs
            + timeDiff * lastStatus.groundSpeedEast
		    /// \todo Re-activate when possible
            // + temp2 * lastStatus.accelHeading
			;

    // STATUS_IND_ALT_MSL
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL) = 1.0f;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = -timeDiff;
    /// \todo Re-activate when possible
    // transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_VERTICAL) =  -timeSquareHalf;

    newStatus.altMSL =
            lastStatus.altMSL
            - timeDiff * lastStatus.verticalSpeed
		    /// \todo Re-activate when possible
            //- timeSquareHalf * lastStatus.accelVertical
			;

    // STATUS_IND_PITCH
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_PITCH) = 1.0f;

    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff;

    newStatus.pitchAngle = lastStatus.pitchAngle +
            lastStatus.pitchRateY * timeDiff;

    // STATUS_IND_ROLL
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROLL) = 1.0f;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff;

    newStatus.rollAngle = lastStatus.rollAngle +
            timeDiff * lastStatus.rollRateX;

    /*
     * Special consideration of propagating the roll (bank) angle
     * ----------------------------------------------------------
     *
     * OK, here's some really weird stuff.
     *
     * One of my nagging pains in the ass with this program is the attitude of the plane, especially the roll (bank) angle of my plane.
     *
     * Since we glider pilots spend a *lot* time circling the knowledge of the roll (bank) angle is crucial
     * because at a serious bank angle (30-50 deg is quite normal) the inertial gyro measurements become a mix of pitch up, and yaw, where in fact
     * these combined ones result in a flat horizontal turn. The centrifugal force becomes a major factor, as it determines our bank angle for flying clean
     * circles in the end.
     *
     * There is no direct measurement of the bank angle. My instrument simply cannot look out of the canopy as we do all the time to assess
     * the bank against the horizon. (Even if there was a horizon sensor, I would not trust it a lot in the Alps).
     * Naively trusting the accelerometer for acceleration along the Y axis as for someone standing on the ground playing Pokemon Go is foolish:
     * As long as we fly with the yaw string straight,
     * as we do all the time as good glider pilots there should be virtually no lateral acceleration on the Y axis. Only the acceleration
     * downward relative to the plane changes depending on how steep and fast we are turning.
     * The most reliable way to assess the bank angle is computing the centrifugal force from the turn rate, and the TAS.
     * This way I can assess the bank angle even from GPS speed, and changes of my (GPS) course over ground.
     *
     * However sometimes we may do a slip, i.e. fly with an intentionally hanging wing, or the plane sits still on the ground with one wing on the ground.
     * Both cases still impose a static roll component which I want to properly factor in.
     *
     * On the other side I do want to preserve the standard model of a constant bank angle which changes on short term driven by the roll rate from the
     * gyro.
     *
     * Therefore I approach this with a hybrid solution: Basically I assume the bank is constant, and changed primarily by the gyro roll rate.
     * As correction I use a 'rubberband effect' from the static bank (lateral acceleration) and the turn induced dynamic bank. The factor for these
     * corrections comes from the difference of the calculated value, and the current model value. It is further calculated from a configurable time constant
     * and the current time interval.
     * I do not want bumps in the aerial road (i.e. short lateral accelerations) affect the bank angle arbitrarily I make the time constant for lateral
     * acceleration rather long, but the time constant for the turn rate correction rather short in order to get (modeled) reality and model in line rather
     * quickly.
     *
     * /

  {
	FloatType bankAngleRot, staticAngle;

	bankAngleRot = calcRotBankAngle(lastStatus.yawRateGloZ,lastStatus.trueAirSpeed);

	// calculate the correction for dynamic bank
	temp1 = (bankAngleRot - newStatus.rollAngle) / dynamicRollTimeConstant * timeDiff;
	// ... and the covariant factors for the TAS and turn rate
	temp2 = calcRotBankAngle(lastStatus.yawRateGloZ,lastStatus.trueAirSpeed + 0.1);
	transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_TAS) =
			((temp2 - newStatus.rollAngle) / dynamicRollTimeConstant * timeDiff - temp1) * 10;
	temp2 = calcRotBankAngle(lastStatus.yawRateGloZ + 0.1,lastStatus.trueAirSpeed);
	transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z) =
			((temp2 - newStatus.rollAngle) / dynamicRollTimeConstant * timeDiff - temp1) * 10;

	// calculate the correction for static bank
	/// \todo Static bank angle is currently calculated correctly when there is no bank otherwise.
	staticAngle = FastMath::fastASin(lastStatus.accelY/GRAVITY);
	temp3 = staticAngle / staticRollTimeConstant * timeDiff;
	// ... and the covariant factors for the lateral acceleration
	temp2 = FastMath::fastASin((lastStatus.accelY + 0.01)/GRAVITY) / staticRollTimeConstant * timeDiff;
	transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ACC_Y) =
			(temp2 - temp3) * 100;

	// Now apply the corrections to the new status
	newStatus.rollAngle += temp1 + temp3;

  }
     */

    // STATUS_IND_HEADING
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_HEADING) = 1.0f;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff;

    newStatus.heading = lastStatus.heading +
    		timeDiff * lastStatus.yawRateZ;

    // STATUS_IND_SPEED_GROUND_N
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_TAS) = cosHeading;
    /// \todo Re-activate when possible
    // transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp2 = cosHeading * timeDiff;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1.0f;

    // angular change to the covariant
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_HEADING) =
            (lastStatus.trueAirSpeed
            	    /// \todo Re-activate when possible
            		// + lastStatus.accelHeading * timeDiff
					)
            * (-sinHeading) * FastMath::degToRad;

    newStatus.groundSpeedNorth =
            lastStatus.trueAirSpeed * cosHeading
		    /// \todo Re-activate when possible
            // + lastStatus.accelHeading * temp2
            + lastStatus.windSpeedNorth;

    // STATUS_IND_SPEED_GROUND_E
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_TAS) = sinHeading;
    /// \todo Re-activate when possible
    // transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp2 = sinHeading * timeDiff;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1.0f;

    // angular change to the covariant
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_HEADING) =
            (lastStatus.trueAirSpeed
            	    /// \todo Re-activate when possible
            		// + lastStatus.accelHeading * timeDiff
					)
            * cosHeading * FastMath::degToRad;

    newStatus.groundSpeedEast =
            lastStatus.trueAirSpeed * sinHeading
    	    /// \todo Re-activate when possible
            // + lastStatus.accelHeading * temp2
            + lastStatus.windSpeedEast;

    // STATUS_IND_TAS

    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_TAS) = 1.0f;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ACC_HEADING) = timeDiff;

    newStatus.trueAirSpeed = lastStatus.trueAirSpeed +
            timeDiff * lastStatus.accelHeading;

    /*
     * STATUS_IND_RATE_OF_SINK
     * The calculation is based on the energy transfer from kinetic energy to potential energy (increase of speed leads to increase of sink).
     * So IMHO this is a pretty crude approximation because it is not taking the changing drag with speed into account.
     * But it is a lot better than nothing.
     */
    /// \todo Calculation of Rate of Sink: Refine the vario compensation by considering the decrease of drag based on the polar.

    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp1 = lastStatus.trueAirSpeed/GRAVITY;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_TAS) = lastStatus.accelHeading/GRAVITY;

    newStatus.rateOfSink =
            temp1 * lastStatus.accelHeading;

    // STATUS_IND_VERTICAL_SPEED
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1.0f;

    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_VERTICAL) = timeDiff;

    newStatus.verticalSpeed = lastStatus.verticalSpeed +
            timeDiff * lastStatus.accelVertical;

    // STATUS_IND_THERMAL_SPEED
    /// \todo Re-activate when possible
    // transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1.0f;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 0.0f;
    // transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = -1.0f;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = 0.0f;

    newStatus.thermalSpeed = lastStatus.verticalSpeed - lastStatus.rateOfSink;

    // STATUS_IND_ACC_HEADING
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ACC_HEADING,GliderVarioStatus::STATUS_IND_ACC_HEADING) = 0.9f;

    newStatus.accelHeading = lastStatus.accelHeading * 0.9f;

    // STATUS_IND_ACC_CROSS
    /// \todo Re-activate when possible
    // transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ACC_CROSS,GliderVarioStatus::STATUS_IND_ACC_CROSS) = 1.0f;
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ACC_CROSS,GliderVarioStatus::STATUS_IND_ACC_CROSS) = 0.0f;

    /// \todo Re-activate when possible
    // newStatus.accelCross = lastStatus.accelCross;
    newStatus.accelCross = 0.0f;

    // STATUS_IND_ACC_VERTICAL
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ACC_VERTICAL,GliderVarioStatus::STATUS_IND_ACC_VERTICAL) = 0.9f;

    newStatus.accelVertical = lastStatus.accelVertical * 0.9f;

    // STATUS_IND_ROTATION_X
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_X,GliderVarioStatus::STATUS_IND_ROTATION_X) = 1.0f;

    newStatus.rollRateX = lastStatus.rollRateX;

    // STATUS_IND_ROTATION_Y
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_Y,GliderVarioStatus::STATUS_IND_ROTATION_Y) = 1.0f;

    newStatus.pitchRateY = lastStatus.pitchRateY;

    // STATUS_IND_ROTATION_GLO_Z
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_ROTATION_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1.0f;

    newStatus.yawRateZ = lastStatus.yawRateZ;

    // STATUS_IND_GYRO_BIAS_X
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = 1.0f;

    newStatus.gyroBiasX = lastStatus.gyroBiasX;

    // STATUS_IND_GYRO_BIAS_Y
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = 1.0f;

    newStatus.gyroBiasY = lastStatus.gyroBiasY;

    // STATUS_IND_GYRO_BIAS_Z
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = 1.0f;

    newStatus.gyroBiasZ = lastStatus.gyroBiasZ;

    // STATUS_IND_MAGNETIC_DECLINATION
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION,GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION) = 1.0f;

    newStatus.magneticDeclination = lastStatus.magneticDeclination;

    // STATUS_IND_MAGNETIC_INCLINATION
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION,GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION) = 1.0f;

    newStatus.magneticInclination = lastStatus.magneticInclination;

    // STATUS_IND_COMPASS_DEVIATION_X
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X) = 1.0f;

    newStatus.compassDeviationX = lastStatus.compassDeviationX;

    // STATUS_IND_COMPASS_DEVIATION_Y
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y) = 1.0f;

    newStatus.compassDeviationY = lastStatus.compassDeviationY;

    // STATUS_IND_COMPASS_DEVIATION_Z
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z) = 1.0f;

    newStatus.compassDeviationZ = lastStatus.compassDeviationZ;

    // STATUS_IND_WIND_SPEED_N
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_WIND_SPEED_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1.0f;

    newStatus.windSpeedNorth = lastStatus.windSpeedNorth;

    // STATUS_IND_WIND_SPEED_E
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_WIND_SPEED_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1.0f;

    newStatus.windSpeedEast = lastStatus.windSpeedEast;

    // STATUS_IND_QFF
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) = 1.0f;

    newStatus.qff = lastStatus.qff;

    // STATUS_IND_LAST_PRESSURE
    transitionMatrix.coeffRef(GliderVarioStatus::STATUS_IND_LAST_PRESSURE,GliderVarioStatus::STATUS_IND_LAST_PRESSURE) = 1.0f;

    newStatus.lastPressure = lastStatus.lastPressure;

}

} // namespace openEV
