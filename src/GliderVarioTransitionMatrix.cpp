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

#include <GliderVarioTransitionMatrix.h>
#include "GliderVarioStatus.h"
#include "RotationMatrix.h"

namespace openEV
{

  /**
   * The rough length of a arc second latitude in meter at 45deg North.
   * \sa <a href="https://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude" >Length of a degree of longitude</a>
   */
  FloatType constexpr lenLatitudeArcSec = 111132.0 / 3600.0;

  FloatType GliderVarioTransitionMatrix::staticRollTimeConstant = 2.0f;
  FloatType GliderVarioTransitionMatrix::dynamicRollTimeConstant = 0.5f;


GliderVarioTransitionMatrix::~GliderVarioTransitionMatrix ()
{

}

void
GliderVarioTransitionMatrix::calcTransitionMatrixAndStatus (
    FloatType                timeDiff,
    GliderVarioStatus const &lastStatus,
	GliderVarioStatus       &newStatus)
{
  // I need the square of the time multiple times when calculating distance from acceleration
  FloatType timeDiffSquare = timeDiff * timeDiff;

  // I need a conversion from the plane coordinates into the world coordinates
  RotationMatrix rotMatrix (lastStatus.heading,lastStatus.pitchAngle,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToWorld = rotMatrix.getMatrixPlaneToGlo();

  // For the EKF I need an approximate derivation of the rotation matrix for the roll, pitch and yaw angles.
  // For practical reasons I approximate the derivation by an increment of 1 degree.
  RotationMatrix rotMatrixIncX (lastStatus.heading,lastStatus.pitchAngle,lastStatus.rollAngle + 1.0f);
  RotationMatrix3DType &rotMatrixPlaneToWorldIncX = rotMatrixIncX.getMatrixPlaneToGlo();
  RotationMatrix rotMatrixIncY (lastStatus.heading,lastStatus.pitchAngle + 1.0f,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToWorldIncY = rotMatrixIncY.getMatrixPlaneToGlo();
  RotationMatrix rotMatrixIncZ (lastStatus.heading + 1.0f,lastStatus.pitchAngle,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToWorldIncZ = rotMatrixIncZ.getMatrixPlaneToGlo();

  // I need a conversion from the plane coordinates into the heading coordinates, i.e. I factor in pitch and roll, but not heading
  RotationMatrix rotMatrixHeading (0.0f,lastStatus.pitchAngle,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToHeading = rotMatrixHeading.getMatrixPlaneToGlo();

  // For the EKF I need an approximate derivation of the rotation matrix for the roll, pitch and yaw.
  // For practical reasons I approximate the derivation by an increment of 1 degree.
  RotationMatrix rotMatrixHeadingIncX (0.0f,lastStatus.pitchAngle,lastStatus.rollAngle + 1.0f);
  RotationMatrix3DType &rotMatrixPlaneToHeadingIncX = rotMatrixHeadingIncX.getMatrixPlaneToGlo();
  RotationMatrix rotMatrixHeadingIncY (0.0f,lastStatus.pitchAngle + 1.0f,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToHeadingIncY = rotMatrixHeadingIncY.getMatrixPlaneToGlo();
  RotationMatrix rotMatrixHeadingIncZ (1.0f,lastStatus.pitchAngle,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToHeadingIncZ = rotMatrixHeadingIncY.getMatrixPlaneToGlo();


  // I need half of time square for distance calculations based on acceleration here and there :)
  FloatType timeSquareHalf  = timeDiff*timeDiff / 2.0f;
  FloatType const lenLongitudeArcSec = lenLatitudeArcSec * FastMath::fastCos(lastStatus.latitude);

  // I am using a number of temporary variables to store factors used for new status calculation, and to store in the transition matrix.
  FloatType temp1, temp2, temp3, temp4, temp5;

  // OK, now systematically propagate the status based on previous status, and the elapsed time
  // Constant factors in comments have been moved to the class constructor. They will not change, and have to be set only once.

// STATUS_IND_GRAVITY
  transitionMatrix(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 1.0f;

  newStatus.gravity = lastStatus.gravity;

// STATUS_IND_LATITUDE
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_LATITUDE) = 1.0f;

  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = temp1 = timeDiff / lenLatitudeArcSec;

  FloatType timeSquareHalf2Lat = timeSquareHalf / lenLatitudeArcSec;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp2 = timeSquareHalf2Lat * FastMath::fastCos(lastStatus.heading);

  // The angles have an indirect effect on the new status by means of the rotation matrix with the accelerations
  // Do a direct deviation of cos.
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ROLL) =
		  timeSquareHalf2Lat * lastStatus.accelHeading * (-FastMath::fastSin(lastStatus.heading));


  newStatus.latitude = lastStatus.latitude +
		  temp1 * lastStatus.groundSpeedNorth +
		  temp2 * lastStatus.accelHeading ;

// STATUS_IND_LONGITUDE
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_LONGITUDE) = 1.0f;

  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_E) = temp1 = timeDiff / lenLongitudeArcSec ;

  FloatType timeSquareHalf2Lon = timeSquareHalf / lenLongitudeArcSec;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp2 = timeSquareHalf2Lon * FastMath::fastSin(lastStatus.heading);

  // The angles have an indirect effect on the new status by means of the rotation matrix with the accelerations
  // I calculate the derivate of sin as cos.
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ROLL) =
		  timeSquareHalf2Lat * lastStatus.accelHeading * FastMath::fastCos(lastStatus.heading);

  newStatus.longitude =
		  lastStatus.longitude +
		  temp1 * lastStatus.groundSpeedEast +
		  temp2 * lastStatus.accelHeading;

// STATUS_IND_ALT_MSL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = -timeDiff;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_VERTICAL) =  -timeSquareHalf;

  newStatus.altMSL =
		  lastStatus.altMSL +
		  -timeDiff * lastStatus.verticalSpeed +
		  -timeSquareHalf * lastStatus.accelVertical;

// STATUS_IND_PITCH
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_PITCH) = 1.0f;

  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y) = temp1 = timeDiff * rotMatrixPlaneToHeading(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Z) = temp2 = timeDiff * rotMatrixPlaneToHeading(1,2);

  // calculate the covariant for angular changes
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y) =
		  lastStatus.pitchRateY * (timeDiff * rotMatrixPlaneToHeadingIncY(1,1) - temp1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Z) =
		  lastStatus.yawRateZ * (-timeDiff * rotMatrixPlaneToHeadingIncZ(1,2) - temp2);

  newStatus.pitchAngle = lastStatus.pitchAngle +
		  temp1 * lastStatus.pitchRateY +
		  temp2 * lastStatus.yawRateZ;

// STATUS_IND_ROLL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROLL) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff;

  newStatus.rollAngle = lastStatus.rollAngle +
		  timeDiff * lastStatus.rollRateX;


  // STATUS_IND_HEADING
    transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_HEADING) = 1.0f;
    transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z) = temp1 = timeDiff;

    newStatus.heading = lastStatus.heading +
  		  temp1 * lastStatus.yawRateGloZ;

  // STATUS_IND_ROTATION_Z
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1.0f;

    newStatus.yawRateZ = lastStatus.yawRateZ;

  // STATUS_IND_ROTATION_GLO_Z
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_X) = temp1 = rotMatrixPlaneToWorld(2,0);
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_Y) = temp2 = rotMatrixPlaneToWorld(2,1);
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = temp3 = rotMatrixPlaneToWorld(2,2);

    // angular changes to the covariant
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROLL) =
  		  lastStatus.rollRateX  * (rotMatrixPlaneToWorldIncX(2,0) - temp1);
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_PITCH) =
  		  lastStatus.pitchRateY * (rotMatrixPlaneToWorldIncY(2,1) - temp2);
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_HEADING) =
  		  lastStatus.yawRateZ   * (rotMatrixPlaneToWorldIncZ(2,2) - temp3);

    newStatus.yawRateGloZ =
  		  temp1 * lastStatus.rollRateX +
  		  temp2 * lastStatus.pitchRateY +
  		  temp3 * lastStatus.yawRateZ;

// STATUS_IND_SPEED_GROUND_N
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_TAS) = temp1 = FastMath::fastCos(lastStatus.heading);
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1.0f;

	// angular change to the covariant
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_HEADING) =
			lastStatus.trueAirSpeed * (FastMath::fastCos(lastStatus.heading + 1.0f) - temp1);

	newStatus.groundSpeedNorth = lastStatus.trueAirSpeed * temp1 + lastStatus.windSpeedNorth;

// STATUS_IND_SPEED_GROUND_E
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_TAS) = temp1 = FastMath::fastSin(lastStatus.heading);
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1.0f;

	// angular change to the covariant
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_HEADING) =
			lastStatus.trueAirSpeed * (FastMath::fastSin(lastStatus.heading + 1.0f) - temp1);

	newStatus.groundSpeedEast = lastStatus.trueAirSpeed * temp1 + lastStatus.windSpeedEast;

// STATUS_IND_TAS

  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_TAS) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp1 = timeDiff * rotMatrixPlaneToHeading(1,0);

  newStatus.trueAirSpeed = lastStatus.trueAirSpeed +
		  temp1 * lastStatus.accelHeading;


  /*
   * STATUS_IND_RATE_OF_SINK
   * The calculation is based on the energy transfer from kinetic energy to potential energy (increase of speed leads to increase of sink).
   * So IMHO this is a pretty crude approximation because it is not taking the changing drag with speed into account.
   * But it is a lot better than nothing.
   */
  /// \todo Calculation of Rate of Sink: Refine the vario compensation by considering the decrease of drag based on the polar.

  transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_ACC_HEADING) = temp1 = lastStatus.trueAirSpeed/GRAVITY;

  newStatus.rateOfSink =
		  temp1 * lastStatus.accelHeading;

// STATUS_IND_VERTICAL_SPEED
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1.0f;

  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_VERTICAL) = timeDiff;

  newStatus.verticalSpeed = lastStatus.verticalSpeed +
		  timeDiff * lastStatus.accelVertical;

// STATUS_IND_THERMAL_SPEED
  transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = -1.0f;

  newStatus.thermalSpeed = lastStatus.verticalSpeed - lastStatus.rateOfSink;

// STATUS_IND_ACC_HEADING
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_HEADING,GliderVarioStatus::STATUS_IND_ACC_HEADING) = 1.0f;

  newStatus.accelHeading = lastStatus.accelHeading;

// STATUS_IND_ACC_CROSS
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_CROSS,GliderVarioStatus::STATUS_IND_ACC_CROSS) = 1.0f;

  newStatus.accelCross = lastStatus.accelCross;

// STATUS_IND_ACC_VERTICAL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_VERTICAL,GliderVarioStatus::STATUS_IND_ACC_VERTICAL) = 1.0f;

  newStatus.accelVertical = lastStatus.accelVertical;

// STATUS_IND_ROTATION_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_X,GliderVarioStatus::STATUS_IND_ROTATION_X) = 1.0f;

  newStatus.rollRateX = lastStatus.rollRateX;

// STATUS_IND_ROTATION_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Y,GliderVarioStatus::STATUS_IND_ROTATION_Y) = 1.0f;

  newStatus.pitchRateY = lastStatus.pitchRateY;

// STATUS_IND_GYRO_BIAS_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = 1.0f;

  newStatus.gyroBiasX = lastStatus.gyroBiasX;

// STATUS_IND_GYRO_BIAS_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = 1.0f;

  newStatus.gyroBiasY = lastStatus.gyroBiasY;

// STATUS_IND_GYRO_BIAS_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = 1.0f;

  newStatus.gyroBiasZ = lastStatus.gyroBiasZ;

  // STATUS_IND_MAGNETIC_DECLINATION
    transitionMatrix(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION,GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION) = 1.0f;

    newStatus.magneticDeclination = lastStatus.magneticDeclination;

// STATUS_IND_MAGNETIC_INCLINATION
    transitionMatrix(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION,GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION) = 1.0f;

    newStatus.magneticInclination = lastStatus.magneticInclination;

// STATUS_IND_COMPASS_DEVIATION_X
	transitionMatrix(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X) = 1.0f;

	newStatus.compassDeviationX = lastStatus.compassDeviationX;

// STATUS_IND_COMPASS_DEVIATION_Y
	transitionMatrix(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y) = 1.0f;

	newStatus.compassDeviationY = lastStatus.compassDeviationY;

// STATUS_IND_COMPASS_DEVIATION_Z
	transitionMatrix(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z) = 1.0f;

	newStatus.compassDeviationZ = lastStatus.compassDeviationZ;

// STATUS_IND_WIND_SPEED_N
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1.0f;

  newStatus.windSpeedNorth = lastStatus.windSpeedNorth;

// STATUS_IND_WIND_SPEED_E
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1.0f;

  newStatus.windSpeedEast = lastStatus.windSpeedEast;


  // STATUS_IND_QFF
  transitionMatrix(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) = 1.0f;

  newStatus.qff = lastStatus.qff;

  // STATUS_IND_LAST_PRESSURE
  transitionMatrix(GliderVarioStatus::STATUS_IND_LAST_PRESSURE,GliderVarioStatus::STATUS_IND_LAST_PRESSURE) = 1.0f;

  newStatus.lastPressure = lastStatus.lastPressure;



}

} // namespace openEV
