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
#include "FastMath.h"

namespace openEV
{

  /**
   * The rough length of a arc second latitude in meter at 45deg North.
   * \sa <a href="https://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude" >Length of a degree of longitude</a>
   */
  FloatType constexpr lenLatitudeArcSec = 111132.0 / 3600.0;

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
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_X) = temp2 = timeSquareHalf2Lat * rotMatrixPlaneToWorld(0,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_Y) = temp3 = timeSquareHalf2Lat * rotMatrixPlaneToWorld(0,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_Z) = temp4 = timeSquareHalf2Lat * rotMatrixPlaneToWorld(0,2);

  // The angles have an indirect effect on the new status by means of the rotation matrix with the accelerations
  // However for the covariance I approximate the derivate by a small difference (here 1 degree).
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ROLL) =
		  timeSquareHalf2Lat * lastStatus.accelX * (rotMatrixPlaneToWorldIncX(0,0) - rotMatrixPlaneToWorld(0,0));
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_PITCH) =
		  timeSquareHalf2Lat * lastStatus.accelY * (rotMatrixPlaneToWorldIncY(0,1) - rotMatrixPlaneToWorld(0,1));
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_HEADING) =
		  timeSquareHalf2Lat * lastStatus.accelZ * (rotMatrixPlaneToWorldIncZ(0,2) - rotMatrixPlaneToWorld(0,2));


  newStatus.latitude = lastStatus.latitude +
		  temp1 * lastStatus.groundSpeedNorth +
		  temp2 * lastStatus.accelX +
		  temp3 * lastStatus.accelY +
		  temp4 * lastStatus.accelZ;

// STATUS_IND_LONGITUDE
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_LONGITUDE) = 1.0f;

  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_E) = temp1 = timeDiff / lenLongitudeArcSec ;

  FloatType timeSquareHalf2Lon = timeSquareHalf / lenLongitudeArcSec;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_X) = temp2 = timeSquareHalf2Lon * rotMatrixPlaneToWorld(1,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_Y) = temp3 = timeSquareHalf2Lon * rotMatrixPlaneToWorld(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_Z) = temp4 = timeSquareHalf2Lon * rotMatrixPlaneToWorld(1,2);

  // The angles have an indirect effect on the new status by means of the rotation matrix with the accelerations
  // However for the covariance I approximate the derivate by an by a small difference (here 1 degree).
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ROLL) =
		  timeSquareHalf2Lat * lastStatus.accelX * (rotMatrixPlaneToWorldIncX(1,0) - rotMatrixPlaneToWorld(1,0));
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_PITCH) =
		  timeSquareHalf2Lat * lastStatus.accelY * (rotMatrixPlaneToWorldIncY(1,1) - rotMatrixPlaneToWorld(1,1));
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_HEADING) =
		  timeSquareHalf2Lat * lastStatus.accelZ * (rotMatrixPlaneToWorldIncZ(1,2) - rotMatrixPlaneToWorld(1,2));

  newStatus.longitude =
		  lastStatus.longitude +
		  temp1 * lastStatus.groundSpeedEast +
		  temp2 * lastStatus.accelX +
		  temp3 * lastStatus.accelY +
		  temp4 * lastStatus.accelZ;

// STATUS_IND_ALT_MSL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = temp1 = -timeDiff;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_X) = temp2 = -timeSquareHalf * (rotMatrixPlaneToWorld(2,0));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_Y) =  temp3 =-timeSquareHalf * (rotMatrixPlaneToWorld(2,1));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_Z) =  temp4 =-timeSquareHalf * (rotMatrixPlaneToWorld(2,2));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_GRAVITY) =  temp5 = -timeSquareHalf;

  // The angles have an indirect effect on the new status by means of the rotation matrix with the accelerations
  // However for the covariance I approximate the derivate by an by a small difference (here 1 degree).
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ROLL) =
		  -timeSquareHalf * lastStatus.accelX * (rotMatrixPlaneToWorldIncX(2,0) - rotMatrixPlaneToWorld(2,0));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_PITCH) =
		  -timeSquareHalf * lastStatus.accelY * (rotMatrixPlaneToWorldIncY(2,1) - rotMatrixPlaneToWorld(2,1));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_HEADING) =
		  -timeSquareHalf * lastStatus.accelZ * (rotMatrixPlaneToWorldIncZ(2,2) - rotMatrixPlaneToWorld(2,2));

  newStatus.altMSL =
		  lastStatus.altMSL +
		  temp1 * lastStatus.verticalSpeed +
		  temp2 * lastStatus.accelX +
		  temp3 * lastStatus.accelY +
		  temp4 * lastStatus.accelZ +
		  temp5 * lastStatus.gravity;

// STATUS_IND_PITCH
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_PITCH) = 1.0f;

  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y) = temp1 =  timeDiff * FastMath::fastCos(lastStatus.rollAngle);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Z) = temp2 = -timeDiff * FastMath::fastSin(lastStatus.rollAngle);

  // calculate the covariant for angular changes
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y) =
		  lastStatus.pitchRateY * (timeDiff * FastMath::fastCos(lastStatus.rollAngle + 1.0f) - temp1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Z) =
		  lastStatus.yawRateZ * (-timeDiff * FastMath::fastSin(lastStatus.rollAngle + 1.0f) - temp2);

  newStatus.pitchAngle = lastStatus.pitchAngle +
		  temp1 * lastStatus.pitchRateY +
		  temp2 * lastStatus.yawRateZ;

// STATUS_IND_ROLL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROLL) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff;

  newStatus.rollAngle = lastStatus.rollAngle +
		  timeDiff * lastStatus.rollRateX;

  // STATUS_IND_ROTATION_Z
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1.0f;

    newStatus.yawRateZ = lastStatus.yawRateZ;

  // STATUS_IND_ROTATION_GLO_Z
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_X) = temp1 = rotMatrixPlaneToWorld(2,0);
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_Y) = temp2 = rotMatrixPlaneToWorld(2,1);
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = temp3 = rotMatrixPlaneToWorld(2,2);

    // angular changes to the covariant
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROLL) =
  		  lastStatus.rollRateX  * (rotMatrixPlaneToWorldIncX(2,0) - rotMatrixPlaneToWorld(2,0));
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_PITCH) =
  		  lastStatus.pitchRateY * (rotMatrixPlaneToWorldIncY(2,1) - rotMatrixPlaneToWorld(2,1));
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_HEADING) =
  		  lastStatus.yawRateZ   * (rotMatrixPlaneToWorldIncZ(2,2) - rotMatrixPlaneToWorld(2,2));

    newStatus.yawRateGloZ =
  		  temp1 * lastStatus.rollRateX +
  		  temp2 * lastStatus.pitchRateY +
  		  temp3 * lastStatus.yawRateZ;

// STATUS_IND_HEADING
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_HEADING) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z) = temp1 = timeDiff;

  newStatus.heading = lastStatus.heading +
		  temp1 * lastStatus.yawRateGloZ;

// STATUS_IND_SPEED_GROUND_N
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_TAS_N) = 1.0f;
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1.0f;
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_X) = temp1 = timeDiff * rotMatrixPlaneToWorld(0,0);
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_Y) = temp2 = timeDiff * rotMatrixPlaneToWorld(0,1);
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_Z) = temp3 = timeDiff * rotMatrixPlaneToWorld(0,2);

	// calculate the covariant for angular changes
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ROLL) =
		  timeDiff * lastStatus.accelX * (rotMatrixPlaneToWorldIncX(0,0) - rotMatrixPlaneToWorld(0,0));
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_PITCH) =
		  timeDiff * lastStatus.accelY * (rotMatrixPlaneToWorldIncY(0,1) - rotMatrixPlaneToWorld(0,1));
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_HEADING) =
		  timeDiff * lastStatus.accelZ * (rotMatrixPlaneToWorldIncZ(0,2) - rotMatrixPlaneToWorld(0,2));

	newStatus.groundSpeedNorth = lastStatus.trueAirSpeedNorth + lastStatus.windSpeedNorth +
		  temp1 * lastStatus.accelX +
		  temp2 * lastStatus.accelY +
		  temp3 * lastStatus.accelZ;

// STATUS_IND_SPEED_GROUND_E
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_TAS_E) = 1.0f;
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1.0f;
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_X) = temp1 = timeDiff * rotMatrixPlaneToWorld(1,0);
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_Y) = temp2 = timeDiff * rotMatrixPlaneToWorld(1,1);
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_Z) = temp3 = timeDiff * rotMatrixPlaneToWorld(1,2);

	// calculate the covariant for angular changes
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ROLL) =
		  timeDiff * lastStatus.accelX * (rotMatrixPlaneToWorldIncX(1,0) - rotMatrixPlaneToWorld(1,0));
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_PITCH) =
		  timeDiff * lastStatus.accelY * (rotMatrixPlaneToWorldIncY(1,1) - rotMatrixPlaneToWorld(1,1));
	transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_HEADING) =
		  timeDiff * lastStatus.accelZ * (rotMatrixPlaneToWorldIncZ(1,2) - rotMatrixPlaneToWorld(1,2));

	newStatus.groundSpeedNorth = lastStatus.trueAirSpeedEast + lastStatus.windSpeedEast +
		  temp1 * lastStatus.accelX +
		  temp2 * lastStatus.accelY +
		  temp3 * lastStatus.accelZ;

// STATUS_IND_TAS
  // I need a conversion from the plane coordinates into the heading coordinates, i.e. I factor in pitch and roll, but not heading
  RotationMatrix rotMatrix (0.0f,lastStatus.pitchAngle,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToHeading = rotMatrix.getMatrixPlaneToGlo();

  // For the EKF I need an approximate derivation of the rotation matrix for the roll, pitch ands.
  // For practical reasons I approximate the derivation by an increment of 1 degree.
  RotationMatrix rotMatrixPlaneToHeadingIncX (lastStatus.heading,lastStatus.pitchAngle,lastStatus.rollAngle + 1.0f);
  RotationMatrix3DType &rotMatrixPlaneToHeadingIncX = rotMatrixPlaneToHeadingIncX.getMatrixPlaneToGlo();
  RotationMatrix rotMatrixPlaneToHeadingIncY (lastStatus.heading,lastStatus.pitchAngle + 1.0f,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToHeadingIncY = rotMatrixPlaneToHeadingIncY.getMatrixPlaneToGlo();


  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_TAS) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ACC_X) = temp1 = timeDiff * rotMatrixPlaneToHeading(1,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ACC_Y) = temp2 = timeDiff * rotMatrixPlaneToHeading(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ACC_Z) = temp3 = timeDiff * rotMatrixPlaneToHeading(1,2);

  // calculate the covariant for angular changes
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ROLL) =
		  timeDiff * lastStatus.accelX * (rotMatrixPlaneToHeadingIncX(1,0) - rotMatrixPlaneToHeading(1,0));
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_PITCH) =
		  timeDiff * lastStatus.accelY * (rotMatrixPlaneToHeadingIncY(1,1) - rotMatrixPlaneToHeading(1,1));

  newStatus.trueAirSpeed = lastStatus.trueAirSpeed +
		  temp1 * lastStatus.accelX +
		  temp2 * lastStatus.accelY +
		  temp3 * lastStatus.accelZ;

  // STATUS_IND_TAS_N
    transitionMatrix(GliderVarioStatus::STATUS_IND_TAS_N,GliderVarioStatus::STATUS_IND_TAS) = temp1 = FastMath::fastCos(lastStatus.heading);

    // Covariance for angular change
    transitionMatrix(GliderVarioStatus::STATUS_IND_TAS_N,GliderVarioStatus::STATUS_IND_HEADING) = lastStatus.trueAirSpeed * (FastMath::fastCos(lastStatus.heading + 1.0f) - temp1);

    newStatus.trueAirSpeedNorth = temp1 * lastStatus.trueAirSpeed;

  // STATUS_IND_TAS_E
    transitionMatrix(GliderVarioStatus::STATUS_IND_TAS_E,GliderVarioStatus::STATUS_IND_TAS) = FastMath::fastSin(lastStatus.heading);

    // Covariance for angular change
    transitionMatrix(GliderVarioStatus::STATUS_IND_TAS_E,GliderVarioStatus::STATUS_IND_HEADING) = lastStatus.trueAirSpeed * (FastMath::fastSin(lastStatus.heading + 1.0f) - temp1);

    newStatus.trueAirSpeedEast = temp1 * lastStatus.trueAirSpeed;

  // STATUS_IND_SPEED_GROUND_N
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_TAS_N) = 1.0f;
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1.0f;
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_X) = temp1 = timeDiff * rotMatrixPlaneToWorld(0,0);
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_Y) = temp2 = timeDiff * rotMatrixPlaneToWorld(0,1);
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_Z) = temp3 = timeDiff * rotMatrixPlaneToWorld(0,2);

      // calculate the covariant for angular changes
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ROLL) =
    		  timeDiff * lastStatus.accelX * (rotMatrixPlaneToWorldIncX(0,0) - rotMatrixPlaneToWorld(0,0));
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_PITCH) =
    		  timeDiff * lastStatus.accelY * (rotMatrixPlaneToWorldIncY(0,1) - rotMatrixPlaneToWorld(0,1));
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_HEADING) =
    		  timeDiff * lastStatus.accelZ * (rotMatrixPlaneToWorldIncZ(0,2) - rotMatrixPlaneToWorld(0,2));

      newStatus.groundSpeedNorth = lastStatus.trueAirSpeedNorth + lastStatus.windSpeedNorth +
    		  temp1 * lastStatus.accelX +
    		  temp2 * lastStatus.accelY +
    		  temp3 * lastStatus.accelZ;

  // STATUS_IND_SPEED_GROUND_E
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_TAS_E) = 1.0f;
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1.0f;
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_X) = temp1 = timeDiff * rotMatrixPlaneToWorld(1,0);
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_Y) = temp2 = timeDiff * rotMatrixPlaneToWorld(1,1);
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_Z) = temp3 = timeDiff * rotMatrixPlaneToWorld(1,2);

      // calculate the covariant for angular changes
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ROLL) =
    		  timeDiff * lastStatus.accelX * (rotMatrixPlaneToWorldIncX(1,0) - rotMatrixPlaneToWorld(1,0));
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_PITCH) =
    		  timeDiff * lastStatus.accelY * (rotMatrixPlaneToWorldIncY(1,1) - rotMatrixPlaneToWorld(1,1));
      transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_HEADING) =
    		  timeDiff * lastStatus.accelZ * (rotMatrixPlaneToWorldIncZ(1,2) - rotMatrixPlaneToWorld(1,2));

      newStatus.groundSpeedNorth = lastStatus.trueAirSpeedEast + lastStatus.windSpeedEast +
    		  temp1 * lastStatus.accelX +
    		  temp2 * lastStatus.accelY +
    		  temp3 * lastStatus.accelZ;

  /*
   * STATUS_IND_RATE_OF_SINK
   * The calculation is based on the energy transfer from kinetic energy to potential energy (increase of speed leads to increase of sink).
   * The acceleration is measured along the body X axis. If pitched up or down the accelerometer reading is affected by gravity.
   * The actual equation is something like: (accX - sin(pitch)*GRAVITY) * TAS / GRAVITY = (accX*TAS/GRAVITY) - sin(pitch)*TAS*GRAVITY/GRAVITY
   * So IMHO this is a pretty crude approximation because I assume that my TAS is exactly along the X axis which is rarely accurate due to
   * changing angles of attack.
   */
  /// \todo Calculation of Rate of Sink: Refine the vario compensation by considering the decrease of drag based on the polar.
  transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_ACC_X) = temp1 = lastStatus.trueAirSpeed/GRAVITY;
  transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_TAS) = temp2 = -FastMath::fastSin(lastStatus.pitchAngle);

  // derivate of angle change for the covariance
  transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_PITCH) = lastStatus.trueAirSpeed * (-FastMath::fastSin(lastStatus.pitchAngle + 1.0f) - temp2);

  newStatus.rateOfSink =
		  temp1 * lastStatus.accelX +
		  temp2 * lastStatus.trueAirSpeed;

// STATUS_IND_VERTICAL_SPEED
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1.0f;

  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_X) = temp1 = timeDiff * rotMatrixPlaneToWorld(2,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_Y) = temp2 = timeDiff * rotMatrixPlaneToWorld(2,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_Z) = temp3 = timeDiff * rotMatrixPlaneToWorld(2,2);

  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_GRAVITY) = temp4 = timeDiff * GRAVITY;

  // Derivates for angular changes in the covariant
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ROLL) =
		  -timeDiff * lastStatus.accelX * (rotMatrixPlaneToWorldIncX(2,0) - rotMatrixPlaneToWorld(2,0));
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_PITCH) =
		  -timeDiff * lastStatus.accelY * (rotMatrixPlaneToWorldIncY(2,1) - rotMatrixPlaneToWorld(2,1));
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_HEADING) =
		  -timeDiff * lastStatus.accelZ * (rotMatrixPlaneToWorldIncZ(2,2) - rotMatrixPlaneToWorld(2,2));

  newStatus.verticalSpeed = lastStatus.verticalSpeed +
		  temp1 * lastStatus.accelX +
		  temp2 * lastStatus.accelY +
		  temp3 * lastStatus.accelZ +
		  temp4 * lastStatus.gravity;

// STATUS_IND_THERMAL_SPEED
  transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = -1.0f;

  newStatus.thermalSpeed = lastStatus.verticalSpeed - lastStatus.rateOfSink;

// STATUS_IND_ACC_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_X,GliderVarioStatus::STATUS_IND_ACC_X) = 1.0f;

  newStatus.accelX = lastStatus.accelX;

// STATUS_IND_ACC_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Y,GliderVarioStatus::STATUS_IND_ACC_Y) = 1.0f;

  newStatus.accelY = lastStatus.accelY;

// STATUS_IND_ACC_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Z,GliderVarioStatus::STATUS_IND_ACC_Z) = 1.0f;

  newStatus.accelZ = lastStatus.accelZ;

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

  // STATUS_IND_COMPASS_ERROR
  transitionMatrix(GliderVarioStatus::STATUS_IND_COMPASS_ERROR,GliderVarioStatus::STATUS_IND_COMPASS_ERROR) = 1.0f;

  newStatus.compassError = lastStatus.compassError;

// STATUS_IND_WIND_SPEED_N
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_N,GliderVarioStatus::STATUS_IND_WIND_SPEED) = temp1 = FastMath::fastCos(lastStatus.windDirection);

  // angular change in the covariant
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_N,GliderVarioStatus::STATUS_IND_WIND_DIR) =
		  lastStatus.windSpeed * (FastMath::fastCos(lastStatus.windDirection + 1.0f) - temp1) ;

  newStatus.windSpeedNorth = temp1 * lastStatus.windSpeed;

// STATUS_IND_WIND_SPEED_E
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_E,GliderVarioStatus::STATUS_IND_WIND_SPEED) = temp1 = FastMath::fastSin(lastStatus.windDirection);

  // angular change in the covariant
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_E,GliderVarioStatus::STATUS_IND_WIND_DIR) =
		  lastStatus.windSpeed * (FastMath::fastSin(lastStatus.windDirection + 1.0f) - temp1) ;

  newStatus.windSpeedEast = temp1 * lastStatus.windSpeed;


  // I will nudge the speed and direction statistically with the Kalman gain.
// STATUS_IND_WIND_SPEED
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED,GliderVarioStatus::STATUS_IND_WIND_SPEED) = 1.0f;

  newStatus.windSpeed = lastStatus.windSpeed;

// STATUS_IND_WIND_DIR
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_DIR,GliderVarioStatus::STATUS_IND_WIND_DIR) = 1.0f;

  newStatus.windDirection = lastStatus.windDirection;

  // STATUS_IND_QNH
  transitionMatrix(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) = 1.0f;

  newStatus.qff = lastStatus.qff;


}

} // namespace openEV
