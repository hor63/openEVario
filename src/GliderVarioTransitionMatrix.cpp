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
   * The rough length of a degree latitude in meter at 45deg North.
   * \ref https://en.wikipedia.org/wiki/Longitude#Noting_and_calculating_longitude
   */
  FloatType constexpr lenLatitude = 111132.0;

GliderVarioTransitionMatrix::~GliderVarioTransitionMatrix ()
{

}

void
GliderVarioTransitionMatrix::calcTransitionMatrix (
    FloatType timeDiff,
    GliderVarioStatus const &lastStatus)
{
  // I need the square of the time multiple times when calculating distance from acceleration
  FloatType timeDiffSquare = timeDiff * timeDiff;

  // I need a conversion from the plane coordinates into the world coordinates
  RotationMatrix rotMatrix (lastStatus.yawAngle,lastStatus.pitchAngle,lastStatus.rollAngle);
  RotationMatrix::RotationMatrixType &rotMatrixPlaneToWorld = rotMatrix.getMatrixPlaneToGlo();
  RotationMatrix::RotationMatrixType &rotMatrixWorldToPlane = rotMatrix.getMatrixGloToPlane();
  FloatType lenDegLongitude = 1852.0 * FastMath::fastCos(lastStatus.latitude);
  // I need half of time square for distance calculations based on acceleration here and there :)
  FloatType timeSquareHalf  = timeDiff*timeDiff / 2.0f;
  FloatType const lenLongitude = lenLatitude * FastMath::fastCos(lastStatus.latitude);



  // OK, now systematically propagate the status based on previous status, and the elapsed time

  //--STATUS_IND_LONGITUDE------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_LONGITUDE) = 1;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_E) = timeDiff / lenLongitude ;

  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_X) = timeSquareHalf*rotMatrixPlaneToWorld(1,0) / lenLongitude;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_Y) = timeSquareHalf*rotMatrixPlaneToWorld(1,1) / lenLongitude;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_Z) = timeSquareHalf*rotMatrixPlaneToWorld(1,2) / lenLongitude;


  //--STATUS_IND_LATITUDE------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_LATITUDE) = 1;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = timeDiff / lenLatitude;

  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_X) = timeSquareHalf*rotMatrixPlaneToWorld(0,0) / lenLatitude;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_Y) = timeSquareHalf*rotMatrixPlaneToWorld(0,1) / lenLatitude;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_Z) = timeSquareHalf*rotMatrixPlaneToWorld(0,2) / lenLatitude;


  //--STATUS_IND_ALT_MSL------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL) = 1;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = -timeDiff;

  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_X) = -timeSquareHalf* (rotMatrixPlaneToWorld(2,0));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_Y) = -timeSquareHalf* (rotMatrixPlaneToWorld(2,1));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_Z) = -timeSquareHalf* (rotMatrixPlaneToWorld(2,2));


  //--STATUS_IND_YAW------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_YAW) = 1;

  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff*rotMatrixPlaneToWorld(2,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff*rotMatrixPlaneToWorld(2,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff*rotMatrixPlaneToWorld(2,2);

  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = -timeDiff*rotMatrixPlaneToWorld(2,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = -timeDiff*rotMatrixPlaneToWorld(2,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = -timeDiff*rotMatrixPlaneToWorld(2,2);


  //--STATUS_IND_PITCH------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_PITCH) = 1;

  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff*rotMatrixPlaneToWorld(1,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff*rotMatrixPlaneToWorld(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff*rotMatrixPlaneToWorld(1,2);

  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = -timeDiff*rotMatrixPlaneToWorld(1,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = -timeDiff*rotMatrixPlaneToWorld(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = -timeDiff*rotMatrixPlaneToWorld(1,2);


  //--STATUS_IND_ROLL------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROLL) = 1;

  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff*rotMatrixPlaneToWorld(0,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff*rotMatrixPlaneToWorld(0,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff*rotMatrixPlaneToWorld(0,2);

  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = timeDiff*rotMatrixPlaneToWorld(0,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = timeDiff*rotMatrixPlaneToWorld(0,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = timeDiff*rotMatrixPlaneToWorld(0,2);


  //--STATUS_IND_SPEED_GROUND_N------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_TAS) = FastMath::fastCos(lastStatus.heading);

  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_X) = timeDiff * rotMatrixPlaneToWorld(0,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_Y) = timeDiff * rotMatrixPlaneToWorld(0,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_Z) = timeDiff * rotMatrixPlaneToWorld(0,2);

  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1;


  //--STATUS_IND_SPEED_GROUND_E------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_TAS) = FastMath::fastSin(lastStatus.heading);

  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_X) = timeDiff * rotMatrixPlaneToWorld(1,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_Y) = timeDiff * rotMatrixPlaneToWorld(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_Z) = timeDiff * rotMatrixPlaneToWorld(1,2);

  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1;


  //--STATUS_IND_TAS------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_TAS) = 1;

  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ACC_X) = timeDiff;


  //--STATUS_IND_HEADING------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_HEADING) = 1;

  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff * rotMatrixPlaneToWorld(2,0) ;
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff * rotMatrixPlaneToWorld(2,1) ;
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff * rotMatrixPlaneToWorld(2,2) ;

  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = -timeDiff * rotMatrixPlaneToWorld(2,0) ;
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = -timeDiff * rotMatrixPlaneToWorld(2,1) ;
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = -timeDiff * rotMatrixPlaneToWorld(2,2) ;


  //--STATUS_IND_RATE_OF_SINK------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = 1;

  // basic calculation based on energy transfer from kinetic energy to potential energy (increase of speed leads to increase of sink).
  /// \todo Calculation of Rate of Sink: Refine the vario compensation by considering the decrease of drag based on the polar.
  transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_ACC_X) = lastStatus.trueAirSpeed/GRAVITY;


  //--STATUS_IND_VERTICAL_SPEED------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1;

  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_X) = timeDiff * rotMatrixPlaneToWorld(2,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_Y) = timeDiff * rotMatrixPlaneToWorld(2,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_Z) = timeDiff * rotMatrixPlaneToWorld(2,2);


  // Now some measured values which will not propagate time based changes, but which are only defined by the measurements
  //--STATUS_IND_ACC_X------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_X,GliderVarioStatus::STATUS_IND_ACC_X) = 1;

  //--STATUS_IND_ACC_Y------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Y,GliderVarioStatus::STATUS_IND_ACC_Y) = 1;

  //--STATUS_IND_ACC_Z------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Z,GliderVarioStatus::STATUS_IND_ACC_Z) = 1;

  //--STATUS_IND_ROTATION_X------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_X,GliderVarioStatus::STATUS_IND_ROTATION_X) = 1;

  //--STATUS_IND_ROTATION_Y------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Y,GliderVarioStatus::STATUS_IND_ROTATION_Y) = 1;

  //--STATUS_IND_ROTATION_Z------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1;

  //--STATUS_IND_GYRO_BIAS_X------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = 1;

  //--STATUS_IND_GYRO_BIAS_Y------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = 1;

  //--STATUS_IND_GYRO_BIAS_Z------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = 1;

  //--STATUS_IND_WIND_SPEED_N------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1;

  //--STATUS_IND_WIND_SPEED_E------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1;

  //--STATUS_IND_ACC_X------------------------------------------------------------------------------------
  transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = -1;
  transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1;

}

} // namespace openEV
