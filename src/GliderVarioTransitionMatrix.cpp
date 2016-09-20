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
GliderVarioTransitionMatrix::calcTransitionMatrix (
    FloatType timeDiff,
    GliderVarioStatus const &lastStatus)
{
  // I need the square of the time multiple times when calculating distance from acceleration
  FloatType timeDiffSquare = timeDiff * timeDiff;

  // I need a conversion from the plane coordinates into the world coordinates
  RotationMatrix rotMatrix (lastStatus.heading,lastStatus.pitchAngle,lastStatus.rollAngle);
  RotationMatrix3DType &rotMatrixPlaneToWorld = rotMatrix.getMatrixPlaneToGlo();
  RotationMatrix3DType &rotMatrixWorldToPlane = rotMatrix.getMatrixGloToPlane();

  // I need half of time square for distance calculations based on acceleration here and there :)
  FloatType timeSquareHalf  = timeDiff*timeDiff / 2.0f;
  FloatType const lenLongitudeArcSec = lenLatitudeArcSec * FastMath::fastCos(lastStatus.latitude);


  // OK, now systematically propagate the status based on previous status, and the elapsed time
  // Constant factors in comments have been moved to the class constructor. They will not change, and have to be set only once.

  // STATUS_IND_CONST_ONE
  transitionMatrix(GliderVarioStatus::STATUS_IND_CONST_ONE,GliderVarioStatus::STATUS_IND_CONST_ONE) = 1.0f;

  // STATUS_IND_LATITUDE
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_LATITUDE) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = timeDiff / lenLatitudeArcSec;
  FloatType timeSquareHalf2Lat = timeSquareHalf / lenLatitudeArcSec;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_X) = timeSquareHalf2Lat * rotMatrixPlaneToWorld(0,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_Y) = timeSquareHalf2Lat * rotMatrixPlaneToWorld(0,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_Z) = timeSquareHalf2Lat * rotMatrixPlaneToWorld(0,2);

  // STATUS_IND_LONGITUDE
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_LONGITUDE) = 1;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_E) = timeDiff / lenLongitudeArcSec ;
  FloatType timeSquareHalf2Lon = timeSquareHalf / lenLongitudeArcSec;
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_X) = timeSquareHalf2Lon * rotMatrixPlaneToWorld(1,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_Y) = timeSquareHalf2Lon * rotMatrixPlaneToWorld(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_Z) = timeSquareHalf2Lon * rotMatrixPlaneToWorld(1,2);

  // STATUS_IND_ALT_MSL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = -timeDiff;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_X) = -timeSquareHalf* (rotMatrixPlaneToWorld(2,0));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_Y) = -timeSquareHalf* (rotMatrixPlaneToWorld(2,1));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_Z) = -timeSquareHalf* (rotMatrixPlaneToWorld(2,2));
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_CONST_ONE) = -timeSquareHalf*GRAVITY;

  // STATUS_IND_HEADING
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_HEADING) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z) = timeDiff;

  // STATUS_IND_PITCH
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_PITCH) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff*rotMatrixPlaneToWorld(1,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff*rotMatrixPlaneToWorld(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff*rotMatrixPlaneToWorld(1,2);

  // STATUS_IND_ROLL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROLL) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff*rotMatrixPlaneToWorld(0,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff*rotMatrixPlaneToWorld(0,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff*rotMatrixPlaneToWorld(0,2);


  // STATUS_IND_SPEED_GROUND_N
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_X) = timeDiff * rotMatrixPlaneToWorld(0,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_Y) = timeDiff * rotMatrixPlaneToWorld(0,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_Z) = timeDiff * rotMatrixPlaneToWorld(0,2);

  // STATUS_IND_SPEED_GROUND_E
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_SPEED_GROUND_E) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_X) = timeDiff * rotMatrixPlaneToWorld(1,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_Y) = timeDiff * rotMatrixPlaneToWorld(1,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_Z) = timeDiff * rotMatrixPlaneToWorld(1,2);

  // STATUS_IND_TAS_N
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS_N,GliderVarioStatus::STATUS_IND_TAS) = FastMath::fastCos(lastStatus.heading);

  // STATUS_IND_TAS_E
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS_E,GliderVarioStatus::STATUS_IND_TAS) = FastMath::fastSin(lastStatus.heading);

  // STATUS_IND_TAS
  /// \todo Link TAS and ground speed with wind
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_TAS) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ACC_X) = timeDiff*FastMath::fastCos(lastStatus.pitchAngle);

  /*
   * STATUS_IND_RATE_OF_SINK
   * The calculation is based on the energy transfer from kinetic energy to potential energy (increase of speed leads to increase of sink).
   * The acceleration is measured along the body X axis. If pitched up or down the accelerometer reading is affected by gravity.
   * The actual equation is something like: (accX - sin(pitch)*GRAVITY) * TAS / GRAVITY = (accX*TAS/GRAVITY) - sin(pitch)*TAS*GRAVITY/GRAVITY
   * So IMHO this is a pretty crude approximation because I assume that my TAS is exactly along the X axis which is rarely accurate due to
   * changing angles of attack.
   */
  /// \todo Calculation of Rate of Sink: Refine the vario compensation by considering the decrease of drag based on the polar.
  transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_ACC_X) = lastStatus.trueAirSpeed/GRAVITY;
  transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_TAS) = -FastMath::fastSin(lastStatus.pitchAngle);

  // STATUS_IND_VERTICAL_SPEED
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1.0f;

  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_X) = timeDiff * rotMatrixPlaneToWorld(2,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_Y) = timeDiff * rotMatrixPlaneToWorld(2,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_Z) = timeDiff * rotMatrixPlaneToWorld(2,2);

  transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_CONST_ONE) = timeDiff * GRAVITY;

  // STATUS_IND_THERMAL_SPEED
  transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1.0f;
  transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = -1.0f;

  // STATUS_IND_ACC_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_X,GliderVarioStatus::STATUS_IND_ACC_X) = 1.0f;
  // STATUS_IND_ACC_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Y,GliderVarioStatus::STATUS_IND_ACC_Y) = 1.0f;
    // STATUS_IND_ACC_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Z,GliderVarioStatus::STATUS_IND_ACC_Z) = 1.0f;

  // STATUS_IND_ROTATION_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_X,GliderVarioStatus::STATUS_IND_ROTATION_X) = 1.0f;
  // STATUS_IND_ROTATION_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Y,GliderVarioStatus::STATUS_IND_ROTATION_Y) = 1.0f;
  // STATUS_IND_ROTATION_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1.0f;
  // STATUS_IND_ROTATION_GLO_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_X) = rotMatrixPlaneToWorld(2,0);
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_Y) = rotMatrixPlaneToWorld(2,1);
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_GLO_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = rotMatrixPlaneToWorld(2,2);

  // STATUS_IND_GYRO_BIAS_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = 1.0f;

  // STATUS_IND_GYRO_BIAS_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = 1.0f;

  // STATUS_IND_GYRO_BIAS_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = 1.0f;

  // STATUS_IND_COMPASS_ERROR
  transitionMatrix(GliderVarioStatus::STATUS_IND_COMPASS_ERROR,GliderVarioStatus::STATUS_IND_COMPASS_ERROR) = 1.0f;

  // STATUS_IND_WIND_SPEED_N
  // STATUS_IND_WIND_SPEED_E
  // STATUS_IND_WIND_SPEED
  // STATUS_IND_WIND_DIR










  //--STATUS_IND_ACC_X------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_X,GliderVarioStatus::STATUS_IND_ACC_X) = 1;

  //--STATUS_IND_ACC_Y------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Y,GliderVarioStatus::STATUS_IND_ACC_Y) = 1;

  //--STATUS_IND_ACC_Z------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Z,GliderVarioStatus::STATUS_IND_ACC_Z) = 1;

  //--STATUS_IND_ROTATION_X------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_X,GliderVarioStatus::STATUS_IND_ROTATION_X) = 1;

  //--STATUS_IND_ROTATION_Y------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Y,GliderVarioStatus::STATUS_IND_ROTATION_Y) = 1;

  //--STATUS_IND_ROTATION_Z------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1;

  //--STATUS_IND_GYRO_BIAS_X------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = 1;

  //--STATUS_IND_GYRO_BIAS_Y------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = 1;

  //--STATUS_IND_GYRO_BIAS_Z------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = 1;

  //--STATUS_IND_WIND_SPEED_N------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1;

  //--STATUS_IND_WIND_SPEED_E------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1;

  //--STATUS_IND_THERMAL_SPEED------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = -1;
  // transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1;

  //--STATUS_IND_STATUS_IND_GRAVITY------------------------------------------------------------------------------------
  // transitionMatrix(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 1;



}

} // namespace openEV
