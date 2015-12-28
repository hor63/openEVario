/*
 * GliderVarioTransitionMatrix.cpp
 *
 *  Created on: Dec 8, 2015
 *      Author: openvario
 */

#include <GliderVarioTransitionMatrix.h>
#include "GliderVarioStatus.h"
#include "RotationMatrix.h"
#include "FastMath.h"

namespace openEV
{

GliderVarioTransitionMatrix::~GliderVarioTransitionMatrix ()
{
  // TODO Auto-generated destructor stub
}

void
GliderVarioTransitionMatrix::calcTransitionMatrix (
    FloatType timeDiff,
    GliderVarioStatus& lastStatus)
{
  /// I need the square of the time multiple times when calculating distance from acceleration
  FloatType timeDiffSquare = timeDiff * timeDiff;

  // I need a conversion from the plane coordinates into the world coordinates
  RotationMatrix rotMatrix (lastStatus.yawAngle,lastStatus.pitchAngle,lastStatus.rollAngle);
  RotationMatrix::RotationMatrixType &rotMatrixPlaneToWorld = rotMatrix.getMatrixPlaneToGlo();
  FloatType lenDegLongitude = 1852.0 * FastMath::fastCos(lastStatus.latitude);
  // I need half of time square for distance calculations based on acceleration here and there :)
  FloatType timeSquareHalf  = timeDiff*timeDiff / 2.0f;


  // OK, now systematically propagate the status based on previous status, and the elapsed time

  //--STATUS_IND_LONGITUDE------------------------------------------------------------------------------------
  // STATUS_IND_LONGITUDE
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_LONGITUDE) = 1;
  // STATUS_IND_LATITUDE
  // STATUS_IND_ALT_MSL
  // STATUS_IND_YAW
  // STATUS_IND_PITCH
  // STATUS_IND_ROLL
  // STATUS_IND_SPEED_GROUND
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND) = timeDiff * FastMath::fastSin(lastStatus.groundDirection);

  // STATUS_IND_TAS
  // STATUS_IND_HEADING
  // STATUS_IND_RATE_OF_CLIMB
  // STATUS_IND_VERTICAL_SPEED

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // STATUS_IND_ACC_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_X) = timeSquareHalf*rotMatrixPlaneToWorld(1,0);
  // STATUS_IND_ACC_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_Y) = timeSquareHalf*rotMatrixPlaneToWorld(1,1);
  // STATUS_IND_ACC_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_Z) = timeSquareHalf*rotMatrixPlaneToWorld(1,2);

  // Turn rates in reference to the body coordinate system

  // STATUS_IND_ROTATION_X
  // STATUS_IND_ROTATION_Y
  // STATUS_IND_ROTATION_Z

  /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter

  // STATUS_IND_GYRO_BIAS_X
  // STATUS_IND_GYRO_BIAS_Y
  // STATUS_IND_GYRO_BIAS_Z
  // STATUS_IND_WIND_SPEED
  // STATUS_IND_WIND_DIR
  // STATUS_IND_THERMAL_SPEED

  //--STATUS_IND_LATITUDE------------------------------------------------------------------------------------
  // STATUS_IND_LONGITUDE
  // STATUS_IND_LATITUDE
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_LATITUDE) = 1;
  // STATUS_IND_ALT_MSL
  // STATUS_IND_YAW
  // STATUS_IND_PITCH
  // STATUS_IND_ROLL
  // STATUS_IND_SPEED_GROUND
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND) = timeDiff * FastMath::fastSin(lastStatus.groundDirection);

  // STATUS_IND_TAS
  // STATUS_IND_HEADING
  // STATUS_IND_RATE_OF_CLIMB
  // STATUS_IND_VERTICAL_SPEED

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // STATUS_IND_ACC_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_X) = timeSquareHalf*rotMatrixPlaneToWorld(0,0);
  // STATUS_IND_ACC_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_Y) = timeSquareHalf*rotMatrixPlaneToWorld(0,1);
  // STATUS_IND_ACC_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_Z) = timeSquareHalf*rotMatrixPlaneToWorld(0,2);

  // Turn rates in reference to the body coordinate system

  // STATUS_IND_ROTATION_X
  // STATUS_IND_ROTATION_Y
  // STATUS_IND_ROTATION_Z

  /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter

  // STATUS_IND_GYRO_BIAS_X
  // STATUS_IND_GYRO_BIAS_Y
  // STATUS_IND_GYRO_BIAS_Z
  // STATUS_IND_WIND_SPEED
  // STATUS_IND_WIND_DIR
  // STATUS_IND_THERMAL_SPEED

  //--STATUS_IND_ALT_MSL------------------------------------------------------------------------------------
  // STATUS_IND_LONGITUDE
  // STATUS_IND_LATITUDE
  // STATUS_IND_ALT_MSL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL) = 1;
  // STATUS_IND_YAW
  // STATUS_IND_PITCH
  // STATUS_IND_ROLL
  // STATUS_IND_SPEED_GROUND

  // STATUS_IND_TAS
  // STATUS_IND_HEADING
  // STATUS_IND_RATE_OF_CLIMB
  // STATUS_IND_VERTICAL_SPEED
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = -timeDiff;

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // STATUS_IND_ACC_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_X) = timeSquareHalf*rotMatrixPlaneToWorld(2,0);
  // STATUS_IND_ACC_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_Y) = timeSquareHalf*rotMatrixPlaneToWorld(2,1);
  // STATUS_IND_ACC_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_Z) = timeSquareHalf*rotMatrixPlaneToWorld(2,2);

  // Turn rates in reference to the body coordinate system

  // STATUS_IND_ROTATION_X
  // STATUS_IND_ROTATION_Y
  // STATUS_IND_ROTATION_Z

  /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter

  // STATUS_IND_GYRO_BIAS_X
  // STATUS_IND_GYRO_BIAS_Y
  // STATUS_IND_GYRO_BIAS_Z
  // STATUS_IND_WIND_SPEED
  // STATUS_IND_WIND_DIR
  // STATUS_IND_THERMAL_SPEED

  //--STATUS_IND_YAW------------------------------------------------------------------------------------
  // STATUS_IND_LONGITUDE
  // STATUS_IND_LATITUDE
  // STATUS_IND_ALT_MSL
  // STATUS_IND_YAW
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_YAW) = 1;
  // STATUS_IND_PITCH
  // STATUS_IND_ROLL
  // STATUS_IND_SPEED_GROUND

  // STATUS_IND_TAS
  // STATUS_IND_HEADING
  // STATUS_IND_RATE_OF_CLIMB
  // STATUS_IND_VERTICAL_SPEED

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // STATUS_IND_ACC_X
  // STATUS_IND_ACC_Y
  // STATUS_IND_ACC_Z

  // Turn rates in reference to the body coordinate system

  // STATUS_IND_ROTATION_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff*rotMatrixPlaneToWorld(2,0);
  // STATUS_IND_ROTATION_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff*rotMatrixPlaneToWorld(2,1);
  // STATUS_IND_ROTATION_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_YAW,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff*rotMatrixPlaneToWorld(2,2);

  /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter

  // STATUS_IND_GYRO_BIAS_X
  // STATUS_IND_GYRO_BIAS_Y
  // STATUS_IND_GYRO_BIAS_Z
  // STATUS_IND_WIND_SPEED
  // STATUS_IND_WIND_DIR
  // STATUS_IND_THERMAL_SPEED

  //--STATUS_IND_PITCH------------------------------------------------------------------------------------
  // STATUS_IND_LONGITUDE
  // STATUS_IND_LATITUDE
  // STATUS_IND_ALT_MSL
  // STATUS_IND_YAW
  // STATUS_IND_PITCH
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_PITCH) = 1;
  // STATUS_IND_ROLL
  // STATUS_IND_SPEED_GROUND

  // STATUS_IND_TAS
  // STATUS_IND_HEADING
  // STATUS_IND_RATE_OF_CLIMB
  // STATUS_IND_VERTICAL_SPEED

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // STATUS_IND_ACC_X
  // STATUS_IND_ACC_Y
  // STATUS_IND_ACC_Z

  // Turn rates in reference to the body coordinate system

  // STATUS_IND_ROTATION_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff*rotMatrixPlaneToWorld(1,0);
  // STATUS_IND_ROTATION_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff*rotMatrixPlaneToWorld(1,1);
  // STATUS_IND_ROTATION_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff*rotMatrixPlaneToWorld(1,2);

  /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter

  // STATUS_IND_GYRO_BIAS_X
  // STATUS_IND_GYRO_BIAS_Y
  // STATUS_IND_GYRO_BIAS_Z
  // STATUS_IND_WIND_SPEED
  // STATUS_IND_WIND_DIR
  // STATUS_IND_THERMAL_SPEED

  //--STATUS_IND_ROLL------------------------------------------------------------------------------------
  // STATUS_IND_LONGITUDE
  // STATUS_IND_LATITUDE
  // STATUS_IND_ALT_MSL
  // STATUS_IND_YAW
  // STATUS_IND_PITCH
  // STATUS_IND_ROLL
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROLL) = 1;
  // STATUS_IND_SPEED_GROUND

  // STATUS_IND_TAS
  // STATUS_IND_HEADING
  // STATUS_IND_RATE_OF_CLIMB
  // STATUS_IND_VERTICAL_SPEED

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // STATUS_IND_ACC_X
  // STATUS_IND_ACC_Y
  // STATUS_IND_ACC_Z

  // Turn rates in reference to the body coordinate system

  // STATUS_IND_ROTATION_X
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_X) = timeDiff*rotMatrixPlaneToWorld(0,0);
  // STATUS_IND_ROTATION_Y
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_Y) = timeDiff*rotMatrixPlaneToWorld(0,1);
  // STATUS_IND_ROTATION_Z
  transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_Z) = timeDiff*rotMatrixPlaneToWorld(0,2);

  /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter

  // STATUS_IND_GYRO_BIAS_X
  // STATUS_IND_GYRO_BIAS_Y
  // STATUS_IND_GYRO_BIAS_Z
  // STATUS_IND_WIND_SPEED
  // STATUS_IND_WIND_DIR
  // STATUS_IND_THERMAL_SPEED

}

} // namespace openEV
