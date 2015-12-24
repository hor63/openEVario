/*
 * GliderVarioTransitionMatrix.cpp
 *
 *  Created on: Dec 8, 2015
 *      Author: openvario
 */

#include <GliderVarioTransitionMatrix.h>
#include "GliderVarioStatus.h"

namespace openEV
{

GliderVarioTransitionMatrix::~GliderVarioTransitionMatrix ()
{
  // TODO Auto-generated destructor stub
}

void
GliderVarioTransitionMatrix::calcTransitionMatrix (
    FloatType timeDiff)
{
  /// I need the square of the time multiple times when calculating distance from acceleration
  FloatType timeDiffSquare = timeDiff * timeDiff;

  // I need a conversion from the plane coordinates into the world coordinates

  /*
  STATUS_IND_LONGITUDE	,  ///< Longitude in deg. East
  STATUS_IND_LATITUDE  	,  ///< Latitude in deg North
  STATUS_IND_ALT_MSL   	,  ///< Altitude in m over Mean Sea Level
  STATUS_IND_YAW 		,  ///< Yaw angle in deg. right turn from true North
  STATUS_IND_PITCH		,  ///< Pitch angle in deg. nose up. Pitch is applied after yaw.
  STATUS_IND_ROLL		,  ///< Roll angle in deg. right. Roll is applied after yaw and pitch.

  /// Speeds and directions

  STATUS_IND_SPEED_GROUND	,  ///< Ground speed in m/s
  STATUS_IND_DIR_GROUND	,  ///< Direction over ground in deg. right turn from true North
  STATUS_IND_TAS		,  ///< True air speed in m/s relative to surrounding air.
  STATUS_IND_HEADING		,  ///< Heading of the plane in deg. right turn from true north. This is the flight direction relative to the surrounding air.
  STATUS_IND_RATE_OF_CLIMB	, ///< Rate of climb in m/s relative to the surrounding air
  STATUS_IND_VERTICAL_SPEED	, ///< Absolute vertical speed in m/s

  /// Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  /// If the plane is pitched up an acceleation on the X axis would speed the plane upward, not forward.

  STATUS_IND_ACC_X		, ///< Acceleration in m/s^2 on the X axis of the plane
  STATUS_IND_ACC_Y		, ///< Acceleration in m/s^2 on the Y axis of the plane
  STATUS_IND_ACC_Z		, ///< Acceleration in m/s^2 on the Z axis of the plane

  /// Turn rates in reference to the body coordinate system

  STATUS_IND_ROTATION_X	, ///< Roll rate in deg/s to the right around the X axis
  STATUS_IND_ROTATION_Y	, ///< Pitch rate in deg/s nose up around the Y axis
  STATUS_IND_ROTATION_Z	, ///< Yaw (turn) rate in deg/s around the Z axis

  /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter

  STATUS_IND_GYRO_BIAS_X	, ///< Bias (0-offset) of the X axis gyro in eg/s
  STATUS_IND_GYRO_BIAS_Y	, ///< Bias (0-offset) of the Y axis gyro in deg/s
  STATUS_IND_GYRO_BIAS_Z	, ///< Bias (0-offset) of the Z axis gyro in deg/s
  STATUS_IND_WIND_SPEED	, ///< Wind speed in m/s
  STATUS_IND_WIND_DIR		, ///< Wind direction in deg. right turn from North.
				      ///< The direction is the direction *from where* the wind blows.
  STATUS_IND_THERMAL_SPEED	, ///< The true reason for the whole exercise! :)
  NUM_ROWS				///< The number of rows in the vector
*/

}

} // namespace openEV
