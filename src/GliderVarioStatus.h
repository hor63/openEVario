/*
 * GliderVarioStatus.h
 * File GliderVarioStatus.h
 *
 *  Created on: Dec 8, 2015
 *      Author: hor
 *
 * Defines the class GliderVarioStatus
 *
 */


#ifndef GLIDERVARIOSTATUS_H_
#define GLIDERVARIOSTATUS_H_

#include <Dense>

namespace openEV
{

/**
 * The global float type. Change this one to double, and the entire system will run in double.
 * For optimal performance this should be *float*. Eigen can use the NEON unit for vectorized arithmetic.
 */
typedef float FloatType;

/**
 * This vector type is used for all 3-dimensional representations of values in Kartesian coodinates
 */
typedef Eigen::Matrix<FloatType, 3, 1> Vector3DType;

/**
 *  \class GliderVarioStatus
 *
 * \brief GliderVarioStatus manages the Kalman filter state x.
 *
 * The class defines the Kalman filter status x as a vector of floats or doubles. Each component of the
 * status vector is clearly identified by the index in the vector.
 * The indexes are enumerated in the *StatusComponentIndex* enum.
 * The components and index enumerators of the status vector are as follows:
 *
 * Worldwide Position:
 * - Longitude STATUS_IND_LONGITUDE:		\b Longitude in decimal degrees. Eastern hemisphere is positive,
 *  							western hemisphere is negative.
 * - Latitude STATUS_IND_LATITUDE:			\b Latitude in decimal degrees. Nothern hemisphere is positive,
 * 							southern hemisphere is negative.
 * - Altitude MSL STATUS_IND_ALT_MSL:		\b Altitude above MSL in m(eter).
 *
 * Attitude:
 * - Yaw angle STATUS_IND_YAW:			\b Yaw angle in Degrees to the right of true North. Also known as \b Heading
 * - Pitch angle STATUS_IND_PITCH:			\b Pitch angle in Degrees nose upward. 0 = horizontal flight. Also known as \b Elevation.
 * - Roll angle STATUS_IND_ROLL:			\b Roll angle in degrees right. Left roll is negative. Also known as \b Bank.
 *
 * Speeds and directions
 * - Ground speed STATUS_IND_SPEED_GROUND		<b>Ground Speed</b> in m/s
 * - Direction over ground STATUS_IND_DIR_GROUND	<b>Flight Direction over ground</b> in Degrees to the right to true North.
 * - True air speed STATUS_IND_TAS			<b>True Air Speed</b> in m/s. Speed relative to the surrounding air
 * - Plane heading STATUS_IND_HEADING		<b>True Heading of the plane</b>.
 * 							I assume that the heading is equal to my movement vector in the air,
 * 							i.e. I assume that I am not slipping.
 * - Plane rate of Climb STATUS_IND_RATE_OF_CLIMB	<b>Rate of Climb</b> of the air plane relative to the air in m/s.
 * 							Up is positive. This is kind of my stick thermals.
 * 							STATUS_IND_VERTICAL_SPEED and Rate of climb are identical in stagnant air.
 * - Absolute vertical speed STATUS_IND_VERTICAL_SPEED	<b>Absolute vertical speed</b> in m/s
 *
 * Accelerations in reference to the body coordinate system
 * - Accel X axis STATUS_IND_ACC_X			<b>Acceleration along X axis</b> in m/s^2
 * - Accel Y axis STATUS_IND_ACC_Y			<b>Acceleration along Y axis</b> in m/s^2
 * - Accel Z axis STATUS_IND_ACC_Z			<b>Acceleration along Y axis</b> in m/s^2
 *
 * Turn rates in reference to the body coordinate system
 * - Rotation around X axis 			<b>Rotation around X axis</b> in degrees per second
 * - Rotation around Y axis 			<b>Rotation around Y axis</b> in degrees per second
 * - Rotation around Z axis 			<b>Rotation around Z axis</b> in degrees per second
 *
 * Derived values which improve the responsiveness of the Kalman filter
 * - Gyro X bias STATUS_IND_GYRO_BIAS_X		<b>Gyro X axis bias</b>
 * 							Gyros tend to have a bias, i.e an offset of the 0-value.
 * 							The bias is not constant but varies over time. Tracking it helps to make the filter more responsive
 * - Gyro Y bias STATUS_IND_GYRO_BIAS_Y		<b>Gyro Y axis bias</b>
 * - Gyro Z bias STATUS_IND_GYRO_BIAS_Z		<b>Gyro Z axis bias</b>
 * - Wind speed STATUS_IND_WIND_SPEED		<b>Wind Speed</b> in m/s
 * - Wind direction STATUS_IND_WIND_DIR		<b>Wind Direction</b> in Degrees, STATUS_IND_DIR_GROUND
 * - Thermal speed STATUS_IND_THERMAL_SPEED	The real thermal updraft in m/s
 *
 */
class GliderVarioStatus
{
public:

  /** \brief Index, i.e. positions of the status components in the status vector
   *
   * Enumeration of the components of the Kalman status vector x
   */
  enum StatusComponentIndex {
    /// Position and attitude

    STATUS_IND_LONGITUDE	,  ///< Longitude in deg. East
    STATUS_IND_LATITUDE  	,  ///< Latitude in deg North
    STATUS_IND_ALT_MSL   	,  ///< Altitude in m over Mean Sea Level
    STATUS_IND_YAW 		,  ///< Yaw angle in deg. right turn from true North
    STATUS_IND_PITCH		,  ///< Pitch angle in deg. nose up. Pitch is applied after yaw.
    STATUS_IND_ROLL		,  ///< Roll angle in deg. right. Roll is applied after yaw and pitch.

    /// Speeds and directions

    STATUS_IND_SPEED_GROUND_N	,  ///< Ground speed component North in m/s
    STATUS_IND_SPEED_GROUND_E	,  ///< Ground speed component East in m/s
    STATUS_IND_TAS		,  ///< True air speed in m/s relative to surrounding air.
    STATUS_IND_HEADING		,  ///< Heading of the plane in deg. right turn from true north. This is the flight direction relative to the surrounding air.
    STATUS_IND_RATE_OF_SINK	, ///< Rate of sink in m/s relative to the surrounding air. Sink because the y axis points downward
    STATUS_IND_VERTICAL_SPEED	, ///< Absolute vertical speed in m/s downward. Z axis is direction down.

    /// Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
    /// If the plane is pitched up an acceleration on the X axis would speed the plane upward, not forward.

    STATUS_IND_ACC_X		, ///< Acceleration in m/s^2 on the X axis of the plane
    STATUS_IND_ACC_Y		, ///< Acceleration in m/s^2 on the Y axis of the plane
    STATUS_IND_ACC_Z		, ///< Acceleration in m/s^2 on the Z axis of the plane

    /// Turn rates in reference to the body coordinate system

    STATUS_IND_ROTATION_X	, ///< Roll rate in deg/s to the right around the X axis
    STATUS_IND_ROTATION_Y	, ///< Pitch rate in deg/s nose up around the Y axis
    STATUS_IND_ROTATION_Z	, ///< Yaw (turn) rate in deg/s around the Z axis

    /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter

    STATUS_IND_GYRO_BIAS_X	, ///< Bias (0-offset) of the X axis gyro in deg/s
    STATUS_IND_GYRO_BIAS_Y	, ///< Bias (0-offset) of the Y axis gyro in deg/s
    STATUS_IND_GYRO_BIAS_Z	, ///< Bias (0-offset) of the Z axis gyro in deg/s
    STATUS_IND_WIND_SPEED_N	, ///< Wind speed North component in m/s
    STATUS_IND_WIND_SPEED_E	, ///< Wind speed East component in m/s
				      ///< The direction is the direction *from where* the wind blows.
    STATUS_IND_THERMAL_SPEED	, ///< The true reason for the whole exercise! :)
    NUM_ROWS				///< The number of rows in the vector
  };

  typedef Eigen::Matrix<FloatType,NUM_ROWS,1> StatusVectorType; ///< Saves typing of the complex template type

  GliderVarioStatus ();
  virtual
  ~GliderVarioStatus ();

  StatusVectorType& getStatusVector() {
    return statusVector;
  }

  // Here come all state vector elements as single references into the vector for easier access

  // Position and attitude
  FloatType& longitude = statusVector[ STATUS_IND_LONGITUDE	];  ///< Longitude in deg. East
  FloatType& latitude = statusVector[ STATUS_IND_LATITUDE  	];  ///< Latitude in deg North
  FloatType& altMSL = statusVector[ STATUS_IND_ALT_MSL   	];  ///< Altitude in m over Mean Sea Level
  FloatType& yawAngle = statusVector[ STATUS_IND_YAW 		];  ///< Yaw angle in deg. right turn from true North
  FloatType& pitchAngle = statusVector[ STATUS_IND_PITCH		];  ///< Pitch angle in deg. nose up. Pitch is applied after yaw.
  FloatType& rollAngle = statusVector[ STATUS_IND_ROLL		];  ///< Roll angle in deg. right. Roll is applied after yaw and pitch.

  // Speeds and directions
  FloatType& groundSpeedNorth = statusVector[ STATUS_IND_SPEED_GROUND_N	];  ///< Ground speed component North in m/s
  FloatType& groundSpeedEast = statusVector[ STATUS_IND_SPEED_GROUND_E	];  ///< Ground speed component East in m/s
  FloatType& trueAirSpeed = statusVector[ STATUS_IND_TAS		];  ///< True air speed in m/s relative to surrounding air.
  FloatType& heading = statusVector[ STATUS_IND_HEADING		];  ///< Heading of the plane in deg. right turn from true north. This is the flight direction relative to the surrounding air.
  FloatType& rateOfSink = statusVector[ STATUS_IND_RATE_OF_SINK	]; ///< Rate of sink in m/s relative to the surrounding air. Sink because the Z axis points downward.
  FloatType& verticalSpeed = statusVector[ STATUS_IND_VERTICAL_SPEED	]; ///< Absolute vertical speed in m/s downward. Z axis is downward.

  // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
  // If the plane is pitched up an acceleation on the X axis would speed the plane upward, not forward.
  FloatType& accelX = statusVector[ STATUS_IND_ACC_X		]; ///< Acceleration in m/s^2 on the X axis of the plane
  FloatType& accelY = statusVector[ STATUS_IND_ACC_Y		]; ///< Acceleration in m/s^2 on the Y axis of the plane
  FloatType& accelZ = statusVector[ STATUS_IND_ACC_Z		]; ///< Acceleration in m/s^2 on the Z axis of the plane

  // Turn rates in reference to the body coordinate system
  FloatType& rollRateX = statusVector[ STATUS_IND_ROTATION_X	]; ///< Roll rate in deg/s to the right around the X axis
  FloatType& pitchRateY = statusVector[ STATUS_IND_ROTATION_Y	]; ///< Pitch rate in deg/s nose up around the Y axis
  FloatType& yawRateZ = statusVector[ STATUS_IND_ROTATION_Z	]; ///< Yaw (turn) rate in deg/s around the Z axis

  // Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter
  FloatType& gyroBiasX = statusVector[ STATUS_IND_GYRO_BIAS_X	]; ///< Bias (0-offset) of the X axis gyro in deg/s
  FloatType& gyroBiasY = statusVector[ STATUS_IND_GYRO_BIAS_Y	]; ///< Bias (0-offset) of the Y axis gyro in deg/s
  FloatType& gyroBiasZ = statusVector[ STATUS_IND_GYRO_BIAS_Z	]; ///< Bias (0-offset) of the Z axis gyro in deg/s
  FloatType& windSpeedNorth = statusVector[ STATUS_IND_WIND_SPEED_N	]; ///< Wind speed North component in m/s
  FloatType& windSpeedEast = statusVector[ STATUS_IND_WIND_SPEED_E		]; ///< Wind speed East component in m/s
				      ///< The direction is the direction *from where* the wind blows.
  FloatType& thermalSpeed = statusVector[ STATUS_IND_THERMAL_SPEED	]; ///< The true reason for the whole exercise! :)

protected:
  StatusVectorType statusVector;


};

} // namespace openEV

std::ostream& operator << (std::ostream& o ,openEV::GliderVarioStatus& s);

#endif /* GLIDERVARIOSTATUS_H_ */
