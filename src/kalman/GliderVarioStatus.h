/*
 * GliderVarioStatus.h
 *
 *  Created on: Dec 8, 2015
 *      Author: hor
 *
 * Defines the class GliderVarioStatus
 *
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

#ifndef GLIDERVARIOSTATUS_H_
#define GLIDERVARIOSTATUS_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "CommonDefs.h"
#include "util/OEV_Enum.h"
#include "util/NaturalConstants.h"

/**
 * This namespace includes a complex Extended Kalman Filter (EKF) for implementing a complete electronic compensated variometer, and artificial horizon.
 *
 * Input to this model are inertial measurements from 3-D accelerometer and gyroscopes, a barometric precision altimeter,
 * IAS and TAS measurement from dynamic pressure and GPS coordinates, heading and speed over ground
 *
 * Invaluable inputs for understanding Kalman filters in general, and practical implementation hints came from
 * [Groves - Kalman Filter-Based Estimation](http://www.artechhouse.com/static/sample/groves-005_ch03.pdf),
 * page 107, 3.2.7  Sequential Measurement Update (link is defunct \emoji :unamused:)
 * \sa [Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, Second Edition from Artechhouse](https://us.artechhouse.com/Principles-of-GNSS-Inertial-and-Multisensor-Integrated-Navigation-Systems-Second-Edition-P2046.aspx)
 *
 * For an EKF I need a Jacobian matrix with the partial derivates at the point of the last status to approximate a linearization of
 * the non-linear function at the latest status. To make my life easier I am using a numeric forward derivation with a small step.
 * For numeric derivation of angles I use an increment of 1 degree. This is small enough for an approximation of sin and cos, and large enough
 * not to create undue numeric issues with scale factors in single precision floats. It has the additional advantage that the increment is
 * 1.0 (degrees), i.e. I am saving myself a division by the increment if it were != 1.
 * \sa [Numerical differentiations of Jacobian matrixes](http://www.iue.tuwien.ac.at/phd/khalil/node14.html)
 *
 */
namespace openEV
{


/**
 *  \class GliderVarioStatus
 *
 * \brief GliderVarioStatus manages the Kalman filter state x as #statusVector_x
 * and the accompanying process error covariance matrix P as #errorCovariance_P,
 * and the system noise covariance matrix Q as #systemNoiseCovariance_Q
 *
 * The class defines the Kalman filter status #statusVector_x as a vector of floats or doubles.
 * Each component of the status vector is clearly identified by the index in the vector.
 * The indexes are enumerated in the #StatusComponentIndex enum.
 * Please see the enumerations for descriptions of the components.
 * The system noise covariance #systemNoiseCovariance_Q which describes the state uncertainties is defined here too
 * as well as the process error covariance #errorCovariance_P.
 *
 */
class OEV_PUBLIC GliderVarioStatus
{

	friend class GliderVarioTransitionMatrix;

public:

    /** \brief Index, i.e. positions of the status components in #statusVector_x
     *
     * Enumeration of the components of the Kalman status vector #statusVector_x
     */
	OEV_ENUM(StatusComponentIndex,

		// constants (but may varying slightly
        STATUS_IND_GRAVITY      ,  ///< The gravity, initialized to #::GRAVITY

        // Position and altitude
        STATUS_IND_LATITUDE_OFFS	,  ///< Latitude offset in meter North of GliderVarioStatus::latitudeBaseArcSec
        STATUS_IND_LONGITUDE_OFFS	,  ///< Longitude offset in meter East of GliderVarioStatus::latitudeBaseArcSec
        STATUS_IND_ALT_MSL   		,  ///< Altitude in m over Mean Sea Level

        // Attitude of the body to the world coordinate system
        STATUS_IND_HEADING			,  ///< Heading of the plane in deg. right turn from true north. This is the flight direction relative to the surrounding air.
        								///< Synonymous with Yaw.
        STATUS_IND_PITCH			,  ///< Pitch angle in deg. nose up. Pitch is applied after yaw.
        STATUS_IND_ROLL		    	,  ///< Roll angle in deg. right. Roll is applied after yaw and pitch.

        // Speeds
        STATUS_IND_SPEED_GROUND_N	,  ///< Ground speed component North in m/s
        STATUS_IND_SPEED_GROUND_E	,  ///< Ground speed component East in m/s
        STATUS_IND_TAS				,  ///< True air speed horizontally in direction of heading
        STATUS_IND_RATE_OF_SINK		, ///< Rate of sink in m/s relative to the surrounding air. Sink because the Z axis points downward
        STATUS_IND_VERTICAL_SPEED	, ///< Absolute vertical speed in m/s downward. Z axis is direction down.
        STATUS_IND_THERMAL_SPEED	, ///< The true reason for the whole exercise! :). As always in Z axis direction downward.

        // Accelerations in direction of the heading, vertical and perpendicular to heading.
        STATUS_IND_ACC_HEADING		, ///< Acceleration in m/s&sup2; horizontally along the heading of the plane
        STATUS_IND_ACC_CROSS		, ///< Acceleration in m/s&sup2; horizontally perpendicular to the heading
        								///< Note that this does *not* include centrifugal force!!!
        								///< This is only residual acceleration not accounted by turning.
        STATUS_IND_ACC_VERTICAL		, ///< Acceleration in m/s&sup2; along body Z axis

        // Turn rates in reference to the world coordinate system
        STATUS_IND_ROTATION_X	, ///< Roll rate in deg/s to the right around the X axis
        STATUS_IND_ROTATION_Y	, ///< Pitch rate in deg/s nose up around the Y axis
        STATUS_IND_ROTATION_Z	, ///< Yaw (turn) rate clockwise in deg/s around the Z (downward) axis

        // Derived values which improve the responsiveness of the Kalman filter.
        STATUS_IND_GYRO_BIAS_X	, ///< Bias (0-offset) of the plane X axis gyro in deg/s
        STATUS_IND_GYRO_BIAS_Y	, ///< Bias (0-offset) of the plane Y axis gyro in deg/s
        STATUS_IND_GYRO_BIAS_Z	, ///< Bias (0-offset) of the plane Z axis gyro in deg/s
        STATUS_IND_MAGNETIC_DECLINATION, ///< Magnetic declination (variation) in degrees.
        STATUS_IND_MAGNETIC_INCLINATION, ///< Inclination of the magnetic vector in degrees in pitch direction (upward), i.e. negative on the northern hemisphere
        STATUS_IND_COMPASS_DEVIATION_X, ///< Strength of the local airplane magnetic field in X direction
        STATUS_IND_COMPASS_DEVIATION_Y, ///< Strength of the local airplane magnetic field in Y direction
        STATUS_IND_COMPASS_DEVIATION_Z, ///< Strength of the local airplane magnetic field in Z direction
        STATUS_IND_WIND_SPEED_N	, ///< Wind speed North component in m/s
        STATUS_IND_WIND_SPEED_E	, ///< Wind speed East component in m/s
        STATUS_IND_QFF			, ///< QFF in hPascal (Using a more realistic model incl. temperature, but ignoring humidity).
        STATUS_IND_LAST_PRESSURE, ///< Calculated pressure from last altMSL. Used to avoid to calculate the expensive
        								///< barometric formula multiple times. I am ignoring the error imposed by the
        								///< last altitude update :)

        STATUS_NUM_ROWS				///< The number of rows in the vector. This is not a component of the vector!
	);

    typedef Eigen::Matrix<FloatType,STATUS_NUM_ROWS,1> StatusVectorType; ///< Saves typing of the complex template type
    typedef Eigen::SparseMatrix<FloatType> StatusCoVarianceType; ///< Co-variance matrix type for P and Q

protected:
        StatusVectorType     statusVector_x;

public:
    GliderVarioStatus ();
    virtual
    ~GliderVarioStatus ();

    /**
     *
     * @return reference to the internal vector for direct matrix manipulation or matrix arithmetics
     */
    StatusVectorType& getStatusVector_x() {
        return statusVector_x;
    }

    /**
     *
     * @return reference to the internal vector for direct matrix arithmetics
     */
    StatusVectorType const &getStatusVector_x() const {
        return statusVector_x;
    }

    /**
     *
     * @return reference to the system noise covariance Q for direct matrix manipulation or matrix arithmetics
     */
    StatusCoVarianceType &getSystemNoiseCovariance_Q() {
        return systemNoiseCovariance_Q;
    }

    /**
     *
     * @return reference to the system noise covariance Q for direct matrix arithmetics
     */
    StatusCoVarianceType const &getSystemNoiseCovariance_Q() const {
        return systemNoiseCovariance_Q;
    }

    /**
     *
     * @return reference to the process error covariance Q for direct matrix manipulation or matrix arithmetics
     */
    StatusCoVarianceType &getErrorCovariance_P() {
        return errorCovariance_P;
    }

    /**
     *
     * @return reference to the process error covariance Q for direct matrix arithmetics
     */
    StatusCoVarianceType const &getErrorCovariance_P() const {
        return errorCovariance_P;
    }

    /**
     * Updating the status may lead to wrap-around of angles. Here are the limits:
     * -Pitch: 90 <= pitch <= 90; If you fly a looping and turn past perpendicular you essentially roll 180 deg, and reverse direction 180 deg
     * -Roll: -180 <= roll < 180; 180 deg counts as -180
     * -Yaw: 0<= yaw < 360; 360 deg counts as 0.
     * Note that pitch must be normalized fist. It may flip roll and yaw around. Yaw and roll are independent from the other angles.
     *
     * Updating the status may lead to an overflow of GliderVarioStatus::longitudeOffs and GliderVarioStatus::latitudeOffs offset beyond one arc second.
     * When that happens GliderVarioStatus::latitudeBaseArcSec and/or GliderVarioStatus::longitudeBaseArcSec are updated until the offsets are < 1sec.
     *
     */
    void normalizeStatus();

    // Here come all state vector elements as single references into the vector for easier access

    /// Constants
    FloatType& gravity  = statusVector_x [ STATUS_IND_GRAVITY ];  ///< The gravity, initialized to #::GRAVITY

    /// Position and altitude
    const FloatType& longitudeOffsC = statusVector_x[ STATUS_IND_LONGITUDE_OFFS	];  ///< Latitude offset in meter North of GliderVarioStatus::latitudeBaseArcSec
    const FloatType& latitudeOffsC	 = statusVector_x[ STATUS_IND_LATITUDE_OFFS  	];  ///< Longitude offset in meter East of GliderVarioStatus::latitudeBaseArcSec
    FloatType& altMSL = statusVector_x[ STATUS_IND_ALT_MSL   	    ];  ///< Altitude in m over Mean Sea Level

    /// Attitude of the body to the world coordinate system
    FloatType& heading = statusVector_x[ STATUS_IND_HEADING		];  ///< Heading of the plane in deg. right turn from true north. This is the flight direction relative to the surrounding air.
    ///< Synonymous with Yaw.
    FloatType& pitchAngle = statusVector_x[ STATUS_IND_PITCH		];  ///< Pitch angle in deg. nose up. Pitch is applied after yaw.
    FloatType& rollAngle = statusVector_x[ STATUS_IND_ROLL		];  ///< Roll angle in deg. right. Roll is applied after yaw and pitch.

    /// Speeds
    FloatType& groundSpeedNorth = statusVector_x[ STATUS_IND_SPEED_GROUND_N	];  ///< Ground speed component North in m/s
    FloatType& groundSpeedEast = statusVector_x[ STATUS_IND_SPEED_GROUND_E	];  ///< Ground speed component East in m/s
    FloatType& trueAirSpeed = statusVector_x[ STATUS_IND_TAS		            ];  ///< True air speed horizontally in direction of heading
    FloatType& rateOfSink = statusVector_x[ STATUS_IND_RATE_OF_SINK	        ]; ///< Rate of sink in m/s relative to the surrounding air. Sink because the Z axis points downward.
    FloatType& verticalSpeed = statusVector_x[ STATUS_IND_VERTICAL_SPEED	    ]; ///< Absolute vertical speed in m/s downward. Z axis is downward.
    FloatType& thermalSpeed = statusVector_x[ STATUS_IND_THERMAL_SPEED	    ]; ///< The true reason for the whole exercise! :)

    /// Accelerations in reference to the body (plane) coordinate system.
    FloatType& accelHeading = statusVector_x[ STATUS_IND_ACC_HEADING ]; ///< Acceleration in m/s&sup2; horizontally along the heading of the plane
    FloatType& accelCross = statusVector_x[ STATUS_IND_ACC_CROSS	 ];///< Acceleration in m/s&sup2; horizontally perpendicular to the heading
    ///< Note that this does *not* include centrifugal force!!!
    ///< This is only residual acceleration not accounted by turning.
    FloatType& accelVertical = statusVector_x[ STATUS_IND_ACC_VERTICAL]; ///< Acceleration in m/s&sup2; along body Z axis

    /// Turn rates in reference to the body (plane) coordinate system
    FloatType& rollRateX = statusVector_x[ STATUS_IND_ROTATION_X	]; ///< Roll rate in deg/s to the right around the X axis
    FloatType& pitchRateY = statusVector_x[ STATUS_IND_ROTATION_Y	]; ///< Pitch rate in deg/s nose up around the Y axis
    FloatType& yawRateZ = statusVector_x[ STATUS_IND_ROTATION_Z]; ///< Yaw (turn) rate clockwise in deg/s around the Z (downward) axis

    /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter
    FloatType& gyroBiasX = statusVector_x[ STATUS_IND_GYRO_BIAS_X	     ]; ///< Bias (0-offset) of the X axis gyro in deg/s
    FloatType& gyroBiasY = statusVector_x[ STATUS_IND_GYRO_BIAS_Y	     ]; ///< Bias (0-offset) of the Y axis gyro in deg/s
    FloatType& gyroBiasZ = statusVector_x[ STATUS_IND_GYRO_BIAS_Z	     ]; ///< Bias (0-offset) of the Z axis gyro in deg/s
    FloatType& magneticDeclination = statusVector_x [ STATUS_IND_MAGNETIC_DECLINATION ] ; ///< Combined magnetic declination (variation) and deviation in degrees.
    FloatType& magneticInclination = statusVector_x [ STATUS_IND_MAGNETIC_INCLINATION]  ; ///< Inclination of the magnetic vector in degrees in pitch direction (upward), i.e. negative on the northern hemisphere
    FloatType& compassDeviationX = statusVector_x [ STATUS_IND_COMPASS_DEVIATION_X] ; ///< Strength of the local airplane magnetic field in X direction
    FloatType& compassDeviationY = statusVector_x [ STATUS_IND_COMPASS_DEVIATION_Y] ; ///< Strength of the local airplane magnetic field in Y direction
    FloatType& compassDeviationZ = statusVector_x [ STATUS_IND_COMPASS_DEVIATION_Z] ; ///< Strength of the local airplane magnetic field in Z direction
    FloatType& windSpeedNorth = statusVector_x[ STATUS_IND_WIND_SPEED_N]; ///< Wind speed North component in m/s
    FloatType& windSpeedEast  = statusVector_x[ STATUS_IND_WIND_SPEED_E]; ///< Wind speed East component in m/s
    FloatType& qff			= statusVector_x[ STATUS_IND_QFF		 ]; ///< QFF in hPascal (Using a realistic model incl. temperature, ignoring humidity).
    FloatType& lastPressure   = statusVector_x[ STATUS_IND_LAST_PRESSURE]; ///< Calculated pressure from last altMSL. Used to avoid to calculate the expensive
    ///< barometric formula multiple times. I am ignoring the error imposed by the
    ///< last altitude update :)

    /** \brief Return latitude from latitude base in arc sec, and offset in m
     *
     * Type is double to the large range
     *
     * @return Latitude in degrees
     */
    double latitude () const {
    	return (double (latitudeBaseArcSec) + double(latitudeOffs) / LEN_LAT_ARC_SEC) / 3600.0;
    }

    /** \brief Set the latitude in degrees
     *
     * Converts the latitude into the integer latitude base in arc sec and the remainder offset in m.
     * Calculates also the length of an arc second longitude at this latitude
     *
     * Type of \p lat is double due to the large range
     *
     * @param lat Latitude in degrees
     */
    void latitude (double lat);

    /** \brief Return longitude from longitude base in arc sec, and offset in m
     *
     * Type is double to the large range
     *
     * @return Longitude in degrees
     */
    double longitude () const {
    	return (double (longitudeBaseArcSec) + double(longitudeOffs) / lenLongitudeArcSec) / 3600.0;
    }

    /** \brief Set the longitude in degrees
     *
     * Converts the longitude into the integer longitude base in arc sec and the remainder offset in m.
     *
     * Type of \p lon is double due to the large range
     *
     * @param lon Longitude in degrees
     */
    void longitude (double lon);

    /// \brief \see latitudeBaseArcSec
    long getLatitudeBaseArcSec () const {
    	return latitudeBaseArcSec;
    }

    /// \brief \see longitudeBaseArcSec
    long getLongitudeBaseArcSec () const {
    	return longitudeBaseArcSec;
    }

    /// \brief \see lenLongitudeArcSec
    FloatType getLenLongitudeArcSec () const {
    	return lenLongitudeArcSec;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    StatusCoVarianceType systemNoiseCovariance_Q;
    StatusCoVarianceType errorCovariance_P;

    FloatType& longitudeOffs;  ///< Latitude offset in meter North of GliderVarioStatus::latitudeBaseArcSec
    FloatType& latitudeOffs;  ///< Longitude offset in meter East of GliderVarioStatus::latitudeBaseArcSec

    /** \brief Base value of the latitude in arc sec
     *
     * GliderVarioStatus::STATUS_IND_LATITUDE and GliderVarioStatus::latitude are the offset in *meter* whereas the latitude here is in *arc seconds*
     *
     * Also note that the type is long, i.e. only the neareast full arc second is used.
     *
     */
    long				 latitudeBaseArcSec = 0l;

    /** \brief Base value of the latitude in arc sec
     *
     * GliderVarioStatus::STATUS_IND_LATITUDE and GliderVarioStatus::latitude are the offset in *meter* whereas the latitude here is in *arc seconds*
     *
     * Also note that the type is long, i.e. only the nearest full arc second is used.
     *
     */
    long				 longitudeBaseArcSec = 0l;

    /** \brief Length of a longitude arc second at the current latitude in m
     *
     * This value is calculated from current latitudeBaseArcSec when it is initialized or updated by normalizeStatus()
     *
     */
    FloatType lenLongitudeArcSec = LEN_LAT_ARC_SEC ;

};

OEV_PUBLIC std::ostream& _printStatusVector (std::ostream &o, openEV::GliderVarioStatus::StatusVectorType const &v);
OEV_PUBLIC std::ostream& _printCovMatrix (std::ostream &o, openEV::GliderVarioStatus::StatusCoVarianceType const &co);

} // namespace openEV

OEV_PUBLIC std::ostream& operator << (std::ostream &o, openEV::GliderVarioStatus& s);
OEV_PUBLIC std::ostream& operator << (std::ostream &o, openEV::GliderVarioStatus::StatusComponentIndex ind);

// Build io manipulators for printStatusVector and printCovMatrix
struct _PrintStatusVector {
	openEV::GliderVarioStatus::StatusVectorType const &v;
};
struct _PrintCovMatrix{
	openEV::GliderVarioStatus::StatusCoVarianceType const &co;
};
// The manipulator function
inline _PrintStatusVector printStatusVector (openEV::GliderVarioStatus::StatusVectorType const &v) {
	return {v};
}
inline _PrintCovMatrix printCovMatrix(openEV::GliderVarioStatus::StatusCoVarianceType const &co) {
	return {co};
}
// The output operator of the helper class of the manipulator
inline std::ostream& operator << (std::ostream &o, _PrintStatusVector v) {
	openEV::_printStatusVector(o,v.v);
	return o;
}
inline std::ostream& operator << (std::ostream &o, _PrintCovMatrix co) {
	openEV::_printCovMatrix(o,co.co);
	return o;
}

#if defined HAVE_LOG4CXX_H
OEV_PUBLIC std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioStatus& s);
OEV_PUBLIC std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioStatus::StatusComponentIndex ind);

// The output operator of the helper class of the manipulator
inline std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, _PrintStatusVector v) {
	std::ostream &o = b;
	openEV::_printStatusVector(o,v.v);
	return o;
}
inline std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, _PrintCovMatrix co) {
	std::ostream &o = b;
	openEV::_printCovMatrix(o,co.co);
	return o;
}

#endif /* #if defined HAVE_LOG4CXX_H */

#endif /* GLIDERVARIOSTATUS_H_ */
