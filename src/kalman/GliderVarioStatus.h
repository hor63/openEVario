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

#include "OEVCommon.h"

/**
 * This namespace includes a complex extended kalman filter (EKF) for implementing a complete electronic compensated variometer, and artificial horizon.
 *
 * Input to this model are inertial measurements from 3-D accelerometer and gyroscopes, a barometric precision altimeter,
 * IAS and TAS measurement from dynamic pressure and GPS coordinates, heading and speed over ground
 *
 * Invaluable inputs for understanding Kalman filters in general, and practical implementation hints came from
 * \sa <a href="http://www.artechhouse.com/static/sample/groves-005_ch03.pdf" >Groves - CHAPTER 3 Kalman Filter-Based Estimation</a>
 *
 * For an EKF I need a Jacobian matrix with the partial derivates at the point of the last status to approximate a linearization of
 * the non-linear function at the latest status. To make my life easier I am using a numeric forward derivation with a small step.
 * For numeric derivation of angles I use an increment of 1 degree. This is small enough for an approximation of sin and cos, and large enough
 * not to create undue numeric issues with scale factors in single precision floats. It has the additional advantage that the increment is
 * 1.0 (degrees), i.e. I am saving myself a division by the increment if it were != 0.
 * \sa <a href="http://www.iue.tuwien.ac.at/phd/khalil/node14.html" >Numerical differentiations of Jacobian matrixes</a>
 *
 */
namespace openEV
{


/**
 * Constant of gravity acceleration.
 * exact values for Germany can be obtained for the German gravity base mesh
 * Deutsches Schweregrundnetz 1994 (DSGN 94) from
 * <a href="https://www.bkg.bund.de/SharedDocs/Downloads/BKG/DE/Downloads-DE-Flyer/BKG-DSGN94-DE.pdf?__blob=publicationFile&v=4" >DSGN94-Punktbeschreibung</a>
 * The constant here is a rough average between Hamburg and Munich (I live in Northern Germany).
 * Since a Kalman filter is not exact numeric science any inaccuracy should be covered by the process variance.
 *
 * \sa <a href="https://www.bkg.bund.de/DE/Ueber-das-BKG/Geodaesie/Schwere/Schwere-Deutschland/schwere-deutsch.html#doc57358bodyText1" >Deutsches Schweregrundnetz 1994 (DSGN94)</a>
 */
static FloatType constexpr GRAVITY = 9.811;

/**
 * difference between K(elvin) and C(elsius)
 */
static FloatType constexpr KtoC = 273.15f;

/**
 * Standard sea level pressure according to ICAO standard atmosphere in Pascal
 * \sa <a href="https://en.wikipedia.org/wiki/International_Standard_Atmosphere#ICAO_Standard_Atmosphere" >ICAO Standard atmosphere</a>
 */
static FloatType constexpr pressureStdMSL = 101325.0f;

/**
 * Initialization value for magnetic inclination.
 * Rough Average value for Germany is about 67 degrees downward (pitch angle is measured upward).
 *
 * \sa <a href="http://www.gfz-potsdam.de/sektion/erdmagnetfeld/infrastruktur/deutsche-saekularpunktmessungen/inklination" >Inklinationskarten für Deutschland von 1982 bis 2012</a>
 * \sa To calculate inclination and declination at any point on earth use <a href="http://www.gfz-potsdam.de/deklinationsrechner" >IGRF-Deklinationsrechner</a>
 * or <a href="http://www.ngdc.noaa.gov/IAGA/vmod/igrf.html" >International Geomagnetic Reference Field</a> and <a href="http://www.ngdc.noaa.gov/geomag-web/?model=igrf" >NOAA Magnetic Field Calculators</a>
 */
extern OEV_PUBLIC FloatType MAG_INCLINATION; // = -67.0f;

/**
 * Nautical mile to m
 *
 * \sa <a href="https://en.wikipedia.org/wiki/Nautical_mile" >Wikipedia: Nautical mile</a>
 */
static FloatType constexpr NM_TO_M = 1852.0f;

/*
 * Here a bunch of gas constants for dry air as an ideal gas
 */

/// Universal gas constant = 8.3144598 J/mol/K
/// \sa <a href="https://en.wikipedia.org/wiki/Gas_constant" >Universal Gas Constant</a>
static FloatType constexpr R         = 8.3144598f;
/// Molar mass of dry air = 0.0289644 kg/mol
static FloatType constexpr M         = 0.0289644f;
/// Specific gas constant for dry air = R/M
/// \sa <a href="https://en.wikipedia.org/wiki/Gas_constant#Specific_gas_constant" >Specific Gas Constant for dry air</a>
static FloatType constexpr Rspec     = R/M;



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
public:

    /** \brief Index, i.e. positions of the status components in #statusVector_x
     *
     * Enumeration of the components of the Kalman status vector #statusVector_x
     */
    enum StatusComponentIndex {

        /// Constants
        STATUS_IND_GRAVITY      ,  ///< The gravity, initialized to #::GRAVITY

        /// Position and altitude
        STATUS_IND_LATITUDE  	,  ///< Latitude in arc seconds North
        STATUS_IND_LONGITUDE	,  ///< Longitude in arc seconds East
        STATUS_IND_ALT_MSL   	,  ///< Altitude in m over Mean Sea Level

        /// Attitude of the body to the world coordinate system
        STATUS_IND_HEADING		,  ///< Heading of the plane in deg. right turn from true north. This is the flight direction relative to the surrounding air.
        ///< Synonymous with Yaw.
        STATUS_IND_PITCH		,  ///< Pitch angle in deg. nose up. Pitch is applied after yaw.
        STATUS_IND_ROLL		    ,  ///< Roll angle in deg. right. Roll is applied after yaw and pitch.

        /// Speeds
        STATUS_IND_SPEED_GROUND_N	,  ///< Ground speed component North in m/s
        STATUS_IND_SPEED_GROUND_E	,  ///< Ground speed component East in m/s
        STATUS_IND_TAS			,  ///< True air speed horizontally in direction of heading
        STATUS_IND_RATE_OF_SINK	, ///< Rate of sink in m/s relative to the surrounding air. Sink because the Z axis points downward
        STATUS_IND_VERTICAL_SPEED	, ///< Absolute vertical speed in m/s downward. Z axis is direction down.
        STATUS_IND_THERMAL_SPEED	, ///< The true reason for the whole exercise! :). As always in Z axis direction downward.

        /// Accelerations in direction of the heading, vertical and perpendicular to heading.
        STATUS_IND_ACC_HEADING		, ///< Acceleration in m/s^2 horizontally along the heading of the plane
        STATUS_IND_ACC_CROSS		, ///< Acceleration in m/s^2 horizontally perpendicular to the heading
        ///< Note that this does *not* include centrifugal force!!!
        ///< This is only residual acceleration not accounted by turning.
        STATUS_IND_ACC_VERTICAL		, ///< Acceleration in m/s^2 along body Z axis

        /// Turn rates in reference to the world coordinate system
        STATUS_IND_ROTATION_X	, ///< Roll rate in deg/s to the right around the X axis
        STATUS_IND_ROTATION_Y	, ///< Pitch rate in deg/s nose up around the Y axis
        STATUS_IND_ROTATION_Z	, ///< Yaw (turn) rate clockwise in deg/s around the Z (downward) axis

        /// Derived values which improve the responsiveness of the Kalman filter.
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
        STATUS_IND_QFF			, ///< QFF in Pascal (Using a more realistic model incl. temperature, but ignoring humidity).
        STATUS_IND_LAST_PRESSURE, ///< Calculated pressure from last altMSL. Used to avoid to calculate the expensive
        ///< barometric formula multiple times. I am ignoring the error imposed by the
        ///< last altitude update :)

        STATUS_NUM_ROWS				///< The number of rows in the vector. This is not a component of the vector!
    };

    typedef Eigen::Matrix<FloatType,STATUS_NUM_ROWS,1> StatusVectorType; ///< Saves typing of the complex template type
    typedef Eigen::SparseMatrix<FloatType> StatusCoVarianceType; ///< Co-variance matrix type for P and Q

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
     */
    void normalizeAngles();

    // Here come all state vector elements as single references into the vector for easier access

    /// Constants
    FloatType& gravity  = statusVector_x [ STATUS_IND_GRAVITY ];  ///< The gravity, initialized to #::GRAVITY

    /// Position and altitude
    FloatType& longitude = statusVector_x[ STATUS_IND_LONGITUDE	];  ///< Latitude in arc seconds North
    FloatType& latitude = statusVector_x[ STATUS_IND_LATITUDE  	];  ///< Longitude in arc seconds East
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
    FloatType& accelHeading = statusVector_x[ STATUS_IND_ACC_HEADING ]; ///< Acceleration in m/s^2 horizontally along the heading of the plane
    FloatType& accelCross = statusVector_x[ STATUS_IND_ACC_CROSS	 ];///< Acceleration in m/s^2 horizontally perpendicular to the heading
    ///< Note that this does *not* include centrifugal force!!!
    ///< This is only residual acceleration not accounted by turning.
    FloatType& accelVertical = statusVector_x[ STATUS_IND_ACC_VERTICAL]; ///< Acceleration in m/s^2 along body Z axis

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
    FloatType& qff			= statusVector_x[ STATUS_IND_QFF		 ]; ///< QFF in Pascal (Using a realistic model incl. temperature, ignoring humidity).
    FloatType& lastPressure   = statusVector_x[ STATUS_IND_LAST_PRESSURE]; ///< Calculated pressure from last altMSL. Used to avoid to calculate the expensive
    ///< barometric formula multiple times. I am ignoring the error imposed by the
    ///< last altitude update :)


protected:
    StatusVectorType     statusVector_x;
    StatusCoVarianceType systemNoiseCovariance_Q;
    StatusCoVarianceType errorCovariance_P;


};

} // namespace openEV

OEV_PUBLIC std::ostream& operator << (std::ostream& o ,openEV::GliderVarioStatus& s);
OEV_PUBLIC std::ostream& operator << (std::ostream &o, openEV::GliderVarioStatus::StatusVectorType &v);
OEV_PUBLIC std::ostream& operator << (std::ostream &o, openEV::GliderVarioStatus::StatusCoVarianceType &co);

#endif /* GLIDERVARIOSTATUS_H_ */
