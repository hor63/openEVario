/*
 * GliderVarioMeasurementVector.h
 *
 *  Created on: Jan 31, 2016
 *      Author: hor
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

#ifndef GLIDERVARIOMEASUREMENTVECTOR_H_
#define GLIDERVARIOMEASUREMENTVECTOR_H_

#include "GliderVarioStatus.h"

namespace openEV {

/**
 * This is the measurement input vector into the Kalman filter.
 * Not all measurements are the raw instrument readings.
 * Particularly pressure readings are converted into altitude and speed before because the conversions are highly non-linear.
 * Otherwise all units are converted to ISO base units.
 * Absolute Magnetometer readings are irrelevant but their ratios are used to estimate the attitude.
 */
class OEV_PUBLIC GliderVarioMeasurementVector {
public:
    GliderVarioMeasurementVector();
    virtual ~GliderVarioMeasurementVector();

#if defined DOXYGEN
    enum MeasureComponentIndex {
#else
	OEV_ENUM(MeasureComponentIndex,
#endif

        // GPS Measurements
        MEASURE_IND_GPS_LAT, ///< Latitude in arc seconds
        MEASURE_IND_GPS_LON, ///< Longitude in arc seconds
        MEASURE_IND_GPS_ALT_MSL, ///< Altitude MSL (geoid) in m
        MEASURE_IND_GPS_HEADING, ///< Heading in Deg
        MEASURE_IND_GPS_SPEED, ///< Speed in m/s

        // Accelerometer
        MEASURE_IND_ACC_X, ///< Acceleration along the X axis in m/s&sup2;
        MEASURE_IND_ACC_Y, ///< Acceleration along the Y axis in m/s&sup2;
        MEASURE_IND_ACC_Z, ///< Acceleration along the Z axis in m/s&sup2;

        // Gyro
        MEASURE_IND_GYRO_RATE_X, ///< Turn rate around the X axis in Deg/s
        MEASURE_IND_GYRO_RATE_Y, ///< Turn rate around the Y axis in Deg/s
        MEASURE_IND_GYRO_RATE_Z, ///< Turn rate around the Z axis in Deg/s

        // Magnetometer
        MEASURE_IND_MAG_X, ///< magnetic field strength along X axis in uT (absolute strength is irrelevant, only used to determine attitude)
        MEASURE_IND_MAG_Y, ///< magnetic field strength along Y axis in uT (absolute strength is irrelevant, only used to determine attitude)
        MEASURE_IND_MAG_Z, ///< magnetic field strength along Z axis in uT (absolute strength is irrelevant, only used to determine attitude)

        // Air pressure values
        MEASURE_IND_STATIC_PRESSURE,  ///< static pressure in Pascal
        MEASURE_IND_DYNAMIC_PRESSURE, ///< dynamic (pitot) pressure in Pascal, i.e. total pressure-static pressure.

        MEASURE_NUM_ROWS
#if defined DOXYGEN
	    };
#else
	);
#endif

    typedef Eigen::Matrix<FloatType,MEASURE_NUM_ROWS,1> MeasureVectorType;
    typedef Eigen::Matrix<FloatType,MEASURE_NUM_ROWS,MEASURE_NUM_ROWS> MeasureCovarianceType;

protected:
        MeasureVectorType measureVector; ///< holder of the vector

public:
    // Here come all measurement components as references into the matrix
    FloatType &gpsLatitude; ///< Latitude in arc seconds
    FloatType &gpsLongitude; ///< Longitude in arc seconds
    FloatType &gpsMSL; ///< Altitude MSL in m
    FloatType &gpsHeading; ///< Heading in Deg
    FloatType &gpsSpeed; ///< Speed in m/s

    // Accelerometer
    FloatType &accelX; ///< Acceleration along the X axis in m/s&sup2;
    FloatType &accelY; ///< Acceleration along the Y axis in m/s&sup2;
    FloatType &accelZ; ///< Acceleration along the Z axis in m/s&sup2;

    // Gyro
    FloatType &gyroRateX; ///< Turn rate around the X axis in Deg/s
    FloatType &gyroRateY; ///< Turn rate around the Y axis in Deg/s
    FloatType &gyroRateZ; ///< Turn rate around the Z axis in Deg/s

    // Magnetometer
    FloatType &magX; ///< magnetic field strength along X axis in uT (absolute strength is irrelevant, only used to determine attitude)
    FloatType &magY; ///< magnetic field strength along Y axis in uT (absolute strength is irrelevant, only used to determine attitude)
    FloatType &magZ; ///< magnetic field strength along Z axis in uT (absolute strength is irrelevant, only used to determine attitude)

    // Air pressure values (converted because raw values are highly non-linear to speed and altitude
    FloatType &staticPressure; ///< static pressure in Pascal
    FloatType &dynamicPressure;       ///< True air speed (based on difference pressure and air density based on absolute pressure) in m/s

    /**
     *
     * @return constant reference to the internal vector for direct matrix manipulation.
     */
    MeasureVectorType const &getMeasureVector() const {
        return measureVector;
    }

    /**
     *
     * @return reference to the internal vector for direct matrix manipulation.
     */
    MeasureVectorType &getMeasureVector() {
        return measureVector;
    }

    /**
     *  @return constant reference to the covariance of the measurement vector.
     */
    MeasureVectorType const &getMeasureError() const {
        return measureError;
    }

    /**
     *  @return reference to the covariance of the measurement vector.
     */
    MeasureVectorType &getMeasureError() {
        return measureError;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    MeasureVectorType measureError; ///< I am not using a measurement covariance matrix here as in classic Kalman theory.
    ///< The error variances are supposed to be independent, i.e. a matrix would be diagonal.
    ///< In addition I am using the sequential measurement method according to
    ///< <a href="http://www.artechhouse.com/static/sample/groves-005_ch03.pdf" >Groves - Kalman Filter-Based Estimation</a>, page 107


};

} /* namespace openEV */

OEV_PUBLIC std::ostream& operator << (std::ostream &o, openEV::GliderVarioMeasurementVector::MeasureComponentIndex ind);

#endif /* GLIDERVARIOMEASUREMENTVECTOR_H_ */
