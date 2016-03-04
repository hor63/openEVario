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
class GliderVarioMeasurementVector {
public:
	GliderVarioMeasurementVector() {
		measureVector.setZero();

	}
	virtual ~GliderVarioMeasurementVector();

	enum MeasureComponentIndex {
		// GPS Measurements
		MEASURE_IND_GPS_LAT, ///< Latitude in Deg
		MEASURE_IND_GPS_LON, ///< Longitude in Deg
		MEASURE_IND_GPS_ALTMSL, ///< Altitude MSL in m
		MEASURE_IND_GPS_HEADING, ///< Heading in Deg
		MEASURE_IND_GPS_SPEED, ///< Speed in knots

		// Accelerometer
		MEASURE_IND_ACC_X, ///< Acceleration along the X axis in m/s^2
		MEASURE_IND_ACC_Y, ///< Acceleration along the Y axis in m/s^2
		MEASURE_IND_ACC_Z, ///< Acceleration along the Z axis in m/s^2

		// Gyro
		MEASURE_IND_GYRO_RATE_X, ///< Turn rate around the X axis in Deg/s
		MEASURE_IND_GYRO_RATE_Y, ///< Turn rate around the Y axis in Deg/s
		MEASURE_IND_GYRO_RATE_Z, ///< Turn rate around the Z axis in Deg/s

		// Magnetometer
		MEASURE_IND_MAG_X, ///< magnetic field strength along X axis in uT (absolute strength is irrelevant, only used to determine attitude)
		MEASURE_IND_MAG_Y, ///< magnetic field strength along Y axis in uT (absolute strength is irrelevant, only used to determine attitude)
		MEASURE_IND_MAG_Z, ///< magnetic field strength along Z axis in uT (absolute strength is irrelevant, only used to determine attitude)

		// Air pressure values (converted because raw values are highly non-linear to speed and altitude
		MEASURE_IND_PRESS_ALT, ///< pressure altitude in MSL
		MEASURE_IND_TAS,       ///< True air speed (based on difference pressure and air density based on absolute pressure) in m/s

		MEASURE_NUM_ROWS
	};

	typedef Eigen::Matrix<FloatType,MEASURE_NUM_ROWS,1> MeasureVectorType;

	// Here come all measurement components as references into the matrix
	FloatType &gpsLatitude = measureVector [MEASURE_IND_GPS_LAT]; ///< Latitude in Deg
	FloatType &gpsLongitude = measureVector [MEASURE_IND_GPS_LON]; ///< Longitude in Deg
	FloatType &gpsMSL = measureVector [MEASURE_IND_GPS_ALTMSL]; ///< Altitude MSL in m
	FloatType &gpsHeading = measureVector [MEASURE_IND_GPS_HEADING]; ///< Heading in Deg
	FloatType &gpsSpeed = measureVector [MEASURE_IND_GPS_SPEED]; ///< Speed in knots

	// Accelerometer
	FloatType &accelX = measureVector [MEASURE_IND_ACC_X]; ///< Acceleration along the X axis in m/s^2
	FloatType &accelY = measureVector [MEASURE_IND_ACC_Y]; ///< Acceleration along the Y axis in m/s^2
	FloatType &accelZ = measureVector [MEASURE_IND_ACC_Z]; ///< Acceleration along the Z axis in m/s^2

	// Gyro
	FloatType &gyroRateX = measureVector [MEASURE_IND_GYRO_RATE_X]; ///< Turn rate around the X axis in Deg/s
	FloatType &gyroRateY = measureVector [MEASURE_IND_GYRO_RATE_Y]; ///< Turn rate around the Y axis in Deg/s
	FloatType &gyroRateZ = measureVector [MEASURE_IND_GYRO_RATE_Z]; ///< Turn rate around the Z axis in Deg/s

	// Magnetometer
	FloatType &magX = measureVector [MEASURE_IND_MAG_X]; ///< magnetic field strength along X axis in uT (absolute strength is irrelevant, only used to determine attitude)
	FloatType &magY = measureVector [MEASURE_IND_MAG_Y]; ///< magnetic field strength along Y axis in uT (absolute strength is irrelevant, only used to determine attitude)
	FloatType &magZ = measureVector [MEASURE_IND_MAG_Z]; ///< magnetic field strength along Z axis in uT (absolute strength is irrelevant, only used to determine attitude)

	// Air pressure values (converted because raw values are highly non-linear to speed and altitude
	FloatType &pressAlt = measureVector [MEASURE_IND_PRESS_ALT]; ///< pressure altitude in MSL
	FloatType &trueAirSpeed = measureVector [MEASURE_IND_TAS];       ///< True air speed (based on difference pressure and air density based on absolute pressure) in m/s

	MeasureVectorType const getMeasureVector() const {
		return measureVector;
	}

protected:
	MeasureVectorType measureVector; ///< holder of the vector
};

} /* namespace openEV */

#endif /* GLIDERVARIOMEASUREMENTVECTOR_H_ */
