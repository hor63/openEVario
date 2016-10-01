/*
 * GliderVarioMeasurementMatrix.cpp
 *
 *  Created on: Feb 14, 2016
 *      Author: hor
 */

#include "GliderVarioMeasurementMatrix.h"

namespace openEV {

GliderVarioMeasurementMatrix::GliderVarioMeasurementMatrix() {

	measurementMatrix.setZero();

	// Most of the stuff is totally straight forward, i.e. factor 1 :)
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_GPS_LAT,		GliderVarioStatus::STATUS_IND_LATITUDE	) = 1.0;
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_GPS_LON,		GliderVarioStatus::STATUS_IND_LONGITUDE	) = 1.0;
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_GPS_ALT_MSL,	GliderVarioStatus::STATUS_IND_ALT_MSL	) = 1.0;
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_GPS_HEADING,	GliderVarioStatus::STATUS_IND_HEADING	) = 1.0;
	// m/s to knots.
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_GPS_SPEED,	GliderVarioStatus::STATUS_IND_HEADING	) = 3600.0/1852.0;

	// Accelerometer
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_ACC_X,		GliderVarioStatus::STATUS_IND_ACC_X		) = 1.0;
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_ACC_Y,		GliderVarioStatus::STATUS_IND_ACC_Y		) = 1.0;
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_ACC_Z,		GliderVarioStatus::STATUS_IND_ACC_Z		) = 1.0;

	// Gyro
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_GYRO_RATE_X,	GliderVarioStatus::STATUS_IND_ROTATION_X) = 1.0;
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_GYRO_RATE_Y,	GliderVarioStatus::STATUS_IND_ROTATION_Y) = 1.0;
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_GYRO_RATE_Z,	GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1.0;

	// Air pressure values (converted because raw values are highly non-linear to speed and altitude
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_PRESS_ALT,	GliderVarioStatus::STATUS_IND_ALT_MSL) = 1;
	measurementMatrix ( GliderVarioMeasurementVector::MEASURE_IND_TAS,			GliderVarioStatus::STATUS_IND_TAS) = 1;


}

GliderVarioMeasurementMatrix::~GliderVarioMeasurementMatrix() {

}

void
GliderVarioMeasurementMatrix::calcMeasurementMatrix (
    FloatType timeDiff,
    GliderVarioStatus const &lastStatus) {

	// Magnetometer
	/// @todo Normalized magnetometer values to direction
	// MEASURE_IND_MAG_X, ///< magnetic field strength along X axis in uT (absolute strength is irrelevant, only used to determine attitude)
	// MEASURE_IND_MAG_Y, ///< magnetic field strength along Y axis in uT (absolute strength is irrelevant, only used to determine attitude)
	// MEASURE_IND_MAG_Z, ///< magnetic field strength along Z axis in uT (absolute strength is irrelevant, only used to determine attitude)

}

} /* namespace openEV */
