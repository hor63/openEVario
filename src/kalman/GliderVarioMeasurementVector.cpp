/*
 * GliderVarioMeasurementVector.cpp
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "GliderVarioMeasurementVector.h"

namespace openEV {

#if !defined DOXYGEN
GliderVarioMeasurementVector::MeasureComponentIndexHelperClass GliderVarioMeasurementVector::MeasureComponentIndexHelperObj;
#endif

GliderVarioMeasurementVector::GliderVarioMeasurementVector() :
	    // All measurement components as references into the matrix
	    gpsLatitude {measureVector [MEASURE_IND_GPS_LAT]},
	    gpsLongitude {measureVector [MEASURE_IND_GPS_LON]},
	    gpsMSL {measureVector [MEASURE_IND_GPS_ALT_MSL]},
	    gpsHeading {measureVector [MEASURE_IND_GPS_HEADING]},
	    gpsSpeed {measureVector [MEASURE_IND_GPS_SPEED]},

	    // Accelerometer
	    accelX {measureVector [MEASURE_IND_ACC_X]},
	    accelY {measureVector [MEASURE_IND_ACC_Y]},
	    accelZ {measureVector [MEASURE_IND_ACC_Z]},

	    // Gyro
	    gyroRateX {measureVector [MEASURE_IND_GYRO_RATE_X]},
	    gyroRateY {measureVector [MEASURE_IND_GYRO_RATE_Y]},
	    gyroRateZ {measureVector [MEASURE_IND_GYRO_RATE_Z]},

	    // Magnetometer
	    magX {measureVector [MEASURE_IND_MAG_X]},
	    magY {measureVector [MEASURE_IND_MAG_Y]},
	    magZ {measureVector [MEASURE_IND_MAG_Z]},

	    // Air pressure values (converted because raw values are highly non-linear to speed and altitude
	    staticPressure {measureVector [MEASURE_IND_STATIC_PRESSURE]},
	    dynamicPressure {measureVector [MEASURE_IND_DYNAMIC_PRESSURE]},

		tempLocalK {measureVector [MEASURE_IND_TEMP_LOCAL_K]}
{
	measureVector.fill(NAN);
	measureError.setZero();

	// Initialize the temperature to 15°C
	measureVector [MEASURE_IND_TEMP_LOCAL_K] = 15.0f + CtoK;

	/// \todo Remove after testing
	measureVector [MEASURE_IND_STATIC_PRESSURE] = 1000.0f * 100.0f; // In Pascal, not hPa, i.e. mBar!
}

GliderVarioMeasurementVector::~GliderVarioMeasurementVector() {

}

} /* namespace openEV */

std::ostream& operator << (std::ostream &o, openEV::GliderVarioMeasurementVector::MeasureComponentIndex ind) {
    o << openEV::GliderVarioMeasurementVector::MeasureComponentIndexHelperObj.getString(ind);
    return o;
}

