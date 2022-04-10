/* Implementation of the openEVario status class
 *
 * GliderVarioStatus.cpp
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

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include "GliderVarioStatus.h"
#include "util/FastMath.h"
#include <iomanip>

namespace openEV
{

FloatType MAG_INCLINATION = -67.0f;

#if !defined DOXYGEN
GliderVarioStatus::StatusComponentIndexHelperClass GliderVarioStatus::StatusComponentIndexHelperObj;
#endif

GliderVarioStatus::GliderVarioStatus () :
		statusVector_x{StatusVectorType::Zero()},
	    // All state vector elements as single references into the vector for easier access
	    /// Constants
	    gravity {statusVector_x ( STATUS_IND_GRAVITY )},


	    /// Position and altitude
	    longitudeOffsC {statusVector_x[ STATUS_IND_LONGITUDE_OFFS]},
	    latitudeOffsC {statusVector_x[ STATUS_IND_LATITUDE_OFFS]},
	    altMSL {statusVector_x[ STATUS_IND_ALT_MSL]},

	    /// Attitude of the body to the world coordinate system
	    heading {statusVector_x[ STATUS_IND_HEADING]},
	    pitchAngle {statusVector_x[ STATUS_IND_PITCH]},
	    rollAngle {statusVector_x[ STATUS_IND_ROLL]},

	    /// Speeds
	    groundSpeedNorth {statusVector_x[ STATUS_IND_SPEED_GROUND_N]},
	    groundSpeedEast {statusVector_x[ STATUS_IND_SPEED_GROUND_E]},
	    trueAirSpeed {statusVector_x[ STATUS_IND_TAS]},
	    rateOfSink {statusVector_x[ STATUS_IND_RATE_OF_SINK]},
	    verticalSpeed {statusVector_x[ STATUS_IND_VERTICAL_SPEED]},
	    thermalSpeed {statusVector_x[ STATUS_IND_THERMAL_SPEED]},

	    /// Accelerations in reference to the body (plane) coordinate system.
	    accelHeading {statusVector_x[ STATUS_IND_ACC_HEADING]},
	    accelCross {statusVector_x[ STATUS_IND_ACC_CROSS]},
	    accelVertical {statusVector_x[ STATUS_IND_ACC_VERTICAL]},

	    /// Turn rates in reference to the body (plane) coordinate system
	    rollRateX {statusVector_x[ STATUS_IND_ROTATION_X]},
	    pitchRateY {statusVector_x[ STATUS_IND_ROTATION_Y]},
	    yawRateZ {statusVector_x[ STATUS_IND_ROTATION_Z]},

	    /// Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter
	    gyroBiasX {statusVector_x[ STATUS_IND_GYRO_BIAS_X]},
	    gyroBiasY {statusVector_x[ STATUS_IND_GYRO_BIAS_Y]},
	    gyroBiasZ {statusVector_x[ STATUS_IND_GYRO_BIAS_Z]},
	    magneticDeclination {statusVector_x [ STATUS_IND_MAGNETIC_DECLINATION]},
	    magneticInclination {statusVector_x [ STATUS_IND_MAGNETIC_INCLINATION]},
	    compassDeviationX {statusVector_x [ STATUS_IND_COMPASS_DEVIATION_X]},
	    compassDeviationY {statusVector_x [ STATUS_IND_COMPASS_DEVIATION_Y]},
	    compassDeviationZ {statusVector_x [ STATUS_IND_COMPASS_DEVIATION_Z]},
	    windSpeedNorth {statusVector_x[ STATUS_IND_WIND_SPEED_N]},
	    windSpeedEast  {statusVector_x[ STATUS_IND_WIND_SPEED_E]},
	    qff {statusVector_x[ STATUS_IND_QFF]},
	    lastPressure {statusVector_x[ STATUS_IND_LAST_PRESSURE]},
		systemNoiseCovariance_Q{STATUS_NUM_ROWS,STATUS_NUM_ROWS},
		errorCovariance_P{STATUS_NUM_ROWS,STATUS_NUM_ROWS},
	    longitudeOffs {statusVector_x[ STATUS_IND_LONGITUDE_OFFS]},
	    latitudeOffs {statusVector_x[ STATUS_IND_LATITUDE_OFFS]}
{

    // Changed to sparse matrixes which start empty by default
    // they are initialized to their dimensions in the initialization section of the constructor
    // above.
	// Reserve enough memory for the sparse matrixes to avoid realloc() and heap fragmentation
	// Usually the system error matrix is diagonal only
    systemNoiseCovariance_Q.reserve(STATUS_NUM_ROWS*2);

    // This one also contains the dependencies of the factors outside the diagonal, but not all are used.
    errorCovariance_P.reserve(STATUS_NUM_ROWS*STATUS_NUM_ROWS/2);


    statusVector_x(STATUS_IND_GRAVITY)= GRAVITY;
    statusVector_x(STATUS_IND_LAST_PRESSURE) = PressureStdMSL;
    statusVector_x(STATUS_IND_QFF) = PressureStdMSL;

}

GliderVarioStatus::~GliderVarioStatus ()
{

}

void GliderVarioStatus::normalizeStatus() {

    if (pitchAngle < -90.0f) {
        if (pitchAngle < -360.0f) {
            do {
                pitchAngle += 360.0f;
            } while (pitchAngle < -360.0f);
            // pitch angle can now be anywhere between 0 and -360
            // Therefore do it again recursively.
            return normalizeStatus();
        } // if (pitchAngle < -360.0f)
        if (pitchAngle <= -180.0f) {
            // I am at an upward pointing angle
            pitchAngle += 360.0f;
        } else { // if (pitchAngle <= -180.0f)
            /* I am between -90 and -180 deg, i.e. positive looping beyond perpendicular
             * i.e. I am flipping direction and roll, and bring pitch back into the 1st quadrant
             */
            pitchAngle = -180.0f - pitchAngle;
            rollAngle += 180.0f;
            heading += 180.0f;
        } // if (pitchAngle <= -180.0f)

    } // if (pitchAngle < -90.0f)

    if (pitchAngle > 90.0f) {
        if (pitchAngle >= 360.0f) {
            do {
                pitchAngle -= 360.0f;
            } while (pitchAngle >= 360.0f);
            // pitch angle can now be anywhere between 0 and +360
            // Therefore do it again recursively.
            return normalizeStatus();
        } // if (pitchAngle > 360.0f)
        if (pitchAngle > 180.0f) {
            pitchAngle -= 360.0f;
            /* Pitch angle is negative now, and between -0 and -180. I deal with that above. Ugh.
             * So I call myself recursively
             */
            return normalizeStatus();
        } else { // if (pitchAngle > 180.0f)
            /* I am between 90 and 180 deg, i.e. looping beyond perpendicular
             * i.e. I am flipping direction and roll, and bring pitch back into the 1st quadrant
             */
            pitchAngle = 180.0f - pitchAngle;
            rollAngle += 180.0f;
            heading += 180.0f;
        } // if (pitchAngle > 180.0f)
    } // if (pitchAngle > 90.0f)

    // roll and yaw are easier: They are independent from the others :)
    if (rollAngle < -180.0f) {

        if (rollAngle >= -360.0f) {
            // This is the normal case
            rollAngle += 360.0f;
        } else { // if (rollAngle >= -360.0f)
            // This should actually happen very rarely if at all...
            do {
                rollAngle += 360.0f;
            } while (rollAngle < -360.0f);

            if (rollAngle < -180.0f) {
                rollAngle += 360.0f;
            }
        } // if (rollAngle >= -360.0f)
    } // if (rollAngle < -180.0f)
    if (rollAngle >= 180.0f) {

        if (rollAngle <= 360.0f) {
            // This is the normal case
            rollAngle -= 360.0f;
        } else { // if (rollAngle >= -360.0f)
            // This should actually happen very rarely if at all...
            do {
                rollAngle -= 360.0f;
            } while (rollAngle > 360.0f);

            if (rollAngle >= 180.0f) {
                rollAngle -= 360.0f;
            }
        } // if (rollAngle >= -360.0f)
    } // if (rollAngle < -180.0f)

    while (heading < 0.0f) {
        heading += 360.0f;
    }
    while (heading >= 360.0f) {
        heading -= 360.0f;
    }


    // The normalization is admittedly extremely primitive but given the fact that an arc sec. is 30-something meters, and
    // the Kalman filter will never run above 100ms per cycle even at 300km/s the plane will never move > 1 arc sec per cycle.
    if (latitudeOffs > LEN_LAT_ARC_SEC) {
    	// Save the current longitude because the length of an arc second will change with the new latitude
    	double longitudeBak = longitude();

    	while (latitudeOffs > LEN_LAT_ARC_SEC) {
    		latitudeOffs -= LEN_LAT_ARC_SEC;
    		latitudeBaseArcSec ++;
    	}
        lenLongitudeArcSec = fabsf(LEN_LAT_ARC_SEC * FastMath::fastCos(FloatType(latitudeBaseArcSec/3600.0f)));

    	longitude(longitudeBak);
    }
    if (latitudeOffs < -LEN_LAT_ARC_SEC) {
    	// Save the current longitude because the length of an arc second will change with the new latitude
    	double longitudeBak = longitude();

    	while (latitudeOffs < -LEN_LAT_ARC_SEC) {
    		latitudeOffs += LEN_LAT_ARC_SEC;
    		latitudeBaseArcSec --;
    	}
        lenLongitudeArcSec = LEN_LAT_ARC_SEC * FastMath::fastCos(FloatType(latitudeBaseArcSec/3600.0f));

    	longitude(longitudeBak);
    }


	while (longitudeOffs > lenLongitudeArcSec) {
		longitudeOffs -= lenLongitudeArcSec;
		longitudeBaseArcSec ++;
	}
	while (longitudeOffs < -lenLongitudeArcSec) {
		longitudeOffs += lenLongitudeArcSec;
		longitudeBaseArcSec --;
	}


}

void openEV::GliderVarioStatus::latitude(double lat) {

	// Save the current longitude because the length of an arc second will change with the new latitude
	double longitudeBak = longitude();

	lat *= 3600.0;

	latitudeBaseArcSec	= lround(lat);
	latitudeOffs		= (lat - latitudeBaseArcSec) * LEN_LAT_ARC_SEC;

    lenLongitudeArcSec = LEN_LAT_ARC_SEC * FastMath::fastCos(FloatType(latitudeBaseArcSec/3600.0));

	longitude(longitudeBak);

}

void openEV::GliderVarioStatus::longitude(double lon) {

	lon *= 3600.0;

	longitudeBaseArcSec	= lround(lon);
	longitudeOffs		= (lon - longitudeBaseArcSec) * lenLongitudeArcSec;

}

static char const * const statusFieldNames [openEV::GliderVarioStatus::STATUS_NUM_ROWS] = {
        "        gravity",
        "  longitudeOffs",
        "   latitudeOffs",
        "         altMSL",
        "        heading",
        "     pitchAngle",
        "      rollAngle",
        "   groundSpeedN",
        "   groundSpeedE",
        "   trueAirSpeed",
        "     rateOfSink",
        "  verticalSpeed",
        "   thermalSpeed",
        "   accelHeading",
        "     accelCross",
        "  accelVertical",
        "      rollRateX",
        "     pitchRateY",
        "       yawRateZ",
        "      gyroBiasX",
        "      gyroBiasY",
        "      gyroBiasZ",
        " magDeclination",
        " magInclination",
        "   compDeviatiX",
        "   compDeviatiY",
        "   compDeviatiZ",
        " windSpeedNorth",
        "  windSpeedEast",
        "            QFF",
        "   lastPressure",
};

std::ostream& _printStatusVector (std::ostream &o, openEV::GliderVarioStatus::StatusVectorType const &v) {

    o << "\n";

    for (int i = 0 ; i < openEV::GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        o << statusFieldNames[i];
    }

    o << "\n" << std::fixed;
    o.precision(6);
    o.fill('_');

    for (int i = 0 ; i < openEV::GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        o.precision(6);
        o.width(15);
        o << v(i,0)     ;
    }

    o << std::endl;

    return o;
}

std::ostream& _printCovMatrix (std::ostream &o, openEV::GliderVarioStatus::StatusCoVarianceType const &co) {

    o << "\n               ";

    for (int i = 0 ; i < openEV::GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        o << statusFieldNames[i];
    }

    o << "\n" << std::fixed;
    o.precision(6);
    o.fill('_');

    for (int i = 0 ; i < openEV::GliderVarioStatus::STATUS_NUM_ROWS; i++) {

        o << "\n" << statusFieldNames[i];

        for (int k = 0 ; k < openEV::GliderVarioStatus::STATUS_NUM_ROWS; k++) {
            o.precision(6);
            o.width(15);
            o << co.coeff(i,k)     ;
        }
    }

    o << std::endl;

    return o;
}



} // namespace openEV


std::ostream& operator <<(std::ostream &o, openEV::GliderVarioStatus &s) {

	openEV::_printStatusVector(o,s.getStatusVector_x());

    return o;
}

std::ostream& operator << (std::ostream &o,openEV::GliderVarioStatus::StatusComponentIndex ind) {
    o << openEV::GliderVarioStatus::StatusComponentIndexHelperObj.getString(ind);
    return o;
}

#if defined HAVE_LOG4CXX_H
OEV_PUBLIC std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioStatus& s) {
	std::ostream &o = b;
	return operator << (o,s);
}
OEV_PUBLIC std::ostream& printStatusVector (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioStatus::StatusVectorType &v) {
	std::ostream &o = b;
	return openEV::_printStatusVector (o,v);
}
OEV_PUBLIC std::ostream& printCovMatrix (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioStatus::StatusCoVarianceType &co) {
	std::ostream &o = b;
	return openEV::_printCovMatrix (o,co);
}
OEV_PUBLIC std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioStatus::StatusComponentIndex ind) {
	std::ostream &o = b;
	return operator << (o,ind);
}
#endif /* #if defined HAVE_LOG4CXX_H */


