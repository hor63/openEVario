/*
 * GliderVarioDriverBase.h
 *
 *  Created on: Oct 01, 2017
 *      Author: hor
 *
 *  Base class of driver classes for OpenEVario
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

#ifndef GLIDERVARIODRIVERBASE_H_
#define GLIDERVARIODRIVERBASE_H_

#include <cstdint>

#if HAVE_CONFUSE_H == 1
    #include <confuse.h>
#else
#	error Confuse configuration library is not available
#endif

#include <list>

#include "OEVCommon.h"
#include "kalman/GliderVarioStatus.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

namespace openEV {

/** \brief Abstract base class for sensor drivers.
 *
 * Abstract base class of sensor drivers. Most methods are pure virtual, and must be implemented by the driver class
 * The actual driver implementation is never exposed to the main program but hidden in the shared library which
 * implements the driver
 */
class OEV_DRIVER_PUBLIC GliderVarioDriverBase {

public:

    GliderVarioDriverBase ()
    : sensorCapabilities {0}
    {
        ;
    }

    virtual ~GliderVarioDriverBase () {}

    /// The list of sensor capabilities. Each one is a position in an integer.
    /// The capabilities are ORed into the capabilities of the driver in #sensorCapabilities.
#if defined DOXYGEN
    enum SensorCapability {
#else
    OEV_ENUM (SensorCapability,
#endif

        GPS_LATITUDE = 0,
        GPS_LONGITUDE = 1,
        GPS_ALTITUDE_MSL = 2,
        GPS_HEADING = 3,
        GPS_SPEED = 4,
        ACCEL_X = 5,
        ACCEL_Y = 6,
        ACCEL_Z = 7,
        GYRO_X = 8,
        GYRO_Y = 9,
        GYRO_Z = 10,
        COMPASS_X = 11,
        COMPASS_Y = 12,
        COMPASS_Z = 13,
        STATIC_PRESSURE = 14,
        DYNAMIC_PRESSURE = 15,
#if defined DOXYGEN
	    };
#else
	);
#endif

    /**
     * Get the capabilities of the sensor defined in #sensorCapabilities.
     * @return capabilities as bit list. The bits are positions defined by #SensorCapability.
     */
    inline uint32_t getSensorCapabilities () {
        return sensorCapabilities;
    }

    /// Check if the driver implements a capability defined in #SensorCapability.
    inline bool hasSensorCapability (SensorCapability capability) {
        return (sensorCapabilities & (1UL<<capability)) != 0;
    }

    /// Set a driver capability. Capabilities are defined in #SensorCapability.
    inline void setSensorCapability (SensorCapability capability) {
        sensorCapabilities |= (1UL<<capability);
    }

    /// Clear a driver capability. Capabilities are defined in #SensorCapability.
    inline void clearSensorCapability (SensorCapability capability) {
        sensorCapabilities &= ~(1UL<<capability);
    }

    // start of the abstract interface which must be implemented by each driver.

    /**
     * Initialize the driver
     */
    virtual void driverInit() = 0;

    /**
     * Add list of options to be read from the configuration file for this driver.
     * The configuration options added are being read from the configuration file, and being passed to this driver
     * with #configureDriver.
     * @param optionList List of options. Add options for your driver to this list with optionList.push_back
     */
    virtual void addConfigurationOptions(std::list<cfg_opt_t>&optionList) = 0;

    /**
     * Configure your driver with the configuration values which you added with #addConfigurationOptions.
     * @param cfg
     */
    virtual void configureDriver (cfg_t* cfg) = 0;




protected:

    /// Set the capabilities of the sensor in #sensorCapabilities.
    /// To be set by the implementing class
    inline void setSensorCapabilities (uint32_t sensorCapabilities) {
        this->sensorCapabilities = sensorCapabilities;
    }

    /// Bit list of capabilities. The bit positions are defined in the enum #SensorCapability.
    uint32_t sensorCapabilities;

    // Abstract functions allowing sub-classing of the driver

    /**
     * Apply the measurements managed by this driver to the Kalman filter.
     * Measurement values are to be applied by the static functions of class openEV::GliderVarioMeasurementUpdater
     * @param varioStatus The current Kalman filter status.
     * @param measurementVector The current measurement vector.
     */
    virtual void applyMeasurement (GliderVarioStatus& varioStatus,GliderVarioMeasurementUpdater& measurementVector) = 0;



}; // class GliderVarioDriverBase

}

OEV_PUBLIC std::ostream& operator << (std::ostream &o,openEV::GliderVarioDriverBase::SensorCapability ind);


#if defined HAVE_LOG4CXX_H
OEV_PUBLIC std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioDriverBase::_SensorCapability e);
#endif /* #if defined HAVE_LOG4CXX_H */

inline openEV::GliderVarioDriverBase::_SensorCapability toString (openEV::GliderVarioDriverBase::SensorCapability i) {
	openEV::GliderVarioDriverBase::_SensorCapability r = {i};
	return r;
}

inline std::ostream& operator << (std::ostream &o, openEV::GliderVarioDriverBase::_SensorCapability e) {
	return operator << (o,e.e);
}

#endif /* GLIDERVARIODRIVERBASE_H_ */

