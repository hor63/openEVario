/*
 * GliderVarioDriverBase.h
 *
 *  Created on: Oct 01, 2017
 *      Author: hor
 *
 *  Base class of driver classes for OpenEVario
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2017  Kai Horstmann
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
#include <memory>
#include <string>
#include <list>
#include <memory>

#include "Properties4CXX/Properties.h"


#include "OEVCommon.h"
#include "kalman/GliderVarioStatus.h"
#include "kalman/GliderVarioMeasurementUpdater.h"
#include "drivers/GliderVarioDriverLibBase.h"

namespace openEV {

// Forward declaration
class GliderVarioMainPriv;

/** \brief Abstract base class for sensor drivers.
 *
 * Abstract base class of sensor drivers. Most methods are pure virtual, and must be implemented by the driver class
 * The actual driver implementation is never exposed to the main program but hidden in the shared library which
 * implements the driver
 */
class OEV_UTILS_PUBLIC GliderVarioDriverBase {

public:

    GliderVarioDriverBase (
    	    char const *driverName,
			char const *description,
			char const *instanceName,
			GliderVarioDriverLibBase &driverLib
			)
    : sensorCapabilities {0},
	  driverName {driverName},
	  description {description},
	  instanceName {instanceName},
	  driverLib {driverLib}
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

		/** \brief This driver will run the idle loop too. The main program will not run the idle loop itself
		 *
		 * This is a capability mainly for simulation, test and debug drivers.
		 *
		 * "Normal" sensor drivers will update the Kalman filter status only when new measurement values arrive.
		 * The main program runs an independent thread which will run every 'idlePredictionCycle' ms, and runs a Kalman filter prediction cycle
		 * when no new measurements force a prediction cycle in between.
		 * A driver with this capability will prevent the main program to start the idle thread because this driver implements also the idle thread.
		 * The purpose is for debugging and testing because the main part of the Kalman filter runs single-threaded.
		 *
		 */
		RUN_IDLE_LOOP = 16,
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

    char const * getDriverName () const {
    	return driverName.c_str();
    }
    char const * getDescription () const {
    	return description.c_str();
    }
    char const * getInstanceName () const {
    	return instanceName.c_str();
    }

    GliderVarioDriverLibBase &GetDriverLib () {
    	return driverLib;
    }

    GliderVarioDriverLibBase const &GetDriverLib () const {
    	return driverLib;
    }

    // start of the abstract interface which must be implemented by each driver.

    /**
     * Initialize the driver
     */
    virtual void driverInit() = 0;

    /** \brief Read the configuration for the driver
     *
     * @param configuration Sub-structure of the driver instance configuration
     */
    virtual void readConfiguration (Properties4CXX::Properties const &configuration) = 0;

    /** \brief Initialize the status components which can be directly initialized from sensor values.
     *
     * Open and run the sensor to initialize the vario status. Initialize the components of the status which can be directly initialized,
     * e.g. the position and altitude from initial GPS readings, or the altitude from the pressure sensor.
     * In addition derived status values like the QFF can be initialized from the GPS altitude, and pressure altitude once both are known.
     *
     * It is in the discretion of the driver if it leaves the sensor connection open or it closes it until \ref start() is called.
     *
     * This call is synchronous. It should return ASAP in order to not make other initializations of the status invalid.
     *
     * @param varioStatus Status and Co-variance of the Kalman filter to be initialized.
     */
    virtual void initializeStatus(GliderVarioStatus &varioStatus) = 0;

    /** \brief Start capturing data from the sensor, and updating the Kalman filter
     *
     * This call starts the internal thread of the driver instance and returns immediately.
     *
     * @param varioMain Pointer to the vario main object
     */
    virtual void start(GliderVarioMainPriv *varioMain) = 0;

    /** \brief Stop capturing data from the sensor
     *
     * This calls returns when the internal thread is stopped and data capturing actually stopped.
     * Connections are closed when necessary.
     *
     */
    virtual void stop() = 0;

    /** \brief Suspend data capturing temporarily
     *
     * If a data capturing and Kalman update cycle is underway it will complete even if this function alreadz returned.
     * It is in the discretion of the driver if data capturing continues (e.g. capturing GPS data).
     *
     * In any case updating of the Kalman filter is suspended.
     */
    virtual void suspend() = 0;

    /** \brief Data capturing and updating of the Kalman filter resumes
     *
     * When data capturing and Kalman filter updates were suspended before this call resumes operations.
     *
     */
    virtual void resume() = 0;

    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) = 0;

protected:

    /// Set the capabilities of the sensor in #sensorCapabilities.
    /// To be set by the implementing class
    inline void setSensorCapabilities (uint32_t sensorCapabilities) {
        this->sensorCapabilities = sensorCapabilities;
    }

    /// Bit list of capabilities. The bit positions are defined in the enum #SensorCapability.
    uint32_t sensorCapabilities;

    std::string driverName;
    std::string description;
    std::string instanceName;

    GliderVarioDriverLibBase &driverLib;

    /// Pointer to the main object. Is being set by \ref start() and set NULL by \ref stop()
    GliderVarioMainPriv *varioMain = 0;

    // Abstract functions allowing sub-classing of the driver



}; // class GliderVarioDriverBase

/// Define a shared pointer to the driver object which keeps a reference count
typedef std::shared_ptr<openEV::GliderVarioDriverBase> GliderVarioDriverBasePtr;

}

#include "main/GliderVarioDriverList.h"

OEV_UTILS_PUBLIC std::ostream& operator << (std::ostream &o,openEV::GliderVarioDriverBase::SensorCapability ind);


#if defined HAVE_LOG4CXX_H
OEV_UTILS_PUBLIC std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioDriverBase::_SensorCapability e);
#endif /* #if defined HAVE_LOG4CXX_H */

inline openEV::GliderVarioDriverBase::_SensorCapability toString (openEV::GliderVarioDriverBase::SensorCapability i) {
	openEV::GliderVarioDriverBase::_SensorCapability r = {i};
	return r;
}

inline std::ostream& operator << (std::ostream &o, openEV::GliderVarioDriverBase::_SensorCapability e) {
	return operator << (o,e.e);
}

#endif /* GLIDERVARIODRIVERBASE_H_ */

