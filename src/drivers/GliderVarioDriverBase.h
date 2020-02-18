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
#include <thread>

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
			);

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

    // The abstract interface which must be implemented by each driver.

    /**
     * Initialize the driver
     */
    virtual void driverInit() = 0;

    /** \brief Read the configuration for the driver
     *
     * @param configuration Sub-structure of the driver instance configuration
     */
    virtual void readConfiguration (Properties4CXX::Properties const &configuration) = 0;

    /** \brief todo Don't know if I really need this...
     *
     * @param varioStatus The current Kalman status to update
     */
    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) = 0;

    // Interface with default implementations by the base class

    /** \brief Startup the driver thread
     *
     * This call starts the internal thread of the driver instance and returns immediately.
     *
     * The standard implementation should suffice in most cases. It can be overridden if necessary.
     * Usually the driver thread will open the communication port, and start acquiring data.
     *
     * The initial data can be used to acquire an initial status for use with \ref initializeStatus().
     *
     * When \ref run() starts updating the Kalman filter the thread is already running.
     *
     * @param varioMain Reference to the vario main object
     * @see run()
     * @see stop()
     */
    virtual void startup(GliderVarioMainPriv &varioMain);

    /** \brief Initialize the status components which can be directly initialized from sensor values.
     *
     * Run the sensor to initialize the vario status. Initialize the components of the status which can be directly initialized,
     * e.g. the position and altitude from initial GPS readings, or the altitude from the pressure sensor.
     * In addition derived status values like the QFF can be initialized from the GPS altitude, and pressure altitude once both are known.
     *
     * It is in the discretion of the driver if it leaves the sensor connection open or it closes it until \ref run() is called.
     *
     * This call is synchronous. It should return ASAP in order to not make other initializations of the status invalid.
     *
     * This call is invoked after \ref startup(). So you can expect that the driver thread is up and running already.
     *
     * @param varioStatus Status and Co-variance of the Kalman filter to be initialized.
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
     */
    virtual void initializeStatus(
    		GliderVarioStatus &varioStatus,
			GliderVarioMainPriv &varioMain) = 0;

    /** \brief Start capturing data from the sensor, and updating the Kalman filter
     *
     * This call comes after \ref startup() and \ref initializeStatus().
     *
     * The standard implementation just sets \ref isRunning \p true.
     *
     * The standard implementation should suffice in most cases. It can be overridden if necessary
     *
     * @param varioMain Reference to the vario main object
     */
    virtual void run(GliderVarioMainPriv &varioMain);

    /** \brief Stop capturing data from the sensor
     *
     * This calls returns when the internal thread is stopped and data capturing actually stopped.
     * Connections are closed when necessary.
     * This requires that the implementation of \ref driverThreadFunction regularly checks \ref isRunning and exits in time
     * Otherwise this function would be stuck forever.
     *
     * The standard implementation should suffice in most cases. It can be overridden if necessary
     *
     */
    virtual void shutdown();

    /** \brief Suspend data capturing temporarily
     *
     * If a data capturing and Kalman update cycle is underway it will complete even if this function already returned.
     * It is in the discretion of the driver if data capturing continues (e.g. capturing GPS data).
     *
     * In any case updating of the Kalman filter is suspended.
     *
     * The standard implementation just sets \ref isRunning \p false.
     *
     */
    virtual void suspend();

    /** \brief Data capturing and updating of the Kalman filter resumes
     *
     * When data capturing and Kalman filter updates were suspended before this call resumes Kalman filter updating.
     *
     *
     * The standard implementation just sets \ref isRunning \p true.
     *
     */
    virtual void resume();

protected:

    /// Bit list of capabilities. The bit positions are defined in the enum #SensorCapability.
    uint32_t sensorCapabilities = 0;

    std::string driverName;
    std::string description;
    std::string instanceName;

    GliderVarioDriverLibBase &driverLib;

    /// Pointer to the main object. Is being set by \ref start() and set NULL by \ref stop()
    GliderVarioMainPriv *varioMain = 0;

    /** \brief Flag if sensor data should update the Kalman filter.
     *
     * If \p true the Kalman filter is being updated \n
     * If \p false the Kalman filter is not to be updated. The driver thread will keep running and acquire data.
     */
    bool isKalmanUpdateRunning = false;

    /** \brief Communication flag to the driver thread to shut itself down
     *
     * When the flag is \p true the driver thread will close the port and shut itself down.
     */
    bool isDriverThreadRunning = false;

    /** \brief Communication flag to the driver thread to shut itself down
     *
     * When the flag is \p true the driver thread will close the port and shut itself down.
     */
    bool stopDriverThread = false;

    /// \brief The sensor driver thread
    std::thread driverThread;

    /// Set a driver capability. Capabilities are defined in #SensorCapability.
    inline void setSensorCapability (SensorCapability capability) {
        sensorCapabilities |= (1UL<<capability);
    }

    /// Clear a driver capability. Capabilities are defined in #SensorCapability.
    inline void clearSensorCapability (SensorCapability capability) {
        sensorCapabilities &= ~(1UL<<capability);
    }

    // Abstract functions allowing sub-classing of the driver

    /** \brief The main worker thread of the sensor driver
     *
     * Must be overridden by the driver implementation
     *
     * This function *must* regularly check if \ref isKalmanUpdateRunning is set false, and then immediately exit.
     * This check must occur within a reasonable interval, typically < 1 sec
     */
    virtual void driverThreadFunction() = 0;

    /** \brief Static function required for std::thread implementation.
     *
     * @param tis Pointer to the object to which the thread belongs
     */
    static void driverThreadEntry (GliderVarioDriverBase* tis);


}; // class GliderVarioDriverBase

/// Define a shared pointer to the driver object which keeps a reference count
typedef std::shared_ptr<openEV::GliderVarioDriverBase> GliderVarioDriverBasePtr;

}

#include "main/GliderVarioDriverList.h"

OEV_UTILS_PUBLIC std::ostream& operator << (std::ostream &o,openEV::GliderVarioDriverBase::SensorCapability ind);

#endif /* GLIDERVARIODRIVERBASE_H_ */

