/*
 * GliderVarioDriverBase.h
 *
 *  Created on: Oct 01, 2017
 *      Author: hor
 *
 *  Base class of driver classes for OpenEVario
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2022  Kai Horstmann
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
#include <chrono>

#include "CommonDefs.h"
#include "Properties4CXX/Properties.h"

#include "util/io/PortBase.h"
#include "kalman/GliderVarioStatus.h"
#include "kalman/GliderVarioMeasurementUpdater.h"
#include "drivers/DriverLibBase.h"
#include "main/GliderVarioMainPriv.h"

namespace openEV {

// Forward declaration
class GliderVarioMainPriv;

namespace drivers {


/** \brief Abstract base class for sensor drivers.
 *
 * Abstract base class of sensor drivers. Most methods are pure virtual, and must be implemented by the driver class
 * The actual driver implementation is never exposed to the main program but hidden in the shared library which
 * implements the driver
 */
class OEV_MAIN_PUBLIC DriverBase {

public:

    DriverBase (
    	    char const *driverName,
			char const *description,
			char const *instanceName,
			DriverLibBase &driverLib
			);

    virtual ~DriverBase () {}

    /// The list of sensor capabilities. Each one is a position in an integer.
    /// The capabilities are ORed into the capabilities of the driver in #sensorCapabilities.
#if defined DOXYGEN
    enum SensorCapability {
#else
    OEV_ENUM (SensorCapability,
#endif

        GPS_POSITION = 0,
        GPS_ALTITUDE_MSL = 1,
        GPS_HEADING = 2,
        GPS_SPEED = 3,
        ACCEL_3D = 4,
        GYRO_3D = 5,
        MAGNETOMETER_3D = 6,
        STATIC_PRESSURE = 7,
        DYNAMIC_PRESSURE = 8,

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

    DriverLibBase &GetDriverLib () {
    	return driverLib;
    }

    DriverLibBase const &GetDriverLib () const {
    	return driverLib;
    }

    /** \brief Set the update cycle of the driver instance
     *
     * @param updateCyle Cycle as OEVDuration
     *
     * \see updateCyle
     */
    void setUpdateCyle (OEVDuration updateCyle) {
    	this->updateCyle = updateCyle;
    }

    /** \brief Get the update cycle of the driver instance
     *
     * @return Update cycle time as OEVDuration
     *
     * \see updateCyle
     *
     */
    OEVDuration getUpdateCyle () {
    	return updateCyle;
    }

    // The abstract interface which must be implemented by each driver.

    /** \brief Initialize the driver
     *
     * This method is called between calls to \ref readConfiguration and \ref startup.
     * Most drivers will probably choose to do nothing here :)
     *
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
     */
    virtual void driverInit(GliderVarioMainPriv &varioMain) = 0;

    /** \brief Read the configuration values which are common for most or all drivers.
     *
     * @param configuration Sub-structure of the driver instance configuration
     */
    void readCommonConfiguration (Properties4CXX::Properties const &configuration);

    /** \brief Read the driver specific configuration for the driver
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
     * @param[in,out] varioStatus Status and Co-variance of the Kalman filter to be initialized.
	 * @param[in,out] measurements Current measurement vector.
	 * Used for cross-referencing measurements of other drivers during the initialization phase
	 * @param[in,out] varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
     */
    virtual void initializeStatus(
    		GliderVarioStatus &varioStatus,
			GliderVarioMeasurementVector &measurements,
			GliderVarioMainPriv &varioMain) = 0;

    /** \brief Start capturing data from the sensor, and updating the Kalman filter
     *
     * This call comes after \ref startup() and \ref initializeStatus().
     *
     * The standard implementation just sets \ref isKalmanUpdateRunning \p true.
     *
     * The standard implementation should suffice in most cases. It can be overridden if necessary
     *
     */
    virtual void run();

    /** \brief Stop capturing data from the sensor
     *
     * This calls returns when the internal thread is stopped and data capturing actually stopped.
     * Connections are closed when necessary.
     * This requires that the implementation of \ref driverThreadFunction regularly checks \ref isKalmanUpdateRunning and exits in time
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
     * The standard implementation just sets \ref isKalmanUpdateRunning \p false.
     *
     */
    virtual void suspend();

    /** \brief Data capturing and updating of the Kalman filter resumes
     *
     * When data capturing and Kalman filter updates were suspended before this call resumes Kalman filter updating.
     *
     *
     * The standard implementation just sets \ref isKalmanUpdateRunning \p true.
     *
     */
    virtual void resume();

    /** \brief Flag if sensor data should update the Kalman filter.
     *
     * If \p true the Kalman filter is being updated \n
     * If \p false the Kalman filter is not to be updated. The driver thread will keep running and acquire data.
     *
     * \see isKalmanUpdateRunning
     */
    bool getIsKalmanUpdateRunning() {
    	return isKalmanUpdateRunning;
    }

    /** \brief Communication flag if the driver thread is running
     *
     * When the flag is \p true the driver thread is running.
     *
     * \see isDriverThreadRunning
     */
    bool getIsDriverThreadRunning(){
    	return isDriverThreadRunning;
    }

    /** \brief Communication flag to the driver thread to shut itself down
     *
     * When the flag is \p true the driver thread will close the port and shut itself down.
     *
     * \see stopDriverThread
     */
    bool getStopDriverThread() {
    	return stopDriverThread;
    }

    /** \brief Little helper to reduce code size
     *
     * If the property \p parameterName does not exist in the properties set \p value will be unchanged.
     *
     * @param[in] calibrationDataParameters Properties which were read from the calibration parameter file
     * @param[in] parameterName Name of the calibration value
     * @param[in,out] value Value of the calibration value
     */
    static void readOrCreateConfigValue(
    		Properties4CXX::Properties* calibrationDataParameters,
    		char const* parameterName,
    		double& value
    		);

    /** \brief Little helper to reduce code size
     *
     * New values are written by an existing property when it exists, and write a new one.
     *
     * @param[in,out] calibrationDataParameters Properties which hold the calibration data
     * @param[in] parameterName Name of the calibration value
     * @param[in] value New value of the calibration value
     */
    static void writeConfigValue (
    		Properties4CXX::Properties* calibrationDataParameters,
    		char const* parameterName,
    		double value
    		);

    /** \brief Helper function to obtain the I/O port object of the correct class
     *
     * Template helper function to
     * - check if \ref portName was configured at all
     * - obtain the the pointer to an I/O port object named by \ref portName
     * - dynamically up-casting the base pointer type io::PortBase to the template pointer type \p t.
     *
     * If any of the activities listed fails an exception with diagnostic text is thrown.
     *
     * @tparam t Specialized class derived from base class \ref io::PortBase
     * @param logger Log4CPP logger object of the calling class
     * @return Pointer to the I/O port object of the class \p t. The pointer is never \p nullptr.
     *   In any case where the result would be \p nullptr an exception is thrown.
     */
    template <typename t>
    t getIoPort(log4cxx::LoggerPtr logger) {
    	t port;

    	try {
    		if (portName.empty()) {
    			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"\"portName\" was not configured.");
    		}

    		port = dynamic_cast<t>(io::PortBase::getPortByName(portName));

    		if (port == nullptr) {
				std::ostringstream str;

				str << "I/O port \"" << portName << "\" is not of type I2CPort. The type is \""
						<< typeid(t).name() << "\" instead.";
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
    		}

    	} catch (std::exception const& e) {
    		std::ostringstream str;
    		str << "I/O port configuration error for device \"" << instanceName
    				<< "\": " << e.what();
    		LOG4CXX_ERROR(logger, str.str());
    		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
    	}

    	return port;
    }

private:

    /** \brief Flag if sensor data should update the Kalman filter.
     *
     * If \p true the Kalman filter is being updated \n
     * If \p false the Kalman filter is not to be updated. The driver thread will keep running and acquire data.
     */
    volatile bool isKalmanUpdateRunning = false;

    /** \brief Communication flag if the driver thread is running
     *
     * When the flag is \p true the driver thread is running.
     */
    volatile bool isDriverThreadRunning = false;

    /** \brief Communication flag to the driver thread to shut itself down
     *
     * When the flag is \p true the driver thread will close the port and shut itself down.
     */
    volatile bool stopDriverThread = false;


protected:

    /// Bit list of capabilities. The bit positions are defined in the enum #SensorCapability.
    uint32_t sensorCapabilities = 0;

    std::string driverName;
    std::string description;
    std::string instanceName;

    DriverLibBase &driverLib;

    /// Pointer to the main object. Is being set by \ref startup() and set NULL by \ref shutdown()
    GliderVarioMainPriv *varioMain = 0;

    /// \brief The sensor driver thread
    std::thread driverThread;

    // == Common configuration parameters =========================
    /** \brief Name of the communications port.
     *
     * I/O ports are defined in the IOPorts section of the configuration
     */
    std::string portName;

    /** \brief The update cycle of the driver
     *
     * Most drivers are polling the sensor with an self-determined cycle.
     * Therefore define the cycle in the driver base, and try to read it when loading the driver
     * in \ref GliderVarioDriverList::loadDriverInstance()
     * Some drivers (e.g. NMEA GPS or the BMX160 sensor board receive data at the pace defined by the active sensor.
     * However define the cycle in the properties even for these drivers. I will use it to calculate the variance increment per cycle.
     * Particularly for GPS receivers the rate can be anywhere between 1 Hz and 20 Hz.
     */
    OEVDuration updateCyle = std::chrono::milliseconds(100);

    /** \brief Timeout in seconds between recovery attempts when an error in the main loop occurs.
     *
     * Configuration parameter is "errorTimeout" in the driver section.
     */
    OEVDuration errorTimeout = std::chrono::seconds(10);

    /** \brief Maximum number of retries upon consecutive errors in the main loop.
     *
     * A value <= 0 means that the number of retries is unlimited.
     *
     * When the maximum number of retries is exceeded the main loop terminates and the driver ceases to operate
     *
     * Configuration parameter is "errorMaxNumRetries" in the driver section.
     *
     */
    int32_t errorMaxNumRetries = 0;

    /// \brief Name of the initial calibration data parameter file
    std::string calibrationDataFileName;
    /** \brief Try to read calibration data from a file.
     *
     * Automatically true when calibrationDataFileName is not empty.
     */
    bool useCalibrationDataFile = false;

    /// \brief Name of the continuously updated calibration data parameter file
    std::string calibrationDataUpdateFileName;

    /// \brief Interval to save the continuously updated calibration data
    OEVClock::duration calibrationDataWriteInterval = OEVClock::duration(0);
    /** \brief Write updated calibration data to a file.
     *
     * Automatically true when calibrationDataUpdateFileName is not empty, and
     * calibrationDataWriteInterval is > 0.
     */
    bool useCalibrationDataUpdateFile = false;

    /// \brief Time of the last calibration data update, or the initial load
    OEVClock::time_point lastCalibrationDataWriteTime;
    /// \brief Flag indicating when the calibration data write thread is still active.
    volatile bool isCalibrationDataUpdateActive = false;

    /** \brief Load dynamic updated calibration date before static calibration data
	 *
     *  When both \ref calibrationDataFileName and \ref calibrationDataUpdateFileName are configured
     *  try loading the file named by calibrationDataUpdateFileName first. When that does not exist
     *  try loading the static file named by calibrationDataUpdateFileName.
     *  Else try loading the files in the opposite order.
     *
     */
    bool loadCalibrationDataUpdateFileBeforeStatic = true;

    /// Loaded and parsed calibration data
    Properties4CXX::Properties *calibrationDataParameters = nullptr;
    /** \brief Thread object for the calibration data writer thread
     *
     * Writing out the calibration data is a fairly time consuming I/O operation.
     * Therefore it is implemented as a one-shot thread which is re-started every
     * \ref calibrationDataUpdateCycle seconds.
     */
    std::thread calibrationDataWriteThread;

    /** \brief Indicator if the previous calibration write run is still active or finished.
     *
     * Indicator if the thread code actually ran to the end.
     * This prevents blocking the driver thread when it joins the last thread run.
     */
    volatile bool calibrationWriterRunning = false;


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
     * This function *must* regularly check if \ref stopDriverThread is set false, and then immediately exit.
     * This check must occur within a reasonable interval, typically < 1 sec
     * Internally it also must check \ref isKalmanUpdateRunning to determine if Kalman updates are suspended, or not yet
     * started.
     * Data acquisition should start immediately to be ready for answering \ref initializeStatus() ASAP.
     *
     */
    virtual void driverThreadFunction() = 0;

    /** \brief Static function required for std::thread implementation.
     *
     * @param tis Pointer to the object to which the thread belongs
     */
    static void driverThreadEntry (DriverBase* tis);

    /** \brief Read calibration data when configured
     *
     * When the update calibration file is defined, and initially loading is defined try to load it.
     * When loading of the update calibration data is not configured, or it does not exist
     * try loading the initial calibration file.
     * Then call \ref applyCalibrationData() which writes the calibration data into the object.
     */
    void readCalibrationData ();
    /** \brief Driver specific function to apply calibration data to the driver instance
     *
     * This function must be overloaded by the driver which uses calibration data storage.
     */
    virtual void applyCalibrationData();
    /** \brief Driver specific function to apply calibration data to the driver instance
     *
     * This function must be overloaded by the driver which uses calibration data storage.
     * The implementation in this call does nothing.
     */

    /** \brief Update the calibration data file when needed
     *
     * \todo Refactor function name
     * Check if the calibration cycle time expired,
     * and write updated calibration data in the file named \ref calibrationDataUpdateFileName
     */
    void updateCalibrationData();

    /** \brief Thread function of \ref calibrationDataWriteThread
     *
     * Analyze the current status.
     *   - When the variance of the gyro bias is smaller than the one of the calibration data update the gyro calibration data.
     *   - When the variance of the magnetic bias (incl. Variance) is smaller than the calibration data update the mag bias data.
     *   - Accelerometer calibration data are not being touched. I presume they are stable.
     *   There is also no accelerometer bias and factor in the model. The Gravity parameter in the model actually applies only to the
     *   Z axis.
     *
     * If any calibration data was updated write out the updated configuration back into the configuration parameter file.
     *
     * *Note*: This function runs in an own thread!
     *
     */
    void calibrationDataWriteFunc();

    /** \brief Fill calibration data parameter list driver specific
     *
     * Collect driver specific calibration data, and write them into \ref calibrationDataParameters.
     * The base class method does nothing. It must be overridden in a driver class.
     */
    virtual void fillCalibrationDataParameters ();

}; // class GliderVarioDriverBase

/// Define a shared pointer to the driver object which keeps a reference count
typedef std::shared_ptr<openEV::drivers::DriverBase> GliderVarioDriverBasePtr;

} // namespace drivers
} // namespace openEV

#include "main/GliderVarioDriverList.h"

OEV_UTILS_PUBLIC std::ostream& operator << (std::ostream &o,openEV::drivers::DriverBase::SensorCapability ind);
#if defined HAVE_LOG4CXX_H
OEV_UTILS_PUBLIC std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::drivers::DriverBase::SensorCapability v);
#endif

#endif /* GLIDERVARIODRIVERBASE_H_ */

