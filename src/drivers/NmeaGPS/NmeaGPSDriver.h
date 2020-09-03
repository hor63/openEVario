/*
 * NmeaGPSDriver.h
 *
 *  Created on: Aug 13, 2020
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2020  Kai Horstmann
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

#ifndef NMEAGPSDRIVER_H_
#define NMEAGPSDRIVER_H_

#include <fstream>
#include <string>
#include <map>

#include "OEVCommon.h"

#include "drivers/GliderVarioDriverBase.h"
#include "NmeaGPSLib.h"

#include "util/io/StreamPort.h"

namespace openEV {

/** \brief Driver for Bosch BMX160 IMU which is mounted on the hovImuBoard sensor board
 *
 * This driver communicates with the BMX SensorBoard to obtain accelerometer, gyroscope, and magnetometer
 */
class NmeaGPSDriver  : public GliderVarioDriverBase {
public:

	NmeaGPSDriver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~NmeaGPSDriver();

    /** \brief Initialize the driver
     *
     * \see GliderVarioDriverBase::driverInit()
     */
    virtual void driverInit(GliderVarioMainPriv &varioMain) override;


    /** \brief Read the configuration
     *
     * \see GliderVarioDriverBase::readConfiguration()
     */
    virtual void readConfiguration (Properties4CXX::Properties const &configuration) override;

    /** \brief Initialize the Kalman filter status from initial sensor measurements
     *
     * \see GliderVarioDriverBase::initializeStatus()
     */
    virtual void initializeStatus(
    		GliderVarioStatus &varioStatus,
			GliderVarioMainPriv &varioMain) override;

    /** \brief Callback to update the Kalman filter status based on received data.
     *
     * \see GliderVarioDriverBase::updateKalmanStatus()
     */
    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) override;

protected:


    /** \brief The main worker thread of this driver
     *
     * \see GliderVarioDriverBase::driverThreadFunction()
     *
     */
    void driverThreadFunction() override;

    /** \brief The main loop of the driver after the port was opened
     *
     * Read data from the sensor, process them, and update the Kalman filter.
     */
    virtual void processingMainLoop ();

private:

    /** \brief Name of the communications port.
     *
     * I/O ports are defined in the IOPorts section of the configuration
     */
    std::string portName;

    /** \brief Timeout in seconds between recovery attempts when an error in the main loop occurs.
     *
     * Configuration parameter is "errorTimeout" in the driver section.
     */
    int32_t errorTimeout = 10;

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

    /// \brief The I/O port.
    ///
    /// Typically this is a serial port, either real RS-232, or Serial via USB or Bluetooth SPP. \n
    /// TCP will work either.
    io::StreamPort *ioPort = nullptr;


};

} /* namespace openEV */
#endif /* NMEAGPSDRIVER_H_ */

