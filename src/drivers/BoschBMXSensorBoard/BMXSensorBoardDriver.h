/*
 * BMXSensorBoardDriver.h
 *
 *  Created on: Feb 04, 2020
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

#ifndef BMXSENSORBOARDDRIVER_H_
#define BMXSENSORBOARDDRIVER_H_

#include <fstream>
#include <string>
#include <map>

#include "OEVCommon.h"

// Shared structures and constants with the sensor board MCU firmware.
#include "horOvIp-I2C-Bridge/BMX160net.h"

#include "drivers/GliderVarioDriverBase.h"
#include "BMXSensorBoardLib.h"
#include "util/io/StreamPort.h"

namespace openEV {

/** \brief Driver for Bosch BMX160 IMU which is mounted on the hovImuBoard sensor board
 *
 * This driver communicates with the BMX SensorBoard to obtain accelerometer, gyroscope, and magnetometer.
 *
 * The hardware of sensor board is available as KiCad schematics and BOM in my GitHub repository [hovImuBoard](https://github.com/hor63/hovImuBoard)
 * The firmware for the AVR micro controller on the board is in my GitHub repository [horOvIp-I2C-Bridge](https://github.com/hor63/horOvIp-I2C-Bridge)
 *
 * Communications with the sensor board is via TCP/IP. The board connects to the computer via serial line using SLIP.
 *
 */
class BMXSensorBoardDriver  : public GliderVarioDriverBase {
public:

	BMXSensorBoardDriver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~BMXSensorBoardDriver();

    /** \brief Initialize the driver
     *
     * \see GliderVarioDriverBase::driverInit()
     */
    virtual void driverInit() override;


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

    /** \brief Start data acquisition
     *
     * \see GliderVarioDriverBase::start()
     */
    void run(GliderVarioMainPriv &varioMain) override;

    /** \brief Suspend the driver temporarily
     *
     * \see GliderVarioDriverBase::suspend()
     */
    virtual void suspend() override;

    /** \brief Resume data acquisition when it was suspended before by suspend()
     *
     * \see GliderVarioDriverBase::resume()
     */
    virtual void resume() override;

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

private:

    std::string portName;

    /// \brief The I/O port. Typically this is a TCP port.
    io::StreamPort *ioPort = nullptr;

    /// BMX160 magnetometer trim data structure
    struct bmm150_trim_registers magTrimData;

};

} /* namespace openEV */
#endif /* BMXSENSORBOARDDRIVER_H_ */

