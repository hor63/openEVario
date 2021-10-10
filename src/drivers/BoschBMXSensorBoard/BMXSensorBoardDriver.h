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

// Shared structures and constants with the sensor board MCU firmware.
#include "horOvIp-I2C-Bridge/BMX160net.h"

#include "drivers/DriverBase.h"
#include "util/drivers/IMUBase.h"
#include "BMXSensorBoardLib.h"
#include "util/io/DatagramPort.h"

namespace openEV::drivers::BoschBMX160 {

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
class BMXSensorBoardDriver  : public IMUBase {
public:

	static constexpr float BMM150_OVERFLOW_OUTPUT_FLOAT = 1.0e10f;

	BMXSensorBoardDriver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~BMXSensorBoardDriver();


    /** \brief Read the configuration
     *
     * \see GliderVarioDriverBase::readConfiguration()
     */
    virtual void readConfiguration (Properties4CXX::Properties const &configuration) override;

protected:


    /** \brief The main worker thread of this driver
     *
     * \see GliderVarioDriverBase::driverThreadFunction()
     *
     */
    virtual void driverThreadFunction() override;

    /** \brief The main loop of the driver after the port was opened
     *
     * Read data from the sensor, process them, and update the Kalman filter.
     */
    virtual void processingMainLoop ();

private:

    std::string portName;

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

    /// \brief The I/O port. Typically this is a TCP port.
    io::DatagramPort *ioPort = nullptr;

    /// \brief BMX160 magnetometer trim data structure
    struct bmm150_trim_registers magTrimData;

    /**
     * @brief This internal API is used to obtain the compensated
     * magnetometer x axis data(micro-tesla) in float.
     */
    float compensate_x(int16_t mag_data_x, uint16_t data_rhall);

    /**
     * @brief This internal API is used to obtain the compensated
     * magnetometer y axis data(micro-tesla) in float.
     */
    float compensate_y(int16_t mag_data_y, uint16_t data_rhall);

    /*!
     * @brief This internal API is used to obtain the compensated
     * magnetometer z axis data(micro-tesla) in float.
     */
    float compensate_z(int16_t mag_data_z, uint16_t data_rhall);

};

} /* namespace openEV */
#endif /* BMXSENSORBOARDDRIVER_H_ */

