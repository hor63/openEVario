/*
 * MPU9150Driver.h
 *
 *  Created on: Jun 07, 2021
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2021  Kai Horstmann
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

#ifndef MPU9150DRIVER_H_
#define MPU9150DRIVER_H_

#include <fstream>
#include <string>
#include <map>
#include <thread>
#include <chrono>

#include "OEVCommon.h"

#include "util/io/I2CPort.h"
#include "util/drivers/IMUBase.h"
#include "MPU-9150Defs.h"
#include "MPU-9150Lib.h"

namespace openEV::drivers::TDK_MPU9150 {

/** \brief Driver for TDK/Invensense MPU-9150 9D IMU
 *
 * This driver communicates with the sensor via I2C to obtain accelerometer, gyroscope, and magnetometer data.
 *
 */
class MPU9150Driver : public IMUBase {
public:

	/** \brief Union of unsigned and signed 16-bit integers to map one to the other
	 *
	 * The C++ specification leaves some conversions from signed to unsigned integer
	 * undefined or compiler dependent.
	 * To allow a penalty-free conversion from unsigned (registers) to signed (interpretation of some measurements)
	 * I am using this union.
	 * Alternatively you can also pointer casting.
	 */
	union UnionInt16{
			uint16_t uintVal;
			int16_t intVal;
		};

	MPU9150Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);

	virtual ~MPU9150Driver();

	/** \brief Read the configuration
     *
     * \see GliderVarioDriverBase::readConfiguration()
     */
    virtual void readConfiguration (Properties4CXX::Properties const &configuration) override;

protected:

	/** \brief Read a single bye using Slave 4 on the auxiliary I2C bus
	 *
	 * @param slaveDevAddr I2C address of the connected device. Typically \ref AK8975_I2CAddr
	 * @param regAddr Byte address of the register to read from
	 * @return The byte value of the register
	 */
    uint8_t readByteAux (uint8_t slaveDevAddr, uint8_t regAddr);

    /**
     *
	 * @param slaveDevAddr I2C address of the connected device. Typically \ref AK8975_I2CAddr
	 * @param regAddr Byte address of the register to read from
     * @param data The byte value to be written to the register
     */
    void writeByteAux (uint8_t slaveDevAddr, uint8_t regAddr, uint8_t data);

    void setupMPU9150();

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
    io::I2CPort *ioPort = nullptr;

    uint8_t i2cAddress = MPU_9150_I2CAddr;

    AK8975_mag_trim_registers trimRegisters;

    FloatType magFactorX = 1229.0f * 2.0f / (4095.0f + 4096.0f);
    FloatType magFactorY = 1229.0f * 2.0f / (4095.0f + 4096.0f);
    FloatType magFactorZ = 1229.0f * 2.0f / (4095.0f + 4096.0f);

    FloatType gyrFactor = 1.0f / 131.0f;
    FloatType accFactor = 1.0f / 8192.0f;



};

} /* namespace openEV */
#endif /* MPU9150DRIVER_H_ */

