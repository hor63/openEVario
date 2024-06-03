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

#include "CommonDefs.h"
#include "util/io/I2CPort.h"
#include "main/driverBase/IMUBase.h"
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

	struct MeasurementData {
		UnionInt16 accelX;
		UnionInt16 accelY;
		UnionInt16 accelZ;
		UnionInt16 tempRaw;
		UnionInt16 gyroX;
		UnionInt16 gyroY;
		UnionInt16 gyroZ;
		uint8_t    magStatus1;
		uint8_t    magStatus2;
		UnionInt16 magX;
		UnionInt16 magY;
		UnionInt16 magZ;
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

    /** \brief Initialize the driver
	 *
	 * Obtains the I/O port pointer.
	 * The remainder is handled by calling \ref IMUBase::driverInit()
     *
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
	 * \see IMUBase::driverInit()
     * \see GliderVarioDriverBase::driverInit()
     */
    virtual void driverInit(GliderVarioMainPriv &varioMain) override;

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

    /** \brief Setup the entire MPU-9150
     *
     * Setup the accels to +-4g
     * Setup the gyros to +-250deg/s
     * Setup the low-pass filter to 20Hz, and 8.5ms delay.
     * Setup the automatic cycle to 20ms/50Hz.
     * Finally call \ref setupAK8975Mag() to setup the magnetometer.
     * on the aux. I2C bus, and automatically initiate a measurement cycle,
     * and collect the measurement data via the aux. bus from the AK8975Mag
     * into the internal measurement registers of the MPU-9150.
     */
    void setupMPU9150();

    /** \brief Setup the AK8975 magnetometer.
     *
     * Setup the magnetometer on the aux. I2C bus.
     * Automatically initiate a measurement cycle,
     * and collect the measurement data via the aux. bus from the AK8975Mag
     * into the internal measurement registers of the MPU-9150.
     * Use Slave 0 to receiving measurement data,
     * and use Slave 1 initiate the next measurement during the upcoming cycle.
     * Slave 0 and 1 run sequentially with the main cycle time of 20 ms.
     *
     */
    void setupAK8975Mag();

    /** \brief The main loop of the driver after the port was opened
     *
     * Read data from the sensor, process them, and update the Kalman filter.
     */
    virtual void processingMainLoop () override;

    virtual io::PortBase* getIoPortPtr() override;

private:

    /// \brief The I/O port. Typically this is a TCP port.
    io::I2CPort *ioPort = nullptr;

    uint8_t i2cAddress = MPU_9150_I2CAddr;

    AK8975_mag_trim_registers trimRegisters;

    FloatType magFactorX = 1229.0f * 2.0f / (4095.0f + 4096.0f);
    FloatType magFactorY = 1229.0f * 2.0f / (4095.0f + 4096.0f);
    FloatType magFactorZ = 1229.0f * 2.0f / (4095.0f + 4096.0f);

    /** \brief Conversion factor of gyroscope raw register values
     *
     * Factor is valid for a gyroscope sensitivity of +-250 deg/s
     */
    FloatType gyrFactor = 1.0f / 131.0f;

    /** \brief Conversion factor of accelerometer raw register values
     *
     * Factor is valid for a accelerometer sensitivity of +-4 g
     */
    FloatType accFactor = 1.0f / 8192.0f;

	MeasurementData measurementData;


};

} /* namespace openEV */
#endif /* MPU9150DRIVER_H_ */

