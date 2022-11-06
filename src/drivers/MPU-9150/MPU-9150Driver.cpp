/*
 * MPU9150Driver.cpp
 *
 *  Created on: Jun 07, 2021
 *      Author: kai_horstmann
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

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <fstream>
#include <chrono>

#include "Properties4CXX/Property.h"

#include "MPU-9150Driver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"
#include "util/RotationMatrix.h"

namespace openEV::drivers::TDK_MPU9150 {

MPU9150Driver::MPU9150Driver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: IMUBase {driverName,description,instanceName,MPU9150Lib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.MPU9150");
	}
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(ACCEL_3D);
	setSensorCapability(GYRO_3D);
	setSensorCapability(MAGNETOMETER_3D);

	// Default cycle time as documented in the template parameter file
	using namespace std::chrono_literals;
	updateCyle = 100ms;

}


MPU9150Driver::~MPU9150Driver() {
}



void MPU9150Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_DEBUG(logger,"Driver" << driverName << " read configuraion");


	try {
		auto portNameConfig = configuration.searchProperty("portName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"PortName\" is a struct or a string list.");
		}

		portName = portNameConfig->getStringValue();

		ioPort = dynamic_cast<io::I2CPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not a stream port.");
		}
	} catch (std::exception const& e) {
		LOG4CXX_ERROR(logger, "Read configuration of driver \"" << driverName
				<< "\" failed:"
				<< e.what());
		throw;
	}

    errorTimeout = configuration.getPropertyValue(std::string("errorTimeout"),(long long)(10));
    errorMaxNumRetries = configuration.getPropertyValue(std::string("errorMaxNumRetries"),(long long)(0));

	i2cAddress = (long long)(configuration.getPropertyValue(
	    		std::string("i2cAddress"),
				(long long)(i2cAddress)));


}

uint8_t MPU9150Driver::readByteAux (uint8_t slaveDevAddr, uint8_t regAddr) {
	log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("readByteAux");
	uint8_t buf[5];

	// Slave 4 register set start
	buf[0] = REG_9150_I2C_SLV4_ADDR;

	buf[1] = slaveDevAddr |
			I2C_SLV4_RW // Read from the AK8975 on the Aux bus
			;
	// I2C_SLV4_REG: The register address on the slave device
	buf[2] = regAddr;
	// I2C_SLV4_DO: Data out (unused)
	buf[3] = 0;
	// I2C_SLV4_CTRL: Activate the slave interface transaction
	buf[4] = 0
			| I2C_SLV4_EN
			// | I2C_SLV4_INT_EN No interrupts used
			// | I2C_SLV4_REG_DIS Use the register addressing mode
			;

	ioPort->writeBlock(i2cAddress, buf, 5);

	// wait for the transaction to complete
	for (int i = 0; i < 100; ++i) {
		// Read I2C_SLV4_CTRL and I2C_SLV4_DI at once
		ioPort->readBlockAtRegAddrByte(i2cAddress, REG_9150_I2C_SLV4_CTRL,buf,2);
		LOG4CXX_TRACE(logger,"Cycle #" << i << ": REG_9150_I2C_SLV4_CTRL = 0x" << std::hex << uint32_t(buf[0]) << std::dec);

		// check if the enable flag was cleared, i.e the transaction finished.
		if ((buf[0] & I2C_SLV4_EN) == 0) {
			break;
		}
	}

	// buf[1] already contains Data-IN from the reading in the loop.
	LOG4CXX_TRACE(logger,"REG_9150_I2C_SLV4_DI = 0x" << std::hex << uint32_t(buf[1]) << std::dec);

	return buf[1];
}

// Use slave 4 for single transactions
void MPU9150Driver::writeByteAux (uint8_t slaveDevAddr, uint8_t regAddr, uint8_t data) {
	log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("writeByteAux");
	uint8_t buf[5];

	// Slave 4 register set start
	buf[0] = REG_9150_I2C_SLV4_ADDR;
	// write to the AK8975 on the Aux bus (read bit not set, i.e. write)
	buf[1] = slaveDevAddr
			// | I2C_SLV4_RW // Write to the AK8975 on the Aux bus
			;
	// I2C_SLV4_REG: The register address on the slave device
	buf[2] = regAddr;
	// I2C_SLV4_DO: Data out, byte to be written
	buf[3] = data;
	// I2C_SLV4_CTRL: Activate the slave interface transaction
	buf[4] = 0
			| I2C_SLV4_EN
			// | I2C_SLV4_INT_EN No interrupts used
			// | I2C_SLV4_REG_DIS Use the register addressing mode
			;

	ioPort->writeBlock(i2cAddress, buf, 5);

	// wait for the transaction to complete
	for (int i = 0; i < 1000; ++i) {
		// Read I2C_SLV4_CTRL
		ioPort->readBlockAtRegAddrByte(i2cAddress, REG_9150_I2C_SLV4_CTRL,buf,1);
		LOG4CXX_TRACE(logger,"Cycle #" << i << ": REG_9150_I2C_SLV4_CTRL = 0x" << std::hex << uint32_t(buf[0]) << std::dec);

		// check if the enable flag was cleared, i.e the transaction finished.
		if ((buf[0] & I2C_SLV4_EN) == 0) {
			break;
		}
	}

}

void MPU9150Driver::setupMPU9150() {
	using namespace std::literals::chrono_literals;
	uint8_t buf[8] = {0};

	// Reset the entire device
	buf[0] = REG_9150_PWR_MGMT_1;
	buf[1] = PWR_MGMT_DEVICE_RESET;
	LOG4CXX_DEBUG(logger,"Write REG_9150_PWR_MGMT_1 reset");
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Wait a bit letting the sensor run through the reset routine.
	std::this_thread::sleep_for(100ms);

	// Clear the sleep flag and all other flags.
	buf[0] = REG_9150_PWR_MGMT_1;
	buf[1] = 0;
	LOG4CXX_DEBUG(logger,"Write REG_9150_PWR_MGMT_1 Clear sleep flag");
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Wait a bit letting the gyros starting up.
	std::this_thread::sleep_for(100ms);

	buf[0] = ioPort->readByteAtRegAddrByte(i2cAddress, REG_9150_WHO_AM_I);

	// Who am I is the I2C adress (ignoring bit 0 which can be set with pin AD0 (Pin 9))
	if ((i2cAddress & ~1) == buf[0]) {
		LOG4CXX_INFO(logger,"WHO AM I contains expected 0x" << std::hex << uint32_t(buf[0]) << std::dec);
	} else {
		LOG4CXX_WARN(logger,"WHO AM I contains unexpected 0x" << std::hex << uint32_t(buf[0])
				<< ". Expected was 0x" << (i2cAddress & ~1)
				<< std::dec);
	}

	// Disable Gyro self-test, and set the full-scale range to +-250deg/sec
	buf[0] = REG_9150_GYRO_CONFIG;
	buf[1] = GYRO_RANGE_250;
	LOG4CXX_DEBUG(logger,"Write REG_9150_GYRO_CONFIG");
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Disable Accel self-test, and set the full-scale range to +-4g
	buf[0] = REG_9150_ACCEL_CONFIG;
	buf[1] = ACCEL_RANGE_4G;
	LOG4CXX_DEBUG(logger,"Write REG_9150_ACCEL_CONFIG");
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Disable external frame synchronization,
	// and set the DLPF filter to 8.5ms delay and 20Hz bandwidth
	buf[0] = REG_9150_CONFIG;
	buf[1] = DLPF_20HZ;
	LOG4CXX_DEBUG(logger,"Write REG_9150_CONFIG DLPF filter");
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Setup the sample rate to 20ms. The Gyro rate will be 1kHz because I just activated the DLPF.
	buf[0] = REG_9150_SMPLRT_DIV;
	buf[1] = 19; // + 1 = 20ms = 50Hz.
	LOG4CXX_DEBUG(logger,"Write REG_9150_SMPLRT_DIV");
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Set the clock source to the X-Gyro via PLL.
	buf[0] = REG_9150_PWR_MGMT_1;
	buf[1] = 0
			// | PWR_MGMT_DEVICE_RESET
			// | PWR_MGMT_SLEEP
			// | PWR_MGMT_CYCLE
			// | PWR_MGMT_DISABLE_TEMP
			| CLKSEL_PLL_X_GYR
			;
	LOG4CXX_DEBUG(logger,"Write REG_9150_PWR_MGMT_1 clock source X-Gyro");
	ioPort->writeBlock(i2cAddress, buf, 2);

	ioPort->readBlockAtRegAddrByte(i2cAddress, REG_9150_TEMP_OUT_H , buf, 2);
	UnionInt16 tempRaw;

	tempRaw.uintVal = (uint16_t(buf[0]) << 8) | uint16_t(buf[1]);
	LOG4CXX_DEBUG(logger,"Temperature raw " << tempRaw.intVal << " = " << (FloatType(tempRaw.intVal)/340.0f + 35.0f));

	setupAK8975Mag();
}

void MPU9150Driver::setupAK8975Mag() {
	using namespace std::literals::chrono_literals;
	uint8_t buf[8] = {0};

	// Disable the aux I2C pass-through
	buf[0] = REG_9150_INT_PIN_CFG;
	buf[1] = 0
			// | I2C_BYPASS_EN
			;
	LOG4CXX_DEBUG(logger,"Write REG_9150_INT_PIN_CFG; disable I2C aux pass-through");
	ioPort->writeBlock(i2cAddress, buf, 2);
	std::this_thread::sleep_for(10ms);

	// enable the aux master controller
	buf[0] = REG_9150_USER_CTRL;
	buf[1] = I2C_MST_EN;
	LOG4CXX_DEBUG(logger,"Write REG_9150_USER_CTRL; enable I2C aux master controller");
	ioPort->writeBlock(i2cAddress, buf, 2);

	std::this_thread::sleep_for(10ms);


	buf[0] = readByteAux (AK8975_I2CAddr,REG_AK8975_WIA);
	buf[1] = readByteAux (AK8975_I2CAddr,REG_AK8975_INFO);
	LOG4CXX_INFO(logger,"AK8975 via Aux: WhoAmI = 0x" << std::hex << uint16_t(buf[0])
			<< ", Info = 0x" << uint16_t(buf[1])
			<< std::dec);

	// Enable prom read mode
	writeByteAux(AK8975_I2CAddr,REG_AK8975_CNTL,AK8975_PROM_READ);
	std::this_thread::sleep_for(10ms);

	trimRegisters.asa_x = readByteAux (AK8975_I2CAddr,REG_AK8975_ASAX);
	LOG4CXX_DEBUG(logger,"AK8975 via Aux: ASAX = 0x" << std::hex << uint16_t(trimRegisters.asa_x)
			<< std::dec);
	trimRegisters.asa_y = readByteAux (AK8975_I2CAddr,REG_AK8975_ASAY);
	LOG4CXX_DEBUG(logger,"AK8975 via Aux: ASAY = 0x" << std::hex << uint16_t(trimRegisters.asa_y)
			<< std::dec);
	trimRegisters.asa_z = readByteAux (AK8975_I2CAddr,REG_AK8975_ASAZ);
	LOG4CXX_DEBUG(logger,"AK8975 via Aux: ASAZ = 0x" << std::hex << uint16_t(trimRegisters.asa_z)
			<< std::dec);

	// Calculate the magnetometer factors adjusted by the factory trim factors
	LOG4CXX_DEBUG(logger,"Unadjusted Mag conversion  = " << magFactorX);
	magFactorX *= FloatType(int(trimRegisters.asa_x) - 128) / 256.0f + 1.0f;
	magFactorY *= FloatType(int(trimRegisters.asa_y) - 128) / 256.0f + 1.0f;
	magFactorZ *= FloatType(int(trimRegisters.asa_z) - 128) / 256.0f + 1.0f;

	LOG4CXX_DEBUG(logger,"Adjusted Mag conversion  = " << magFactorX << ", " << magFactorY << ", " << magFactorZ);

	// Set back to power-down mode
	writeByteAux(AK8975_I2CAddr,REG_AK8975_CNTL,AK8975_PWR_DOWN);
	std::this_thread::sleep_for(10ms);

	// Start single measurement mode
	writeByteAux(AK8975_I2CAddr,REG_AK8975_CNTL,AK8975_SINGLE_MEAS);

	// Enable the data ready interrupt
	buf[0] = REG_9150_INT_ENABLE;
	buf[1] = 1;
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Delay the data ready interrupt until the external data (i.e. magnetometer) are also received.
	buf[0] = REG_9150_I2C_MST_CTRL;
	buf[1] = WAIT_FOR_ES // WAIT_FOR_ES: DATA_RDY is not raised before external data arrived
			| I2C_MST_P_NSR // Always send a STOP between aux master transactions
			; // I2C master clock divider remains 0 = 348 kHz.
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Now set up Slave 0 and 1:
	// Slave 0 reads out the magnetometer readings
	// Slave 1 starts a new measurement cycle for reading by slave 0 in the next cycle

	// First set the byte being sent to the magnetometer, which is to set
	// the single measurement mode.
	buf[0] = REG_9150_I2C_SLV1_DO;
	buf[1] = AK8975_SINGLE_MEAS; // Single Measurement mode
	ioPort->writeBlock(i2cAddress, buf, 2);

	// Start writing at the start of slave 0 control registers,
	// but continue writing the slave 1 registers in one transaction.
	buf[0] = REG_9150_I2C_SLV0_ADDR;
	// Slave 0
	buf[1] = I2C_SLVx_RW // read
			| AK8975_I2CAddr; // The I2C address of the magnetometer
	// The start register to read
	buf[2] = REG_AK8975_ST1;
	buf[3] = 8 // Number of bytes (status 1, 3 axis - 2-byte measurements, status2)
			| I2C_SLVx_EN // Enable slave interface
			;

	// Slave 1
	buf[4] = // I2C_SLVx_RW | // write (do not set read bit)
			 AK8975_I2CAddr; // The I2C address of the magnetometer
	// The Control register where the value in REG_9150_I2C_SLV1_DO (see above) is being written.
	buf[5] = REG_AK8975_CNTL;
	buf[6] = 1 // Number of bytes (Send one command byte)
			| I2C_SLVx_EN // Enable slave interface
			;
	ioPort->writeBlock(i2cAddress, buf, 7);

}

void MPU9150Driver::driverThreadFunction() {

	int numRetries = 0;

	if (ioPort == nullptr) {
		LOG4CXX_ERROR (logger,"No valid I/O port for driver " << getDriverName()
				<< ". The driver is not operable");
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				ioPort->open();
				numRetries = 0;
				setupMPU9150();
				processingMainLoop ();
				ioPort->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,"Error in main loop of driver \"" << getDriverName()
						<< "\":" << e.what());
				ioPort->close();

				sleep(errorTimeout);
			}
		}
	}
}

void MPU9150Driver::processingMainLoop () {

	auto nextStartConversion = OEVClock::now();

	while (!getStopDriverThread()) {
		SensorData &currSensorData = sensorDataArr[currSensorDataIndex];
		uint8_t buf[6	// Accel
					+ 2	// Temperature
					+ 6	// Gyro
					+ 8	// 8 bytes from Slave0
					];

		// advance the index, and wrap it around if necessary
		currSensorDataIndex++;
		currSensorDataIndex &= SIZE_SENSOR_DATA_ARRAY - 1;
		currSensorData.accelDataValid = false;
		currSensorData.gyroDataValid = false;
		currSensorData.magDataValid = false;

		for (;;) {
			buf[0] = ioPort->readByteAtRegAddrByte(i2cAddress, REG_9150_INT_STATUS);
			LOG4CXX_DEBUG(logger,"Interrupt status = 0x" << std::hex << uint16_t(buf[0]) << std::dec);
			if ((buf[0] & DATA_RDY_INT) == DATA_RDY_INT) {
				break;
			}
			// Re-start the clock, and re-start the cycle.
			nextStartConversion = OEVClock::now();
		}

		ioPort->readBlockAtRegAddrByte(i2cAddress, REG_9150_ACCEL_XOUT_H, buf, sizeof(buf));

		// Accel and gyro data are stored big-endian
		measurementData.accelX.uintVal = (buf[0] << 8) | buf[1];
		measurementData.accelY.uintVal = (buf[2] << 8) | buf[3];
		measurementData.accelZ.uintVal = (buf[4] << 8) | buf[5];

		measurementData.tempRaw.uintVal = (buf[6] << 8) | buf[7];

		measurementData.gyroX.uintVal = (buf[8] << 8) | buf[9];
		measurementData.gyroY.uintVal = (buf[10] << 8) | buf[11];
		measurementData.gyroZ.uintVal = (buf[12] << 8) | buf[13];

		measurementData.magStatus1 = buf[14];
		// Mag data are stored little-endian,
		// the magnetometer from another manufacturer slapped onto the primary die)
		measurementData.magX.uintVal = (buf[16] << 8) | buf[15];
		measurementData.magY.uintVal = (buf[18] << 8) | buf[17];
		measurementData.magZ.uintVal = (buf[20] << 8) | buf[19];

		measurementData.magStatus2 = buf[21];

		// Convert the raw data, and write them into the sensor data structure.
		currSensorData.accelDataValid = true;
		currSensorData.accelX = FloatType(measurementData.accelX.intVal) * accFactor *
				calibrationData.accelXFactor - calibrationData.accelXBias;
		currSensorData.accelY = FloatType(measurementData.accelY.intVal) * accFactor *
				calibrationData.accelYFactor - calibrationData.accelYBias;
		currSensorData.accelZ = FloatType(measurementData.accelZ.intVal) * accFactor *
				calibrationData.accelZFactor - calibrationData.accelZBias;

		LOG4CXX_TRACE(logger,"Accel  data " << measurementData.accelX.intVal
				<< ", " << measurementData.accelY.intVal
				<< ", " << measurementData.accelZ.intVal
				);
		LOG4CXX_TRACE(logger,"Accel  data in g = "
				<< (FloatType(measurementData.accelX.intVal) * accFactor) << ", "
				<< (FloatType(measurementData.accelY.intVal) * accFactor) << ", "
				<< (FloatType(measurementData.accelZ.intVal) * accFactor)
				);

		currSensorData.gyroDataValid = true;
		currSensorData.gyroX = FloatType(measurementData.gyroX.intVal) * gyrFactor
				* calibrationData.gyrXFactor;
		currSensorData.gyroY = FloatType(measurementData.gyroY.intVal) * gyrFactor
				* calibrationData.gyrYFactor;
		currSensorData.gyroZ = FloatType(measurementData.gyroZ.intVal) * gyrFactor
				* calibrationData.gyrZFactor;

		LOG4CXX_TRACE(logger,"Gyro   data " << measurementData.gyroX.intVal
				<< ", " << measurementData.gyroY.intVal
				<< ", " << measurementData.gyroZ.intVal
				);
		LOG4CXX_TRACE(logger,"Gyro  data in deg/s = "
				<< (FloatType(measurementData.gyroX.intVal) * gyrFactor) << ", "
				<< (FloatType(measurementData.gyroY.intVal) * gyrFactor) << ", "
				<< (FloatType(measurementData.gyroZ.intVal) * gyrFactor)
				);

		LOG4CXX_DEBUG(logger,"Magnet status1 = 0x" << std::hex << uint16_t(measurementData.magStatus1)
				<< ", status2 = 0x" << std::hex << uint16_t(measurementData.magStatus2));
		if ((measurementData.magStatus1 & AK8975_DRDY) && // data is ready
				!(measurementData.magStatus2 & (AK8975_HOFL | AK8975_DERR))) { // And no overflow or data error
			currSensorData.magDataValid = true;

			// NOTE: The magnetometer X and Y axes are rotated by 90 degrees.
			// Therefore I cannot compensate flipped axes by the factors, but I need to re-arrange programatically here.
			currSensorData.magX = FloatType(measurementData.magY.intVal) * magFactorY
					* calibrationData.magXFactor;
			currSensorData.magY = FloatType(measurementData.magX.intVal) * magFactorX
					* calibrationData.magYFactor;
			currSensorData.magZ = FloatType(measurementData.magZ.intVal) * magFactorZ
					* calibrationData.magZFactor;

			// Also print the raw data with X and Y exchanged
			LOG4CXX_TRACE(logger,"Magnet data " << measurementData.magY.intVal
					<< ", " << measurementData.magX.intVal
					<< ", " << measurementData.magZ.intVal
					);
			LOG4CXX_TRACE(logger,"Magnet data in uT = "
					<< currSensorData.magX << ", "
					<< currSensorData.magY << ", "
					<< currSensorData.magZ
					);
		}
		LOG4CXX_DEBUG(logger,"Temperature raw " << measurementData.tempRaw.intVal << " = "
				<< (FloatType(measurementData.tempRaw.intVal)/340.0f + 35.0f));

		updateKalman(currSensorData);

		// In case that you miss a cycle advance to the next cycle
		auto now = OEVClock::now();
		do {
			nextStartConversion += updateCyle;
		} while (nextStartConversion < now);
		std::this_thread::sleep_until(nextStartConversion);

	}
}

} // namespace openEV
