/*
 * TE_MEAS_AbsPressureDriver.cpp
 *
 *  Created on: Apr 26, 2021
 *      Author: hor
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <fstream>
#include <chrono>
#include <thread>

#include "TE-MEAS-AbsPressureDriver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.TE_MEAS_AbsPressure");
	}
}

#endif

namespace openEV::drivers::TE_MEAS_AbsPressure {

TE_MEAS_AbsPressureDriver::TE_MEAS_AbsPressureDriver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(STATIC_PRESSURE	);

}


TE_MEAS_AbsPressureDriver::~TE_MEAS_AbsPressureDriver() {

}


void TE_MEAS_AbsPressureDriver::driverInit(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

}

void TE_MEAS_AbsPressureDriver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, __FUNCTION__ << " Driver" << driverName << " read configuraion");

	try {
		auto portNameConfig = configuration.searchProperty("portName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"PortName\" is a struct or a string list.");
		}

		portName = portNameConfig->getStringValue();

		ioPort = dynamic_cast<io::I2CPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not an I2C port.");
		}
	} catch (std::exception const& e) {
		LOG4CXX_ERROR(logger, "Read configuration of driver \"" << driverName
				<< "\" failed:"
				<< e.what());
		throw;
	}

	i2cAddress = (long long)(configuration.getPropertyValue(
	    		std::string("i2cAddress"),
				(long long)(i2cAddress)));
	useTemperatureSensor = configuration.getPropertyValue(
    		std::string("useTemperatureSensor"),
			useTemperatureSensor);
	checkCRC = configuration.getPropertyValue(
    		std::string("checkCRC"),
			checkCRC);
    errorTimeout = configuration.getPropertyValue(
    		std::string("errorTimeout"),
			(long long)(errorTimeout));
    errorMaxNumRetries = configuration.getPropertyValue(
    		std::string("errorMaxNumRetries"),
			(long long)(errorMaxNumRetries));

	LOG4CXX_INFO(logger,"	portName = " << portName);
	LOG4CXX_INFO(logger,"	i2cAddress = 0x" << std::hex <<  uint32_t(i2cAddress) << std::dec);
	LOG4CXX_INFO(logger,"	useTemperatureSensor = " << useTemperatureSensor);
	LOG4CXX_INFO(logger,"	checkCRC = " << checkCRC);
	LOG4CXX_INFO(logger,"	errorTimeout = " << errorTimeout);
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);

}

#define SQUARE(x) ((x)*(x))

void TE_MEAS_AbsPressureDriver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain) {

	// Wait for 20 seconds for 16 samples to appear, and a defined temperature value
	for (int i = 0; i < 20; i++) {
		if (numValidInitValues < NumInitValues || isnan(temperatureVal)) {
			using namespace std::chrono_literals; // used for the term "1s" below. 's' being the second literal.

			LOG4CXX_TRACE(logger,__FUNCTION__ << ": Only " << numValidInitValues <<
					" valid samples collected. Wait another second");
			std::this_thread::sleep_for(1s);
		} else {
			break;
		}
	}

	if (numValidInitValues >= NumInitValues) {
		FloatType avgPressure = 0.0f;
		double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

		for (int i = 0 ; i < NumInitValues; i++) {
			avgPressure += initValues[i];
			LOG4CXX_TRACE(logger," initValues[" << i << "] = " << initValues[i]);
		}
		avgPressure /= FloatType(NumInitValues);
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": avgPressure = " << avgPressure);

		if (!isnan(measurements.gpsMSL)) {
			initQFF(varioStatus,measurements,varioMain,avgPressure);
		}

		if (isnan(varioStatus.altMSL)) {
			if (!isnan(varioStatus.qff) && !isnan(temperatureVal) /*!isnan(measurements.tempLocalC)*/) {
			auto const currTempK = temperatureVal /*measurements.tempLocalC*/ + CtoK;
			varioStatus.altMSL  = (currTempK -(pow((avgPressure / varioStatus.qff),(1.0/BarometricFormulaExponent)) * currTempK)) / TempLapseIndiffBoundLayer;
			varioStatus.lastPressure = avgPressure;
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Initial altitude from QFF:" << varioStatus.qff
					<< ", Temp (K): " << currTempK
					<< " = " << varioStatus.altMSL);

			// 1 mbar initial uncertainty translated into 8m.
			varioStatus.getErrorCovariance_P().
							coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) = SQUARE(1.0*8.0);

			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Initial variance = "
					<< varioStatus.getErrorCovariance_P().
					coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL));
			} else {
				LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Either QFF or local temperature or both are undefined. QFF = "
						<< varioStatus.qff << ", temperature = " << temperatureVal);
			}

		} else {
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": altMSL is already defined = " << varioStatus.altMSL);
		}

		if (isnan(varioStatus.getSystemNoiseCovariance_Q().
				coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL)) ||
				(varioStatus.getSystemNoiseCovariance_Q().
				coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) > SQUARE(4.0) * baseIntervalSec)) {

			varioStatus.getSystemNoiseCovariance_Q().
							coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) = SQUARE(2.0) * baseIntervalSec;
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": System noise increment = "
					<< varioStatus.getSystemNoiseCovariance_Q().
												coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL));
		}

	} else {
		LOG4CXX_WARN(logger,__FUNCTION__ << "Could not obtain " << NumInitValues
				<< " valid measurements in a row for 20 seconds. Cannot initialize the Kalman filter state.");
	}


}

void TE_MEAS_AbsPressureDriver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	// Nothing to do here

}


void TE_MEAS_AbsPressureDriver::driverThreadFunction() {

	int numRetries = 0;

	if (ioPort == nullptr) {
		LOG4CXX_ERROR (logger,"No valid I/O port for driver " << getDriverName()
				<< ". The driver is not operable");
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				ioPort->open();
				numRetries = 0;
				processingMainLoop ();
				ioPort->close();
			}
			catch (TE_MEAS_AbsPressureCRCErrorException const & e1) {
				LOG4CXX_FATAL(logger,"CRC error Error of driver \"" << getDriverName()
						<< "\". Stop processing. Reason: " << e1.what());
				return;
			}
			catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,"Error in main loop of driver \"" << getDriverName()
						<< "\":" << e.what());
				ioPort->close();

				sleep(errorTimeout);
			}
		}
	}
}

void TE_MEAS_AbsPressureDriver::processingMainLoop() {
    std::chrono::system_clock::time_point nextStartConversion;

    using namespace std::chrono_literals;

	setupSensor();

	nextStartConversion = std::chrono::system_clock::now();

	while (!getStopDriverThread()) {

		startPressureConversion();

		std::this_thread::sleep_for(10ms);

		readoutPressure();
		nextStartConversion += 100ms;
		std::this_thread::sleep_until(nextStartConversion);
	}

}

/// Like in the AVR libraries
#define _BV(x) (1<<(x))

void TE_MEAS_AbsPressureDriver::setupSensor() {
    using namespace std::chrono_literals;

	// Reset the sensor.
	ioPort->writeByte(i2cAddress, CMD_Reset);
	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Sent reset to " << getDriverName());

	// Let the sensor run through its start code
	std::this_thread::sleep_for(20ms);

	// Read out the coefficients from the RAM.
	for (uint8_t i = 0; i < lenPromArray;++i) {
		uint8_t buf[2];
		// Send the PROM read command
		ioPort->writeByte(i2cAddress, CMD_PROM_READ_REG (i));
		LOG4CXX_TRACE(logger,__FUNCTION__ << ": Send read Prom word #" << uint32_t(i));
		std::this_thread::sleep_for(5ms);
		ioPort->readBlock(i2cAddress, buf, sizeof(buf));
		promArray[i] = buf[0] << 8 | buf[1];
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": PROM word #" << uint32_t(i) << " = " << std::hex << promArray[i] << std::dec);
	}


	verifyCRC();

}

void TE_MEAS_AbsPressureDriver::verifyCRC() {

	// Extract the CRC stored in the PROM.
	uint16_t crcStored = promArray[7] & 0xF;

	// Start verbatim part from AN520
	int cnt; // simple counter
	uint16_t nRem; // crc reminder
	uint16_t crcRead; // original value of the crc
	uint8_t nBit;
	nRem = 0x00;
	crcRead = promArray[7]; //save read CRC

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": CRC value in the PROM is" << crcStored);

	promArray[7]=(0xFF00 & (promArray[7])); //CRC byte is replaced by 0
	for (cnt = 0; cnt < 16; cnt++) { // operation is performed on bytes
		// choose LSB or MSB
		if (cnt%2==1) {
			nRem ^= (uint16_t) ((promArray[cnt>>1]) & 0x00FF);
		}
		else {
			nRem ^= (uint16_t) (promArray[cnt>>1]>>8);
		}
		for (nBit = 8; nBit > 0; nBit--) {
			if (nRem & (0x8000)){
				nRem = (nRem << 1) ^ 0x3000;
			}
			else {
				nRem = (nRem << 1);
			}
		}
	}
	nRem= (0x000F & (nRem >> 12)); // final 4-bit reminder is CRC code
	promArray[7]=crcRead; // restore the crc_read to its original place
	// End verbatim part from AN520

	// ... and start of my stuff
	if (crcStored != nRem) {
		std::ostringstream str;
		str << "Driver " << getDriverName() << ": CRC mismatch: CRC in the PROM = 0x" << std::hex << crcStored
				<< "; calculated CRC = 0x" << nRem;
		if (checkCRC) {
			throw TE_MEAS_AbsPressureCRCErrorException (__FILE__, __LINE__, str.str().c_str());
		} else {
			LOG4CXX_WARN(logger,__FUNCTION__ << str.str());
		}
	} else {
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Calculated CRC value " << nRem << " is equal to the CRC in the PROM. CRC check successful.");
	}
}

void TE_MEAS_AbsPressureDriver::startPressureConversion() {

	uint8_t ctrl1AddrVal[2];

	ioPort->writeByte(i2cAddress, CMD_Convert_D1_OSR_4096);
	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Sent command " << CMD_Convert_D1_OSR_4096 << " to " << getDriverName());

}

void TE_MEAS_AbsPressureDriver::readoutPressure() {

	uint8_t statusVal = 0;
	uint8_t sensorValues[3];

	ioPort->readBlockAtRegAddrByte(i2cAddress, 0, sensorValues, sizeof(sensorValues));

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Read sensor values = 0x"
			<< std::hex << uint16_t(sensorValues[0]) << ", 0x" << uint16_t(sensorValues[1]) << ", 0x" << uint16_t(sensorValues[2])
			<< std::dec <<" from " << getDriverName());

	if (sensorValues[0] != 0 || sensorValues[1] != 0 || sensorValues[2] != 0 ) {

		/* !!!-
		// Only accept values within the operational range
		if (pressureVal >= 500.0f && pressureVal <= 1500.0f) {

			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal << " mBar."
					);

			if (getIsKalmanUpdateRunning()) {
				GliderVarioMainPriv::LockedCurrentStatus lockedStatus(*varioMain);
				FloatType &tempLocalC = lockedStatus.getMeasurementVector()->tempLocalC;

				if (useTemperatureSensor) {
					tempLocalC = temperatureVal;
				}

				GliderVarioMeasurementUpdater::staticPressureUpd(
						pressureVal, tempLocalC, SQUARE(0.5),
						*lockedStatus.getMeasurementVector(), *lockedStatus.getCurrentStatus());

			} else {
				if (numValidInitValues < NumInitValues) {
					initValues[numValidInitValues] = pressureVal;
					numValidInitValues ++;
				}
			}

		} else { // if (pressureVal >= 500.0f && pressureVal <= 1500.0f) {
			// and if you are out of the operational range re-start initialization data collection.
			// More likely the sensor is screwed you you connected the static system to a pump or the oxygen system :D
			numValidInitValues = 0;
		}
		*/

	} else {
		// Missed cycle in the initialization phase. Start from scratch.
		numValidInitValues = 0;
	}

}

void TE_MEAS_AbsPressureDriver::startTemperatureConversion() {
}

void TE_MEAS_AbsPressureDriver::readoutTemperature() {
}

void TE_MEAS_AbsPressureDriver::initQFF(
		GliderVarioStatus &varioStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain,
		FloatType avgPressure) {

	GliderVarioStatus::StatusCoVarianceType &systemNoiseCov = varioStatus.getSystemNoiseCovariance_Q();
	GliderVarioStatus::StatusCoVarianceType &errorCov = varioStatus.getErrorCovariance_P();
	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	FloatType pressureFactor = calcBarometricFactor(
    		measurements.gpsMSL,
			temperatureVal
			);
	LOG4CXX_DEBUG (logger,__FUNCTION__ << " pressureFactor = " << pressureFactor);

	varioStatus.qff = avgPressure / pressureFactor;

	// Assume quite a bit lower variance of qff pressure as the initial altitude variance (9)
	errorCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			= SQUARE(1);
	systemNoiseCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) =
			SQUARE(0.05) * baseIntervalSec; // 0.1hPa/sec

	LOG4CXX_DEBUG (logger,"	QFF = " << varioStatus.qff
			<< ", initial variance = "
			<< errorCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			<< ", variance increment = "
			<< systemNoiseCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			<< " / " << baseIntervalSec << "s");



}

} /* namespace openEV */
