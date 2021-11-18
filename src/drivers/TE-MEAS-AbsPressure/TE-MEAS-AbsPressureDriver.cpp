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
: DriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(STATIC_PRESSURE	);

	// Default cycle time as documented in the template parameter file
	using namespace std::chrono_literals;
	updateCyle = 100ms;

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

#if !TE_MEAS_ABS_PRESSURE_TEST_MODE
		ioPort = dynamic_cast<io::I2CPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not an I2C port.");
		}
#endif
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
#if TE_MEAS_ABS_PRESSURE_TEST_MODE
	// If in test mode the CRC check is guaranteed to fail because
	// only coefficient example values are given in the data sheets
	// but not a complete set of PROM memory.
	checkCRC = false;
#else
	checkCRC = configuration.getPropertyValue(
    		std::string("checkCRC"),
			checkCRC);
#endif
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
				coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) > SQUARE(2.0) * baseIntervalSec)) {

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

	if (
#if TE_MEAS_ABS_PRESSURE_TEST_MODE
			0
#else
			ioPort == nullptr
#endif
			) {
		LOG4CXX_ERROR (logger,"No valid I/O port for driver " << getDriverName()
				<< ". The driver is not operable");
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
#if !TE_MEAS_ABS_PRESSURE_TEST_MODE
				ioPort->open();
#endif
				numRetries = 0;
				processingMainLoop ();
#if !TE_MEAS_ABS_PRESSURE_TEST_MODE
				ioPort->close();
#endif
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
#if !TE_MEAS_ABS_PRESSURE_TEST_MODE
				ioPort->close();
#endif

				sleep(errorTimeout);
			}
		}
	}
}

void TE_MEAS_AbsPressureDriver::processingMainLoop() {

    using namespace std::chrono_literals;

	setupSensor();

	// read the temperature initially.
	startTemperatureConversion();
	std::this_thread::sleep_for(5ms);
	readoutTemperature();

	auto nextStartConversion = std::chrono::system_clock::now();

	while (!getStopDriverThread()) {

		startPressureConversion();

		std::this_thread::sleep_for(10ms);

		readoutPressure();

		// read the temperature for the next cycle.
		// Temperature changes are comparingly sedate due to the thermal inertia of the sensor.
		startTemperatureConversion();
		std::this_thread::sleep_for(5ms);
		readoutTemperature();

		// In case that you miss a cycle advance to the next cycle
		auto now = std::chrono::system_clock::now();
		do {
			nextStartConversion += updateCyle;
		} while (nextStartConversion < now);
		std::this_thread::sleep_until(nextStartConversion);
	}

}

/// Like in the AVR libraries
#define _BV(x) (1<<(x))

void TE_MEAS_AbsPressureDriver::setupSensor() {
    using namespace std::chrono_literals;

#if !TE_MEAS_ABS_PRESSURE_TEST_MODE
	// Reset the sensor.
	ioPort->writeByte(i2cAddress, CMD_Reset);
	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Sent reset to " << getDriverName());
#endif

	// Let the sensor run through its start code
	std::this_thread::sleep_for(20ms);

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    testFillPromData();
#else
	// Read out the coefficients from the RAM.
	for (uint8_t i = 0; i < lenPromArray;++i) {
		uint8_t buf[2];
		// Send the PROM read command
		ioPort->writeByte(i2cAddress, CMD_PROM_READ_REG (i));
		LOG4CXX_TRACE(logger,__FUNCTION__ << ": Send read Prom word #" << uint32_t(i));
		std::this_thread::sleep_for(5ms);
		ioPort->readBlock(i2cAddress, buf, sizeof(buf));
		promArray[i] = buf[0] << 8 | buf[1];
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": PROM word #" << uint32_t(i) << " = 0x"
				<< std::hex << promArray[i] << std::dec << " = " << promArray[i]);
	}
#endif

	// Verify that the PROM content is not corrupted.
	verifyCRC();

}

void TE_MEAS_AbsPressureDriver::startPressureConversion() {

#if !TE_MEAS_ABS_PRESSURE_TEST_MODE
	ioPort->writeByte(i2cAddress, CMD_Convert_D1_OSR_4096);
	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Sent command " << CMD_Convert_D1_OSR_4096 << " to " << getDriverName());
#endif
}

void TE_MEAS_AbsPressureDriver::readoutPressure() {

	uint8_t sensorValues[3];

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    testGetPressureVal(sensorValues);
#else
	ioPort->readBlockAtRegAddrByte(i2cAddress, 0, sensorValues, sizeof(sensorValues));
#endif

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Read sensor values = 0x"
			<< std::hex << uint16_t(sensorValues[0]) << ", 0x" << uint16_t(sensorValues[1]) << ", 0x" << uint16_t(sensorValues[2])
			<< std::dec <<" from " << getDriverName());

	if (sensorValues[0] != 0 || sensorValues[1] != 0 || sensorValues[2] != 0 ) {

		// Call sensor type specific method of sub-class
		convertPressure(sensorValues);

		// Only accept values within the operational range
		if (pressureVal >= 300.0f && pressureVal <= 1200.0f) {

			if (getIsKalmanUpdateRunning()) {
				GliderVarioMainPriv::LockedCurrentStatus lockedStatus(*varioMain);
				FloatType &tempLocalC = lockedStatus.getMeasurementVector()->tempLocalC;

				if (useTemperatureSensor) {
					tempLocalC = temperatureVal;
				}

				GliderVarioMeasurementUpdater::staticPressureUpd(
						pressureVal, tempLocalC, SQUARE(0.1),
						*lockedStatus.getMeasurementVector(), *lockedStatus.getCurrentStatus());

			} else {
				if (numValidInitValues < NumInitValues) {
					initValues[numValidInitValues] = pressureVal;
					numValidInitValues ++;
				}
			}

		} else { // if (pressureVal >= 500.0f && pressureVal <= 1500.0f) {
			LOG4CXX_WARN(logger,__FUNCTION__ << ": Pressure = " << pressureVal << " mBar is outside the expected range!");
			// and if you are out of the operational range re-start initialization data collection.
			// More likely the sensor is screwed you you connected the static system to a pump or the oxygen system :D
			numValidInitValues = 0;
		}

	} else {
		// Missed cycle in the initialization phase. Start from scratch.
		numValidInitValues = 0;
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": 0-values from A/D");
	}

}

void TE_MEAS_AbsPressureDriver::startTemperatureConversion() {
#if !TE_MEAS_ABS_PRESSURE_TEST_MODE
	ioPort->writeByte(i2cAddress, CMD_Convert_D2_OSR_1024);
	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Sent command " << CMD_Convert_D2_OSR_1024 << " to " << getDriverName());
#endif
}

void TE_MEAS_AbsPressureDriver::readoutTemperature() {

	uint8_t sensorValues[3];

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    testGetTemperatureVal(sensorValues);
#else
	ioPort->readBlockAtRegAddrByte(i2cAddress, 0, sensorValues, sizeof(sensorValues));
#endif

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Read sensor values = 0x"
			<< std::hex << uint16_t(sensorValues[0]) << ", 0x" << uint16_t(sensorValues[1]) << ", 0x" << uint16_t(sensorValues[2])
			<< std::dec <<" from " << getDriverName());

	if (sensorValues[0] != 0 || sensorValues[1] != 0 || sensorValues[2] != 0 ) {
		convertTemperature(sensorValues);
	} // if (sensorValues[0] != 0 || sensorValues[1] != 0 || sensorValues[2] != 0 )

}

void TE_MEAS_AbsPressureDriver::convertTemperature(const uint8_t rawValue[]) {
	int32_t D2 = (int32_t(rawValue[0])<<16) | (int32_t(rawValue[1])<<8) | int32_t(rawValue[2]);
	deltaTemp = D2 - int32_t(promArray[5]) * 256L;

	tempCentiC = 2000L + ((int64_t(deltaTemp) * int64_t(promArray[6])) / (1LL<<23));

	// Second order temperature compensation is implemented in subclasses.
	// Second order compensation is not defined for all sensors.

	temperatureVal = FloatType(tempCentiC) / 100.0f;


	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Calculate temp: D2 = " << D2
			<< ", dT = " << deltaTemp
			<< ", TEMP = " << tempCentiC
			<< ", temperatureVal = " << temperatureVal);

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
	errorCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) = 1.0f;
	systemNoiseCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) =
			SQUARE(0.0001) * baseIntervalSec;

	LOG4CXX_DEBUG (logger,"	QFF = " << varioStatus.qff
			<< ", initial variance = "
			<< errorCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			<< ", variance increment = "
			<< systemNoiseCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			<< " / " << baseIntervalSec << "s");

}

EightPinDriver::~EightPinDriver() {
}

void EightPinDriver::verifyCRC() {

	// Extract the CRC stored in the PROM.
	auto crcStored = promArray[7] & 0xF;

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
			nRem ^= uint16_t((promArray[cnt>>1]) & 0x00FF);
		}
		else {
			nRem ^= uint16_t(promArray[cnt>>1]>>8);
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

FourPinDriver::~FourPinDriver() {
}

void FourPinDriver::verifyCRC() {

	// Extract the CRC stored in the PROM.
	uint16_t crcStored = ((promArray[0] >> 12) & 0x000F);

	// original value of the crc
	auto crcRead = promArray[0];


	// Start verbatim from the MS5637 data sheet except for
	// more definite types, and variable name changes, and re-formatting.
	int cnt; // simple counter
	uint16_t nRem=0; // crc reminder
	uint8_t nBit;
	promArray[0]=((promArray[0]) & 0x0FFF); // CRC byte is replaced by 0
	promArray[7]=0; // Subsidiary value, set to 0
	for (cnt = 0; cnt < 16; cnt++) { // operation is performed on bytes
		// choose LSB or MSB
		if (cnt%2==1) {
			nRem ^= uint16_t((promArray[cnt>>1]) & 0x00FF);
		} else {
			nRem ^= uint16_t(promArray[cnt>>1]>>8);
		}
		for (nBit = 8; nBit > 0; nBit--) {
			if (nRem & (0x8000)) {
				nRem = (nRem << 1) ^ 0x3000;
			} else  {
				nRem = (nRem << 1);
			}
		}
	}
	nRem= ((nRem >> 12) & 0x000F); // final 4-bit reminder is CRC code
	// End verbatim from the MS5637 data sheet

	// restore the PROM array
	promArray[0] = crcRead;

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

MS5803Driver::~MS5803Driver() {
}

void MS5803Driver::convertTemperature(const uint8_t rawValue[]) {

	// un-compensated conversion by the base class
	TE_MEAS_AbsPressureDriver::convertTemperature(rawValue);

	// Second order temperature compensation only for the converted temperature
	if (tempCentiC < 2000L) {

		temperatureVal -= FloatType((int64_t(deltaTemp) * int64_t(deltaTemp)) / (1LL<<31)) / 100.0f;

		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Second order compensation: T = " << tempCentiC
				<< ", temperatureVal = " << temperatureVal);
	}
}

void MS5803Driver::convertPressure(const uint8_t rawValue[]) {
	int32_t D1 = (int32_t(rawValue[0])<<16) | (int32_t(rawValue[1])<<8) | int32_t(rawValue[2]);

	int64_t OFF = (int64_t(promArray[2])<<16) + ((int64_t(promArray[4])*int64_t(deltaTemp)) / (1LL<<7));
	int64_t SENS = (int64_t(promArray[1])<<15) + ((int64_t(promArray[3])*int64_t(deltaTemp)) / (1LL<<8));

	// Second order temperature compensation
	int64_t OFF2 = 0LL;
	int64_t SENSE2 = 0LL;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate first order            D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			);

	if (tempCentiC < 2000L) {
		int32_t tempSquare20 = tempCentiC - 2000L;
		tempSquare20 *= tempSquare20;

		OFF2 = 3LL * tempSquare20;
		SENSE2 = (7LL * tempSquare20) / (1LL<<3);

		if (tempCentiC < -1500L) {
			int32_t tempSquareMin15 = tempCentiC + 1500L;
			tempSquareMin15 *= tempSquareMin15;

			SENSE2 += 2LL * tempSquareMin15;
		}
	} else { // if (TEMP < 2000)
		if (tempCentiC > 4500) {
			int32_t tempSquare45 = tempCentiC - 4500L;
			tempSquare45 *= tempSquare45;

			SENSE2 = int64_t(tempSquare45) / 8LL;
		}
	} // if (TEMP < 2000) {} else

	OFF -= OFF2;
	SENS -= SENSE2;

	int64_t P = (((int64_t(D1)*SENS) / (1LL<<21)) - OFF) / (1LL<<15);

	pressureVal = FloatType(P) / 100.0f;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate Pressure Second order: D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			<< ", dT = " << deltaTemp
			<< ", TEMP = " << tempCentiC
			<< ", OFF2 = " << OFF2
			<< ", SENSE2 = " << SENSE2
			<< ", P = " << P
			);

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal);
}

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
void MS5803Driver::testFillPromData() {
	promArray[1] = 40127;
	promArray[2] = 36924;
	promArray[3] = 23317;
	promArray[4] = 23282;
	promArray[5] = 33464;
	promArray[6] = 28312;
}

void MS5803Driver::testGetTemperatureVal(uint8_t data[]) {
	uint32_t val = 8569150;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

void MS5803Driver::testGetPressureVal(uint8_t data[]) {
	uint32_t val = 9085466;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}
#endif // #if TE_MEAS_ABS_PRESSURE_TEST_MODE

MS5607Driver::~MS5607Driver() {
}

void MS5607Driver::convertTemperature(const uint8_t rawValue[]) {

	// un-compensated conversion by the base class
	TE_MEAS_AbsPressureDriver::convertTemperature(rawValue);

	// Second order temperature compensation only for the converted temperature
	if (tempCentiC < 2000L) {

		temperatureVal -= FloatType((int64_t(deltaTemp) * int64_t(deltaTemp)) / (1LL<<31)) / 100.0f;

		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Second order compensation: T = " << tempCentiC
				<< ", temperatureVal = " << temperatureVal);
	}
}

void MS5607Driver::convertPressure(const uint8_t rawValue[]) {
	int32_t D1 = (int32_t(rawValue[0])<<16) | (int32_t(rawValue[1])<<8) | int32_t(rawValue[2]);

	int64_t OFF = (int64_t(promArray[2])<<17) + ((int64_t(promArray[4])*int64_t(deltaTemp)) / (1LL<<6));
	int64_t SENS = (int64_t(promArray[1])<<16) + ((int64_t(promArray[3])*int64_t(deltaTemp)) / (1LL<<7));

	// Second order temperature compensation
	int64_t OFF2 = 0LL;
	int64_t SENSE2 = 0LL;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate first order            D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			);

	if (tempCentiC < 2000L) {
		int32_t tempSquare20 = tempCentiC - 2000L;
		tempSquare20 *= tempSquare20;

		OFF2 = (61LL * tempSquare20) / 16LL;
		SENSE2 = 2LL * tempSquare20;

		if (tempCentiC < -1500L) {
			int32_t tempSquareMin15 = tempCentiC + 1500L;
			tempSquareMin15 *= tempSquareMin15;

			OFF2 += 15LL * tempSquareMin15;
			SENSE2 += 8LL * tempSquareMin15;
		}
	} // if (TEMP < 2000)

	OFF -= OFF2;
	SENS -= SENSE2;

	int64_t P = (((int64_t(D1)*SENS) / (1LL<<21)) - OFF) / (1LL<<15);

	pressureVal = FloatType(P) / 100.0f;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate Pressure Second order: D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			<< ", dT = " << deltaTemp
			<< ", TEMP = " << tempCentiC
			<< ", OFF2 = " << OFF2
			<< ", SENSE2 = " << SENSE2
			<< ", P = " << P
			);

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal);
}

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
void MS5607Driver::testFillPromData() {
	promArray[1] = 46372;
	promArray[2] = 43981;
	promArray[3] = 29059;
	promArray[4] = 27842;
	promArray[5] = 31553;
	promArray[6] = 28165;
}

void MS5607Driver::testGetTemperatureVal(uint8_t data[]) {
	uint32_t val = 8077636;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

void MS5607Driver::testGetPressureVal(uint8_t data[]) {
	uint32_t val = 6465444;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}
#endif // #if TE_MEAS_ABS_PRESSURE_TEST_MODE

MS5611Driver::~MS5611Driver() {
}

void MS5611Driver::convertTemperature(const uint8_t rawValue[]) {

	// un-compensated conversion by the base class
	TE_MEAS_AbsPressureDriver::convertTemperature(rawValue);

	// Second order temperature compensation only for the converted temperature
	if (tempCentiC < 2000L) {
		temperatureVal -=  FloatType((int64_t(deltaTemp) * int64_t(deltaTemp)) / (1LL<<31)) / 100.0f;
	}

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Second order compensation: T = " << tempCentiC
			<< ", temperatureVal = " << temperatureVal);
}

void MS5611Driver::convertPressure(const uint8_t rawValue[]) {
	int32_t D1 = (int32_t(rawValue[0])<<16) | (int32_t(rawValue[1])<<8) | int32_t(rawValue[2]);

	int64_t OFF = (int64_t(promArray[2])<<16) + ((int64_t(promArray[4])*int64_t(deltaTemp)) / (1LL<<7));
	int64_t SENS = (int64_t(promArray[1])<<15) + ((int64_t(promArray[3])*int64_t(deltaTemp)) / (1LL<<8));

	// Second order temperature compensation
	int64_t OFF2 = 0LL;
	int64_t SENSE2 = 0LL;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate first order            D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			);

	if (tempCentiC < 2000L) {
		int32_t tempSquare20 = tempCentiC - 2000L;
		tempSquare20 *= tempSquare20;

		OFF2 = (5LL * tempSquare20) / 2LL;
		SENSE2 = (5LL * tempSquare20) / 4LL;

		if (tempCentiC < -1500L) {
			int32_t tempSquareMin15 = tempCentiC + 1500L;
			tempSquareMin15 *= tempSquareMin15;

			OFF2 += 7LL * tempSquareMin15;
			SENSE2 += (11LL * tempSquareMin15) / 2LL;
		}
	} // if (TEMP < 2000)

	OFF -= OFF2;
	SENS -= SENSE2;

	int64_t P = (((int64_t(D1)*SENS) / (1LL<<21)) - OFF)  / (1LL<<15);

	pressureVal = FloatType(P) / 100.0f;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate Pressure Second order: D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			<< ", dT = " << deltaTemp
			<< ", TEMP = " << tempCentiC
			<< ", OFF2 = " << OFF2
			<< ", SENSE2 = " << SENSE2
			<< ", P = " << P
			);

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal);
}

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
void MS5611Driver::testFillPromData() {
	promArray[1] = 40127;
	promArray[2] = 36924;
	promArray[3] = 23317;
	promArray[4] = 23282;
	promArray[5] = 33464;
	promArray[6] = 28312;
}

void MS5611Driver::testGetTemperatureVal(uint8_t data[]) {
	uint32_t val = 8569150;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

void MS5611Driver::testGetPressureVal(uint8_t data[]) {
	uint32_t val = 9085466;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}
#endif // #if TE_MEAS_ABS_PRESSURE_TEST_MODE

MS5637Driver::~MS5637Driver() {
}

void MS5637Driver::convertTemperature(const uint8_t rawValue[]) {

	// un-compensated conversion by the base class
	TE_MEAS_AbsPressureDriver::convertTemperature(rawValue);

	// Second order temperature compensation only for the converted temperature
	if (tempCentiC < 2000L) {
		temperatureVal -= FloatType((3LL * int64_t(deltaTemp) * int64_t(deltaTemp)) / (1LL<<33)) / 100.0f;
	} else {
		temperatureVal -= FloatType((5LL * int64_t(deltaTemp) * int64_t(deltaTemp)) / (1LL<<38)) / 100.0f;
	}

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Second order compensation: T = " << tempCentiC
			<< ", temperatureVal = " << temperatureVal);
}

void MS5637Driver::convertPressure(const uint8_t rawValue[]) {
	int32_t D1 = (int32_t(rawValue[0])<<16) | (int32_t(rawValue[1])<<8) | int32_t(rawValue[2]);

	int64_t OFF = (int64_t(promArray[2])<<17) + ((int64_t(promArray[4])*int64_t(deltaTemp)) / (1LL<<6));
	int64_t SENS = (int64_t(promArray[1])<<16) + ((int64_t(promArray[3])*int64_t(deltaTemp)) / (1LL<<7));

	// Second order temperature compensation
	int64_t OFF2 = 0LL;
	int64_t SENSE2 = 0LL;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate first order            D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			);

	if (tempCentiC < 2000L) {
		int32_t tempSquare20 = tempCentiC - 2000L;
		tempSquare20 *= tempSquare20;

		OFF2 = (61LL * tempSquare20) / 16LL;
		SENSE2 = (29LL * tempSquare20) / 16LL;

		if (tempCentiC < -1500L) {
			int32_t tempSquareMin15 = tempCentiC + 1500L;
			tempSquareMin15 *= tempSquareMin15;

			OFF2 += 17LL * tempSquareMin15;
			SENSE2 += 9LL * tempSquareMin15;

		}

	} // if (TEMP < 2000)

	OFF -= OFF2;
	SENS -= SENSE2;

	int64_t P = (((int64_t(D1)*SENS) / (1LL<<21)) - OFF)  / (1LL<<15);

	pressureVal = FloatType(P) / 100.0f;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate Pressure Second order: D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			<< ", dT = " << deltaTemp
			<< ", TEMP = " << tempCentiC
			<< ", OFF2 = " << OFF2
			<< ", SENSE2 = " << SENSE2
			<< ", P = " << P
			);

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal);
}

#if TE_MEAS_ABS_PRESSURE_TEST_MODE

void MS5637Driver::testFillPromData() {
	promArray[1] = 46372;
	promArray[2] = 43981;
	promArray[3] = 29059;
	promArray[4] = 27842;
	promArray[5] = 31553;
	promArray[6] = 28165;
}

void MS5637Driver::testGetTemperatureVal(uint8_t data[]) {
	uint32_t val = 8077636;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

void MS5637Driver::testGetPressureVal(uint8_t data[]) {
	uint32_t val = 6465444;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

#endif // #if TE_MEAS_ABS_PRESSURE_TEST_MODE

MS5805Driver::~MS5805Driver() {
}

void MS5805Driver::convertTemperature(const uint8_t rawValue[]) {

	// un-compensated conversion by the base class
	TE_MEAS_AbsPressureDriver::convertTemperature(rawValue);

	// Second order temperature compensation only for the converted temperature
	if (tempCentiC < 2000L) {
		temperatureVal -= FloatType((11LL * int64_t(deltaTemp) * int64_t(deltaTemp)) / (1LL<<35)) / 100.0f;

		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Second order compensation: T = " << tempCentiC
				<< ", temperatureVal = " << temperatureVal);
	}
}

void MS5805Driver::convertPressure(const uint8_t rawValue[]) {
	int32_t D1 = (int32_t(rawValue[0])<<16) | (int32_t(rawValue[1])<<8) | int32_t(rawValue[2]);

	int64_t OFF = (int64_t(promArray[2])<<17) + ((int64_t(promArray[4])*int64_t(deltaTemp)) / (1LL<<6));
	int64_t SENS = (int64_t(promArray[1])<<16) + ((int64_t(promArray[3])*int64_t(deltaTemp)) / (1LL<<7));

	// Second order temperature compensation
	int64_t OFF2 = 0LL;
	int64_t SENSE2 = 0LL;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate first order            D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			);

	if (tempCentiC < 2000L) {
		int32_t tempSquare20 = tempCentiC - 2000L;
		tempSquare20 *= tempSquare20;

		OFF2 = (31LL * tempSquare20) / 8LL;
		SENSE2 = (63LL * tempSquare20) / 32LL;

	} // if (TEMP < 2000)

	OFF -= OFF2;
	SENS -= SENSE2;

	int64_t P = (((int64_t(D1)*SENS) / (1LL<<21)) - OFF)  / (1LL<<15);

	pressureVal = FloatType(P) / 100.0f;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate Pressure Second order: D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			<< ", dT = " << deltaTemp
			<< ", TEMP = " << tempCentiC
			<< ", OFF2 = " << OFF2
			<< ", SENSE2 = " << SENSE2
			<< ", P = " << P
			);

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal);
}

#if TE_MEAS_ABS_PRESSURE_TEST_MODE

void MS5805Driver::testFillPromData() {
	promArray[1] = 46372;
	promArray[2] = 43981;
	promArray[3] = 29059;
	promArray[4] = 27842;
	promArray[5] = 31553;
	promArray[6] = 28165;
}

void MS5805Driver::testGetTemperatureVal(uint8_t data[]) {
	uint32_t val = 8077636;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

void MS5805Driver::testGetPressureVal(uint8_t data[]) {
	uint32_t val = 6465444;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

#endif // #if TE_MEAS_ABS_PRESSURE_TEST_MODE

MS5837Driver::~MS5837Driver() {
}

void MS5837Driver::convertTemperature(const uint8_t rawValue[]) {

	// un-compensated conversion by the base class
	TE_MEAS_AbsPressureDriver::convertTemperature(rawValue);

	// Second order temperature compensation only for the converted temperature
	if (tempCentiC < 2000L) {
		temperatureVal -= FloatType((11LL * int64_t(deltaTemp) * int64_t(deltaTemp)) / (1LL<<35)) / 100.0f;

		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Second order compensation: T = " << tempCentiC
				<< ", temperatureVal = " << temperatureVal);
	}
}

void MS5837Driver::convertPressure(const uint8_t rawValue[]) {
	int32_t D1 = (int32_t(rawValue[0])<<16) | (int32_t(rawValue[1])<<8) | int32_t(rawValue[2]);

	int64_t OFF = (int64_t(promArray[2])<<17) + ((int64_t(promArray[4])*int64_t(deltaTemp)) / (1LL<<6));
	int64_t SENS = (int64_t(promArray[1])<<16) + ((int64_t(promArray[3])*int64_t(deltaTemp)) / (1LL<<7));

	// Second order temperature compensation
	int64_t OFF2 = 0LL;
	int64_t SENSE2 = 0LL;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate first order            D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			);

	if (tempCentiC < 2000L) {
		int32_t tempSquare20 = tempCentiC - 2000L;
		tempSquare20 *= tempSquare20;

		OFF2 = (31LL * tempSquare20) / 8LL;
		SENSE2 = (63LL * tempSquare20) / 32LL;

	} // if (TEMP < 2000)

	OFF -= OFF2;
	SENS -= SENSE2;

	int64_t P = (((int64_t(D1)*SENS) / (1LL<<21)) - OFF)  / (1LL<<15);

	pressureVal = FloatType(P) / 100.0f;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate Pressure Second order: D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			<< ", dT = " << deltaTemp
			<< ", TEMP = " << tempCentiC
			<< ", OFF2 = " << OFF2
			<< ", SENSE2 = " << SENSE2
			<< ", P = " << P
			);

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal);
}

#if TE_MEAS_ABS_PRESSURE_TEST_MODE

void MS5837Driver::testFillPromData() {
	promArray[1] = 46372;
	promArray[2] = 43981;
	promArray[3] = 29059;
	promArray[4] = 27842;
	promArray[5] = 31553;
	promArray[6] = 28165;
}

void MS5837Driver::testGetTemperatureVal(uint8_t data[]) {
	uint32_t val = 8077636;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

void MS5837Driver::testGetPressureVal(uint8_t data[]) {
	uint32_t val = 6465444;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

#endif // #if TE_MEAS_ABS_PRESSURE_TEST_MODE

MS5839_MS5840Driver::~MS5839_MS5840Driver() {
}

void MS5839_MS5840Driver::convertTemperature(const uint8_t rawValue[]) {

	// un-compensated conversion by the base class
	TE_MEAS_AbsPressureDriver::convertTemperature(rawValue);

	// Second order temperature compensation only for the converted temperature
	if (tempCentiC < 2000L) {
		int64_t deltaTempSq = int64_t(deltaTemp) * int64_t(deltaTemp);
		if (tempCentiC <= 1000L) {
			temperatureVal -= FloatType((14LL * deltaTempSq) / (1LL<<35)) / 100.0f;
		} else {
			temperatureVal -= FloatType((12LL * deltaTempSq) / (1LL<<35)) / 100.0f;
		}

		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Second order compensation: T = " << tempCentiC
				<< ", temperatureVal = " << temperatureVal);
	}
}

void MS5839_MS5840Driver::convertPressure(const uint8_t rawValue[]) {
	int32_t D1 = (int32_t(rawValue[0])<<16) | (int32_t(rawValue[1])<<8) | int32_t(rawValue[2]);

	int64_t OFF = (int64_t(promArray[2])<<17) + ((int64_t(promArray[4])*int64_t(deltaTemp)) / (1LL<<6));
	int64_t SENS = (int64_t(promArray[1])<<16) + ((int64_t(promArray[3])*int64_t(deltaTemp)) / (1LL<<7));

	// Second order temperature compensation
	int64_t OFF2 = 0LL;
	int64_t SENSE2 = 0LL;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate first order            D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			);

	if (tempCentiC < 2000L) {
		int32_t tempSquare20 = tempCentiC - 2000L;
		tempSquare20 *= tempSquare20;

		if (tempCentiC <= 1000L) {
			OFF2 = (35LL * tempSquare20) / 8LL;
			SENSE2 = (63LL * tempSquare20) / 32LL;
		} else {
			OFF2 = (30LL * tempSquare20) / 256LL;
		}

	} // if (TEMP < 2000)

	OFF -= OFF2;
	SENS -= SENSE2;

	int64_t P = (((int64_t(D1)*SENS) / (1LL<<21)) - OFF)  / (1LL<<15);

	pressureVal = FloatType(P) / 100.0f;

	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Calculate Pressure Second order: D1 = " << D1
			<< ", OFF = " << OFF
			<< ", SENS = " << SENS
			<< ", dT = " << deltaTemp
			<< ", TEMP = " << tempCentiC
			<< ", OFF2 = " << OFF2
			<< ", SENSE2 = " << SENSE2
			<< ", P = " << P
			);

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal);
}

#if TE_MEAS_ABS_PRESSURE_TEST_MODE

void MS5839_MS5840Driver::testFillPromData() {
	promArray[1] = 46372;
	promArray[2] = 43981;
	promArray[3] = 29059;
	promArray[4] = 27842;
	promArray[5] = 31553;
	promArray[6] = 28165;
}

void MS5839_MS5840Driver::testGetTemperatureVal(uint8_t data[]) {
	uint32_t val = 8077636;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

void MS5839_MS5840Driver::testGetPressureVal(uint8_t data[]) {
	uint32_t val = 6465444;

	data[0] = uint8_t((val >> 16) & 0xffU);
	data[1] = uint8_t((val >> 8) & 0xffU);
	data[2] = uint8_t(val & 0xffU);
}

#endif // #if TE_MEAS_ABS_PRESSURE_TEST_MODE

} /* namespace openEV */
