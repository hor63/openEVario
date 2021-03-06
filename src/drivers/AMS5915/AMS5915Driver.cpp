/*
 * AMS5915Driver.cpp
 *
 *  Created on: Apr 21 2021
 *      Author: hor
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <fstream>
#include <chrono>
#include <thread>

#include "AMS5915Driver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.AMS5915");
	}
}

#endif

namespace openEV::drivers::AMS5915 {

AMS5915Driver::AMS5915Driver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,AMS5915Lib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(DYNAMIC_PRESSURE);

}


AMS5915Driver::~AMS5915Driver() {

}


void AMS5915Driver::driverInit(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

	// Read the calibration data file, and extract the initial parameters
	if (calibrationDataParameters) {
		try {
			calibrationDataParameters->readConfiguration();
		} catch (std::exception const &e) {
			LOG4CXX_ERROR(logger,"Driver " << driverName
					<< ": Error reading calibration data from file " << calibrationDataFileName
					<< ": " << e.what());
			// The file does not exist, or it has unexpected/undefined content.
			// Therefore I am initializing the calibration parameters fresh.
			delete calibrationDataParameters;
			calibrationDataParameters = new Properties4CXX::Properties(calibrationDataFileName);

		}

		double pressureBiasD = pressureBias;
		readOrCreateConfigValue(calibrationDataParameters,pressureBiasCalibrationName,pressureBiasD);
		pressureBias = FloatType(pressureBiasD);
		LOG4CXX_DEBUG (logger,__FUNCTION__ << ": Driver " << getDriverName()
				<< " Read pressure bias from calibration data = " << pressureBias);
	}

}

void AMS5915Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, __FUNCTION__ << " Driver" << driverName << " read configuraion");

	try {
		auto portNameConfig = configuration.searchProperty("portName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"portName\" is a struct or a string list.");
		}

		portName = portNameConfig->getStringValue();

		ioPort = dynamic_cast<io::I2CPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not an I2C port.");
		}
	} catch (std::exception const& e) {
		LOG4CXX_ERROR(logger, "Read configuration \"portName\" of driver \"" << driverName
				<< "\" failed:"
				<< e.what());
		throw;
	}

	{

		pMin = configuration.getPropertyValue("pMin", NAN);
		if (isnan(pMin)) {
			std::ostringstream str;
			str << "Read pMin configuration for driver \"" << driverName
					<< "\" failed: Either configurations \"pMin\" is not defined, "
					"or the value is not numeric.";
			LOG4CXX_ERROR(logger, str.str());
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
		}


		pMax = configuration.getPropertyValue("pMax", NAN);
		if (isnan(pMax)) {
			std::ostringstream str;
			str << "Read pMax configuration for driver \"" << driverName
					<< "\" failed: Either configurations \"pMax\" is not defined, "
					"or the value is not numeric.";
			LOG4CXX_ERROR(logger, str.str());
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
		}

	}

    try {
    	auto fileNameProp = configuration.searchProperty("calibrationDataFile");

		if (!fileNameProp->isList() && !fileNameProp->isStruct()) {
	    	calibrationDataFileName = fileNameProp->getStringValue();
	    	calibrationDataParameters = new Properties4CXX::Properties(calibrationDataFileName);
		}

    } catch (...) {
    	LOG4CXX_INFO(logger,"Driver" << driverName << ": No calibration data file specified");
    }

    pressureRange = fabs(pMax - pMin);
    pressureResolution = (pMax - pMin) / (AMS5915PressureRangeMaxCount - AMS5915PressureRangeMinCount);
	// Expected static error is assessed by the full measurement range.
	pressureErrorStatic = pressureErrorStaticFactor * pressureRange;

	i2cAddress = (long long)(configuration.getPropertyValue(
	    		std::string("i2cAddress"),
				(long long)(i2cAddress)));
	useTemperatureSensor = configuration.getPropertyValue(
    		std::string("useTemperatureSensor"),
			useTemperatureSensor);
    errorTimeout = configuration.getPropertyValue(
    		std::string("errorTimeout"),
			(long long)(errorTimeout));
    errorMaxNumRetries = configuration.getPropertyValue(
    		std::string("errorMaxNumRetries"),
			(long long)(errorMaxNumRetries));

    LOG4CXX_INFO(logger,"Driver" << driverName << " data>");
    LOG4CXX_INFO(logger,"	pMin (mBar) = " << pMin);
    LOG4CXX_INFO(logger,"	pMax (mBar)= " << pMax);
    LOG4CXX_INFO(logger,"	pressureRange (mBar) = " << pressureRange);
    LOG4CXX_INFO(logger,"	pressureResolution (mBar/bit) = " << pressureResolution);
    LOG4CXX_INFO(logger,"	pressureErrorStatic (mBar) = " << pressureErrorStatic);
	LOG4CXX_INFO(logger,"	portName = " << portName);
	LOG4CXX_INFO(logger,"	i2cAddress = 0x" << std::hex <<  uint32_t(i2cAddress) << std::dec);
	LOG4CXX_INFO(logger,"	useTemperatureSensor = " << useTemperatureSensor);
	LOG4CXX_INFO(logger,"	errorTimeout = " << errorTimeout);
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);
	if(calibrationDataParameters) {
		LOG4CXX_INFO(logger,"	Calibration data file name = " << calibrationDataFileName);
	}

}

#define SQUARE(x) ((x)*(x))

void AMS5915Driver::initializeStatus(
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
		FloatType initialTAS = 0.0f;
		double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

		for (int i = 0 ; i < NumInitValues; i++) {
			avgPressure += FloatType(initValues[i]);
			LOG4CXX_TRACE(logger," initValues[" << i << "] = " << initValues[i]);
		}
		avgPressure /= FloatType(NumInitValues);
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": avgPressure = " << avgPressure << " mBar");

		// Store the avg pressure as offset only when the instrument is obviously not switched on during flight.
		// or during high-wind conditions on the field (> 20 km/h)

		if (isnan(pressureBias)) {
			// No pre-loaded bias value from calibration data.
			// Assume initial startup in controlled environment.
			pressureBias = avgPressure;
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": No bias from calibration data available. pressureBias = " << pressureBias << " mBar");
		} else {

			// Dynamic pressure in mBar at about 20km/h on the ground at 0C. Variations at different temperatures
			// and atmospheric pressures are insignificant here.
			// I only want a threshold to differentiate between switching the device on in high-wind conditions or even in flight.
			static constexpr FloatType PressureLimit = 0.2;

			if (fabs(avgPressure - pressureBias) < PressureLimit) {
				// Not too far off.
				// Assume the measured value is the new offset/bias of the sensor.
				LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Old Pressure bias = " << pressureBias << ", new pressureBias = " << avgPressure << " mBar");
				pressureBias = avgPressure;
			}

		}

		if (pressureBias == avgPressure && calibrationDataParameters) {
			// The bias has been updated or freshly set.
			try {

				writeConfigValue(calibrationDataParameters, pressureBiasCalibrationName, pressureBias);

				std::ofstream of(calibrationDataFileName,of.out | of.trunc);
				if (of.good()) {
					calibrationDataParameters->writeOut(of);
					LOG4CXX_DEBUG(logger,__FUNCTION__ << ": written new avg. measurement as bias.");
				}
			} catch (std::exception const &e) {
				LOG4CXX_ERROR(logger,"Error in " << __PRETTY_FUNCTION__
						<< ". Cannot write calibration data. Error = " << e.what());
			}
			catch (...) {}
		} else {
			// There is a significant pressure on the sensor.
			// Convert it into into IAS. On the ground this is approximately TAS
			// When there is already an actual pressure value available, even better.
			FloatType currStaticPressure;
			if (!isnan(varioStatus.lastPressure)) {
				currStaticPressure = varioStatus.lastPressure;
			} else {
				currStaticPressure = PressureStdMSL;
			}

			FloatType airDensity = currStaticPressure*100.0f / Rspec / (temperatureVal + CtoK);
			initialTAS = sqrtf(200.0f * avgPressure / airDensity);

			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": TAS @ "
					<< temperatureVal << "C, " << currStaticPressure << "mBar = "
					<< initialTAS << "m/s.");
		}

		// All data is collected. Initialize the status
		varioStatus.trueAirSpeed = initialTAS;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_TAS,varioStatus.STATUS_IND_TAS) = 100.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_TAS,varioStatus.STATUS_IND_TAS) =
					SQUARE(2.0) * baseIntervalSec;



	} else {
		LOG4CXX_WARN(logger,__FUNCTION__ << "Could not obtain " << NumInitValues
				<< " valid measurements in a row for 20 seconds. Cannot initialize the Kalman filter state.");

	}

	if (isnan(pressureBias)) {
		pressureBias = 0.0f;
	}

}

void AMS5915Driver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	// Nothing to do here

}


void AMS5915Driver::driverThreadFunction() {

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

void AMS5915Driver::processingMainLoop() {
    std::chrono::system_clock::time_point nextStartConversion;

    using namespace std::chrono_literals;

	nextStartConversion = std::chrono::system_clock::now();

	while (!getStopDriverThread()) {

		//std::this_thread::sleep_until(nextStartConversion + 60ms);

		readoutAMS5915();
		nextStartConversion += 20ms;
		std::this_thread::sleep_until(nextStartConversion);
	}

}

/// Like in the AVR libraries
#define _BV(x) (1<<(x))

FloatType AMS5915Driver::convertRegisterPressureToMBar(
		uint16_t registerVal) const {
	FloatType rc;

	rc = FloatType(registerVal - AMS5915PressureRangeMinCount) * pressureResolution + pMin;

	LOG4CXX_TRACE(logger,__FUNCTION__
			<< ": Register value = " << std::hex << registerVal << std::dec
			<< "; Pressure = " << rc << " hPa");

	return rc;
}

void AMS5915Driver::readoutAMS5915() {

	uint8_t statusVal = 0;
	uint8_t sensorValues[4];
	uint16_t pressureRawVal;
	uint16_t temperatureRawVal;

	ioPort->readBlock(i2cAddress, sensorValues, sizeof(sensorValues));
	pressureRawVal =
			uint16_t(sensorValues[AMS5915_PRESSURE_HIGH] & AMS5915_PRESSURE_HIGH_BYTE_MASK) << 8 |
			uint16_t(sensorValues[AMS5915_PRESSURE_LOW]);
	temperatureRawVal =
			(uint16_t(sensorValues[AMS5915_TEMP_HIGH]) << 8 |
			uint16_t(sensorValues[AMS5915_TEMP_LOW] & AMS5915_TEMP_LOW_BYTE_MASK)) >> AMS5915_TEMP_SHIFT_COUNT;

	LOG4CXX_TRACE(logger,__FUNCTION__<< ": values = " << std::hex
			<< uint32_t(sensorValues[0]) << ':' << uint32_t(sensorValues[1]) << ':'
			<< uint32_t(sensorValues[2]) << ':' << uint32_t(sensorValues[3])
			<< ", pressureRaw = 0x" << std::hex << pressureRawVal << " = " << std::dec << pressureRawVal
			<< ", temperatureRawVal = 0x" << std::hex << temperatureRawVal << std::dec << " = " << temperatureRawVal);

	// Formula directly taken from data sheet Rev. 3.1, pg. 11
	temperatureVal = FloatType(temperatureRawVal) * (200.0f/2048.0f) - 50.0f;

	LOG4CXX_DEBUG(logger, __FUNCTION__ << ": temperatureVal = " << temperatureVal);

    FloatType pressureVal = convertRegisterPressureToMBar(pressureRawVal);

	if (getIsKalmanUpdateRunning()) {

		// When the sensor is saturated skip the measurement
		if (pressureVal >= pMin && pressureVal <= pMax) {
			GliderVarioMainPriv::LockedCurrentStatus lockedStatus(*varioMain);
			FloatType &tempLocalC = lockedStatus.getMeasurementVector()->tempLocalC;

			if (useTemperatureSensor) {
				tempLocalC = temperatureVal;
			}

			pressureVal -= pressureBias;

			FloatType pressureVariance = fabs(pressureVal) * pressureErrorDynFactor + pressureErrorStatic;
			pressureVariance = pressureVariance * pressureVariance;
			LOG4CXX_TRACE(logger,__FUNCTION__<< ": pressureVariance = " << pressureVariance);

			GliderVarioMeasurementUpdater::dynamicPressureUpd(pressureVal, tempLocalC, pressureVariance,
					*lockedStatus.getMeasurementVector(), *lockedStatus.getCurrentStatus());
		} // if (pressureVal >= pMin && pressureVal <= pMax)
	} else {
		// When the sensor is saturated reset collecting initial values
		if (pressureVal >= pMin && pressureVal <= pMax) {
			if (numValidInitValues < NumInitValues) {
				initValues[numValidInitValues] = pressureVal;
				numValidInitValues ++;
			}
		} else {
			numValidInitValues = 0;
		}
	}


}

} /* namespace openEV::drivers::AMS5915 */
