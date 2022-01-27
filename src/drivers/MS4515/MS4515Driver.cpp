/*
 * MS4515Driver.cpp
 *
 *  Created on: Jan 17 2021
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
#  include "config.h"
#endif

#include <fstream>
#include <chrono>
#include <thread>

#include "MS4515Driver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.MS4515");
	}
}

#endif

namespace openEV::drivers::MS4515 {

MS4515Driver::MS4515Driver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: DriverBase {driverName,description,instanceName,MS4515Lib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(DYNAMIC_PRESSURE);

	// Default cycle time as documented in the template parameter file
	using namespace std::chrono_literals;
	updateCyle = 100ms;

}


MS4515Driver::~MS4515Driver() {

}


void MS4515Driver::driverInit(GliderVarioMainPriv &varioMain) {

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

void MS4515Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, __FUNCTION__ << " Driver" << driverName << " read configuraion");

	try {
		auto portNameConfig = configuration.searchProperty("portName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			std::ostringstream str;

			str << "Invalid configuration of driver " << driverName
					<< ": Variable \"portName\" is a struct or a string list but not a plain string.";

			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
		}

		portName = portNameConfig->getStringValue();

		ioPort = dynamic_cast<io::I2CPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			std::ostringstream str;

			str << "Invalid configuration of driver " << driverName
					<< ": I/O Port \""<< portName << "\" is not an I2C port.";

			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not an I2C port.");
		}
	} catch (std::exception const& e) {
		std::ostringstream str;

		str << "Read configuration \"portName\" of driver \"" << driverName
						<< "\" failed:" << e.what();

		LOG4CXX_ERROR(logger,str.str());
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
	}

	try {
		auto sensorTypeConfig =  configuration.searchProperty("sensorType");

		if (sensorTypeConfig->isList() || sensorTypeConfig->isStruct()) {
			std::ostringstream str;

			str << "Invalid configuration of driver " << driverName
					<< ": Variable \"sensorType\" is a struct or a string list but not a plain string.";

			LOG4CXX_ERROR (logger, str.str());

			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"sensorType\" is a struct or a string list.");
		}

		// Be generous. Accept lowercase letter 'a' as valid sensor type too.
		if (sensorTypeConfig->getStringValue().compare("A") || sensorTypeConfig->getStringValue().compare("a")) {
			sensorType = SENSOR_TYPE_A;
		}

		// Be generous. Accept lowercase letter 'b' as valid sensor type too.
		if (sensorTypeConfig->getStringValue().compare("B") || sensorTypeConfig->getStringValue().compare("b")) {
			sensorType = SENSOR_TYPE_B;
		}

		if (sensorType == SENSOR_TYPE_UNDEFINED) {
			std::ostringstream str;
			str << __FUNCTION__ << ": Configuration value of \"sensorType\" for driver \"" << driverName
					<< "\" is :\"" << sensorTypeConfig->getStringValue() << "\". Valid values are 'A' or 'B'.";
			LOG4CXX_ERROR (logger, str.str());
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
		}

	} catch (std::exception const& e) {
		std::ostringstream str;

		str << "Read configuration \"sensorType\" for driver \"" << driverName
				<< "\" failed: "
				<< e.what();
		LOG4CXX_ERROR(logger, str.str());
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
	}

	{
		double tmpVal;

		try {
			auto tmpConfig = configuration.searchProperty("pMin_inH2O");

			try {
				pMin = tmpConfig->getDoubleValue() * InchH20toMBar;
			} catch (std::exception const &e) {
				std::ostringstream str;

				str << "Read \"pMin_inH2O\" configuration for driver \"" << driverName
						<< "\" failed: " << e.what();
				LOG4CXX_ERROR(logger, str.str());
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
			}

		} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
			; // Ignore the missing parameter. It can also come in the guise of "pMin_hPa".
		}

		try {
			auto tmpConfig = configuration.searchProperty("pMin_hPa");

			if (UnInitVal != pMin) {
				std::ostringstream str;

				str << "Invalid configuration of pMin configuration for driver \"" << driverName
						<< "\" failed: Both configurations \"pMin_inH2O\" and \"pMin_hPa\" are defined. "
						"Only one of these two is allowed.";
				LOG4CXX_ERROR(logger, str.str());
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
			}

			try {
				pMin = tmpConfig->getDoubleValue();
			} catch (std::exception const &e) {
				std::ostringstream str;

				str << "Read \"pMin_hPa\" configuration for driver \"" << driverName
						<< "\" failed: " << e.what();
				LOG4CXX_ERROR(logger, str.str());
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
			}
		} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
			; // Ignore the missing parameter here. Check the absence of pMin below.
		}

		if (UnInitVal == pMin) {
			std::ostringstream str;
			str << "Read pMin configuration for driver \"" << driverName
					<< "\" failed: Neither configurations \"pMin_inH2O\" and \"pMin_hPa\" are defined.";
			LOG4CXX_ERROR(logger, str.str());
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
		}

#error Change reading config for pMax too.
		tmpVal = configuration.getPropertyValue("pMax_inH2O", UnInitVal);

		if (UnInitVal != tmpVal) {
			pMax = tmpVal * InchH20toMBar;
		}

		tmpVal = configuration.getPropertyValue("pMax_hPa", UnInitVal);
		if (UnInitVal != tmpVal) {
			if (UnInitVal != pMax) {
				std::ostringstream str;

				str << "Read pMax configuration for driver \"" << driverName
						<< "\" failed: Both configurations \"pMax_inH2O\" and \"pMax_hPa\" are defined. "
						"Only one of these is allowed.";
				LOG4CXX_ERROR(logger, str.str());
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
			} else {
				pMax = tmpVal;
			}
		}

		if (UnInitVal == pMax) {
			std::ostringstream str;
			str << "Read pMax configuration for driver \"" << driverName
					<< "\" failed: Either neither configurations \"pMax_inH2O\" and \"pMax_hPa\" are defined, "
					"or the value is not numeric.";
			LOG4CXX_ERROR(logger, str.str());
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
		}

	}

	// See description of convertRegisterPressureToMBar() for these calculations
	{
		double loFact;
		double hiFact;

		if (sensorType == SENSOR_TYPE_A) {
			loFact = 0.1;
			hiFact = 0.8;
		} else {
			loFact = 0.05;
			hiFact = 0.9;
		}

	     f1 = 16383.0*loFact;
	     f2 = (pMax-pMin)/(16383.0*hiFact);
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
    LOG4CXX_INFO(logger,"	sensorType = " << configuration.searchProperty("sensorType")->getStringValue());
    LOG4CXX_INFO(logger,"	pMin (mBar) = " << pMin);
    LOG4CXX_INFO(logger,"	pMax (mBar)= " << pMax);
    LOG4CXX_INFO(logger,"	pressureRange (mBar) = " << pressureRange);
    LOG4CXX_INFO(logger,"	pressureErrorStatic (mBar) = " << pressureErrorStatic);
	LOG4CXX_INFO(logger,"	portName = " << portName);
	LOG4CXX_INFO(logger,"	i2cAddress = 0x" << std::hex <<  uint32_t(i2cAddress) << std::dec);
	LOG4CXX_INFO(logger,"	useTemperatureSensor = " << useTemperatureSensor);
	LOG4CXX_INFO(logger,"	errorTimeout = " << errorTimeout);
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);
	if(calibrationDataParameters) {
		LOG4CXX_INFO(logger,"	Calibration data file name = " << calibrationDataFileName);
	}

    LOG4CXX_DEBUG(logger,"	f1 = " << f1);
    LOG4CXX_DEBUG(logger,"	f2 = " << f2);

}

#define SQUARE(x) ((x)*(x))

void MS4515Driver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain) {

	// Wait for 20 seconds for 16 samples to appear, and a defined temperature value
	for (int i = 0; i < 20; i++) {
		if (numValidInitValues < NumInitValues || std::isnan(temperatureVal)) {
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

		if (std::isnan(pressureBias)) {
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
			if (!std::isnan(varioStatus.lastPressure)) {
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
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_TAS,varioStatus.STATUS_IND_TAS) = 10.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_TAS,varioStatus.STATUS_IND_TAS) =
					SQUARE(3.0) * baseIntervalSec;



	} else {
		LOG4CXX_WARN(logger,__FUNCTION__ << "Could not obtain " << NumInitValues
				<< " valid measurements in a row for 20 seconds. Cannot initialize the Kalman filter state.");

	}

	if (std::isnan(pressureBias)) {
		pressureBias = 0.0f;
	}

}

void MS4515Driver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	// Nothing to do here

}


void MS4515Driver::driverThreadFunction() {

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

void MS4515Driver::processingMainLoop() {

    using namespace std::chrono_literals;

	auto nextStartConversion = OEVClock::now();

	while (!getStopDriverThread()) {

		//std::this_thread::sleep_until(nextStartConversion + 60ms);

		readoutMS4515();

		// In case that you miss a cycle advance to the next cycle
		auto now = OEVClock::now();
		do {
			nextStartConversion += updateCyle;
		} while (nextStartConversion < now);
		std::this_thread::sleep_until(nextStartConversion);
}

}

/// Like in the AVR libraries
#define _BV(x) (1<<(x))

FloatType MS4515Driver::convertRegisterPressureToMBar(
		FloatType registerVal) const {
	FloatType rc;

	rc = (registerVal - f1) * f2 + pMin;

	LOG4CXX_TRACE(logger,__FUNCTION__
			<< ": Register value = " << std::hex << uint16_t(registerVal) << std::dec
			<< "; Pressure = " << rc << " hPa");

	return rc;
}

void MS4515Driver::readoutMS4515() {

	uint8_t sensorValues[4];
	uint8_t status = MS4515_STATUS_STALE;
	uint16_t pressureRawVal;
	uint16_t temperatureRawVal;

	do {
		ioPort->readBlock(i2cAddress, sensorValues, sizeof(sensorValues));
		status = (sensorValues[MS4515_BRIDGE_HIGH] & MS4515_STATUS_MASK) >> MS4515_STATUS_BIT;
		LOG4CXX_TRACE(logger,__FUNCTION__<< " Status = " << MS4515Status(status));
	} while (status == MS4515_STATUS_STALE);
	pressureRawVal =
			(uint16_t(sensorValues[MS4515_BRIDGE_HIGH]) & MS4515_PRESSURE_HIGH_BYTE_MASK) << 8 |
			uint16_t(sensorValues[MS4515_BRIDGE_LOW]);
	temperatureRawVal =
			(uint16_t(sensorValues[MS4515_TEMP_HIGH]) << 8 |
			(uint16_t(sensorValues[MS4515_TEMP_LOW]) & MS4515_TEMPERATURE_LOW_MASK)) >> MS4515_TEMPERATURE_LOW_BIT;

	LOG4CXX_TRACE(logger,__FUNCTION__<< ": status = " << MS4515Status(status)
			<< " values = " << std::hex
			<< uint32_t(sensorValues[0]) << ':' << uint32_t(sensorValues[1]) << ':'
			<< uint32_t(sensorValues[2]) << ':' << uint32_t(sensorValues[3])
			<< ", pressureRaw = 0x" << std::hex << pressureRawVal << " = " << std::dec << pressureRawVal
			<< ", temperatureRawVal = 0x" << std::hex << temperatureRawVal << std::dec << " = " << temperatureRawVal);

	temperatureVal = FloatType(temperatureRawVal) * (50.0f/511.0f) - 50.0f;

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

} /* namespace openEV::drivers::MS4515 */
