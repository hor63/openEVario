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
#  include <config.h>
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
: GliderVarioDriverBase {driverName,description,instanceName,MS4515Lib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(STATIC_PRESSURE	);

}


MS4515Driver::~MS4515Driver() {

}


void MS4515Driver::driverInit(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

}

void MS4515Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

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

	try {
		auto sensorTypeConfig =  configuration.searchProperty("sensorType");

		if (sensorTypeConfig->isList() || sensorTypeConfig->isStruct()) {
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
			std::stringstream str;
			str << __FUNCTION__ << ": Configuration value of \"sensorType\" for driver \"" << driverName
					<< "\" is :\"" << sensorTypeConfig->getStringValue() << "\". Valid values are 'A' or 'B'.";
			LOG4CXX_ERROR (logger, str.str());
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
		}

	} catch (std::exception const& e) {
		std::ostringstream str;

		str << "Read configuration \"sensorType\" of driver \"" << driverName
				<< "\" failed: "
				<< e.what();
		LOG4CXX_ERROR(logger, str.str());
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
	}

	{
		double tmpVal;

		tmpVal = configuration.getPropertyValue("pMin_inH2O", NAN);

		if (!isnan(tmpVal)) {
			pMin = tmpVal * InchH20toMBar;
		}

		tmpVal = configuration.getPropertyValue("pMin_hPa", NAN);
		if (!isnan(tmpVal)) {
			if (!isnan(pMin)) {
				std::ostringstream str;

				str << "Read pMin configuration for driver \"" << driverName
						<< "\" failed: Both configurations \"pMin_inH2O\" and \"pMin_hPa\" are defined. "
						"Only one of these is allowed.";
				LOG4CXX_ERROR(logger, str.str());
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
			} else {
				pMin = tmpVal;
			}
		}

		if (isnan(pMin)) {
			std::ostringstream str;
			str << "Read pMin configuration for driver \"" << driverName
					<< "\" failed: Either neither configurations \"pMin_inH2O\" and \"pMin_hPa\" are defined, "
					"or the value is not numeric.";

		}


		tmpVal = configuration.getPropertyValue("pMax_inH2O", NAN);

		if (!isnan(tmpVal)) {
			pMax = tmpVal * InchH20toMBar;
		}

		tmpVal = configuration.getPropertyValue("pMax_hPa", NAN);
		if (!isnan(tmpVal)) {
			if (!isnan(pMax)) {
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

		if (isnan(pMax)) {
			std::ostringstream str;
			str << "Read pMax configuration for driver \"" << driverName
					<< "\" failed: Either neither configurations \"pMax_inH2O\" and \"pMax_hPa\" are defined, "
					"or the value is not numeric.";

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
    		std::string("errorTimeout"),
			(long long)(errorMaxNumRetries));

    LOG4CXX_INFO(logger,"	sensorType = " << configuration.searchProperty("sensorType")->getStringValue());
    LOG4CXX_INFO(logger,"	pMin = " << pMin);
    LOG4CXX_INFO(logger,"	pMax = " << pMax);
	LOG4CXX_INFO(logger,"	portName = " << portName);
	LOG4CXX_INFO(logger,"	i2cAddress = 0x" << std::hex <<  uint32_t(i2cAddress) << std::dec);
	LOG4CXX_INFO(logger,"	useTemperatureSensor = " << useTemperatureSensor);
	LOG4CXX_INFO(logger,"	errorTimeout = " << errorTimeout);
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);

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
    std::chrono::system_clock::time_point nextStartConversion;

    using namespace std::chrono_literals;

	nextStartConversion = std::chrono::system_clock::now();

	while (!getStopDriverThread()) {

		//std::this_thread::sleep_until(nextStartConversion + 60ms);

		readoutMS4515();
		nextStartConversion += 20ms;
		std::this_thread::sleep_until(nextStartConversion);
	}

}

/// Like in the AVR libraries
#define _BV(x) (1<<(x))



void MS4515Driver::readoutMS4515() {

	uint8_t statusVal = 0;
	uint8_t sensorValues[4];
	uint8_t status;
	uint16_t pressureRawVal;
	uint16_t temperatureRawVal;

	ioPort->readBlock(i2cAddress, sensorValues, sizeof(sensorValues));
	status = (sensorValues[MS4515_BRIDGE_HIGH] & MS4515_STATUS_MASK) >> MS4515_STATUS_BIT;
	if (status == MS4515_STATUS_STALE) {
		LOG4CXX_TRACE(logger,__FUNCTION__<< " stale status. Read again");
		ioPort->readBlock(i2cAddress, sensorValues, sizeof(sensorValues));
		status = (sensorValues[MS4515_BRIDGE_HIGH] & MS4515_STATUS_MASK) >> MS4515_STATUS_BIT;
	}
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

}

} /* namespace openEV::drivers::MS4515 */
