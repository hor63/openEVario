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

#include <fmt/format.h>

#include "MS4515Driver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

namespace openEV::drivers::MS4515 {

MS4515Driver::MS4515Driver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: DifferentialPressureSensorBase {driverName,description,instanceName,MS4515Lib::theOneAndOnly}
{
#if defined HAVE_LOG4CXX_H
	logger = log4cxx::Logger::getLogger("openEV.Drivers.MS4515");
#endif /* HAVE_LOG4CXX_H */

  i2cAddress = MS4515DOI2CAddr;

}


MS4515Driver::~MS4515Driver() {}

void MS4515Driver::fillCalibrationDataParameters () {

	if (UnInitVal == pressureBias) {
		writeConfigValue(*calibrationDataParameters, pressureBiasCalibrationName, 0.0);
	} else {
		writeConfigValue(*calibrationDataParameters, pressureBiasCalibrationName, pressureBias);
	}

	// write the 0-bias only once after the status initialization.
	if (statusInitDone) {
		doCyclicUpdateCalibrationDataFile = false;
	}

	LOG4CXX_DEBUG (logger,__PRETTY_FUNCTION__ << ": Device " << instanceName
			<< " Write pressure bias to calibration data = " << pressureBias);
}

void MS4515Driver::applyCalibrationData(){

	double pressureBiasD = pressureBias;
	readOrCreateConfigValue(*calibrationDataParameters,pressureBiasCalibrationName,pressureBiasD);
	pressureBias = FloatType(pressureBiasD);
	LOG4CXX_DEBUG (logger,__PRETTY_FUNCTION__ << ": Device " << instanceName
			<< " Read pressure bias from calibration data = " << pressureBias);
}

void MS4515Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, __FUNCTION__ << " Device" << instanceName << " read configuraion");

	std::string propertyName;

	try {

		propertyName = "sensorType";
		auto sensorTypeConfig =  configuration.searchProperty(propertyName);

		if (sensorTypeConfig->isList() || sensorTypeConfig->isStruct()) {

			throw GliderVarioFatalConfigException(__FILE__,__LINE__,
					_("Property is a struct or a list."));
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
			auto str = fmt::format(_(
					"Value {0} is invalid. Valid values are 'A' or 'B'."),
					sensorTypeConfig->getStringValue());
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
		}

		double factor = 1.0;
		Properties4CXX::Property const *valProperty = nullptr;

		try {
			propertyName = "pMin_inH2O";
			valProperty = configuration.searchProperty(propertyName);

		} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
			; // Ignore the missing parameter. It can also come in the guise of "pMin_hPa".
		}

		try {
			propertyName = "pMin_hPa";
			auto tmpProperty = configuration.searchProperty(propertyName);

			factor = InchH2OtoMBar;

			if (valProperty) {
				auto str = fmt::format(_(
						"Both properties \"{0}\" and \"{1}\" are defined. Only one of these two is allowed."),
						"pMin_inH2O","pMin_hPa");
				LOG4CXX_ERROR(logger, str);
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
			} else {
				valProperty = tmpProperty;
			}

		} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
			if (!valProperty) {
				auto str = fmt::format(_("Neither configurations \"{0}\" and \"{1}\" are defined."),
						"pMin_inH2O","pMin_hPa");
				LOG4CXX_ERROR(logger, str);
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
			}
		}

		try {
			pMin = valProperty->getDoubleValue() * factor;
		} catch (std::exception const &e) {
			// Above I try to read different properties. Determine which one is the one.
			propertyName = valProperty->getPropertyName();
			throw;
		}

		factor = 1.0;
		valProperty = nullptr;

		try {
			propertyName = "pMax_inH2O";
			valProperty = configuration.searchProperty(propertyName);

		} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
			; // Ignore the missing parameter. It can also come in the guise of "pMax_hPa".
		}

		try {
			propertyName = "pMax_hPa";
			auto tmpProperty = configuration.searchProperty(propertyName);

			factor = InchH2OtoMBar;

			if (valProperty) {
				auto str = fmt::format(_(
						"Both properties \"{0}\" and \"{1}\" are defined. Only one of these two is allowed."),
						"pMax_inH2O","pMax_hPa");
				LOG4CXX_ERROR(logger, str);
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
			} else {
				valProperty = tmpProperty;
			}

		} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
			if (!valProperty) {
				auto str = fmt::format(_("Neither configurations \"{0}\" and \"{1}\" are defined."),
						"pMax_inH2O","pMax_hPa");
				LOG4CXX_ERROR(logger, str);
				throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
			}
		}

		try {
			pMax = valProperty->getDoubleValue() * factor;
		} catch (std::exception const &e) {
			// Above I try to read different properties. Determine which one is the one.
			propertyName = valProperty->getPropertyName();
			throw;
		}
	} catch (std::exception const& e) {

		auto str = fmt::format(_("{0}: Read configuration property {1} for driver instance \"{2}\" failed: {3}"),
				__PRETTY_FUNCTION__,propertyName,instanceName, e.what());
		LOG4CXX_ERROR(logger, str);
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
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

    pressureRange = fabs(pMax - pMin);
	// Expected static error is assessed by the full measurement range.
	pressureErrorStatic = pressureErrorStaticFactor * pressureRange;

	i2cAddress = (long long)(configuration.getPropertyValue(
	    		std::string("i2cAddress"),
				(long long)(i2cAddress)));
	useTemperatureSensor = configuration.getPropertyValue(
    		std::string("useTemperatureSensor"),
			useTemperatureSensor);

	LOG4CXX_INFO(logger, fmt::format (_("{0}: for driver instance \"{1}\""),
			__PRETTY_FUNCTION__, instanceName));
    LOG4CXX_INFO(logger,"	sensorType = " << configuration.searchProperty("sensorType")->getStringValue());
    LOG4CXX_INFO(logger,"	pMin (mBar) = " << pMin);
    LOG4CXX_INFO(logger,"	pMax (mBar)= " << pMax);
    LOG4CXX_INFO(logger,"	pressureRange (mBar) = " << pressureRange);
    LOG4CXX_INFO(logger,"	pressureErrorStatic (mBar) = " << pressureErrorStatic);
	LOG4CXX_INFO(logger,"	portName = " << portName);
	LOG4CXX_INFO(logger,"	i2cAddress = 0x" << std::hex <<  uint32_t(i2cAddress) << std::dec);
	LOG4CXX_INFO(logger,"	useTemperatureSensor = " << useTemperatureSensor);
	LOG4CXX_INFO(logger,"	errorTimeout = " << ((errorTimeout.count() * decltype(errorTimeout)::period::num) / decltype(errorTimeout)::period::den));
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);
	if(calibrationDataParameters) {
		LOG4CXX_INFO(logger,fmt::format(_("	Calibration data file name = {0}"), calibrationDataFileName));
	}

    LOG4CXX_DEBUG(logger,"	f1 = " << f1);
    LOG4CXX_DEBUG(logger,"	f2 = " << f2);

}

void MS4515Driver::driverThreadFunction() {

	int numRetries = 0;

	if (ioPort == nullptr) {
		LOG4CXX_ERROR (logger,fmt::format(_(
				"No valid I/O port for driver instance \"{0}\". The driver is not operable"),instanceName));
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				ioPort->open();
				numRetries = 0;
				processingMainLoop ();
				ioPort->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,fmt::format(_("Error in the main loop of driver instance \"{0}\": "),
						instanceName,e.what()));
				ioPort->close();

				std::this_thread::sleep_for(errorTimeout);
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

    LOG4CXX_DEBUG(logger, __FUNCTION__ << ": pressureVal = " << pressureVal);

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

std::ostream& operator << (std::ostream &o, openEV::drivers::MS4515::MS4515Register v) {
    o << openEV::drivers::MS4515::MS4515RegisterHelperObj.getString(v);
    return o;
}
std::ostream& operator << (std::ostream &o, openEV::drivers::MS4515::MS4515Status v) {
    o << openEV::drivers::MS4515::MS4515StatusHelperObj.getString(v);
    return o;
}
#if defined HAVE_LOG4CXX_H
std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::drivers::MS4515::MS4515Register v) {
	std::ostream& o = b;
    o << openEV::drivers::MS4515::MS4515RegisterHelperObj.getString(v);
    return o;
}
std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::drivers::MS4515::MS4515Status v) {
	std::ostream& o = b;
    o << openEV::drivers::MS4515::MS4515StatusHelperObj.getString(v);
    return o;
}
#endif

