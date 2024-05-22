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
#  include "config.h"
#endif

#include <fstream>
#include <chrono>
#include <thread>

#include "fmt/format.h"

#include "AMS5915Driver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

namespace openEV::drivers::AMS5915 {

AMS5915Driver::AMS5915Driver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: DifferentialPressureSensorBase {driverName,description,instanceName,AMS5915Lib::theOneAndOnly}
{
#if defined HAVE_LOG4CXX_H
	logger = log4cxx::Logger::getLogger("openEV.Drivers.AMS5915");
#endif /* HAVE_LOG4CXX_H */
}


AMS5915Driver::~AMS5915Driver() {}

void AMS5915Driver::fillCalibrationDataParameters () {

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

void AMS5915Driver::applyCalibrationData(){

	double pressureBiasD = pressureBias;
	readOrCreateConfigValue(*calibrationDataParameters,pressureBiasCalibrationName,pressureBiasD);
	pressureBias = FloatType(pressureBiasD);
	LOG4CXX_DEBUG (logger,__PRETTY_FUNCTION__ << ": Device " << instanceName
			<< " Read pressure bias from calibration data = " << pressureBias);
}


void AMS5915Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, fmt::format (_("{0}: for driver instance \"{1}\""),
			__PRETTY_FUNCTION__, instanceName));

	std::string propertyName;

	try {
		propertyName = "pMin";
		auto configPMin = configuration.searchProperty(propertyName);
		pMin = configPMin->getDoubleValue();

		propertyName = "pMax";
		auto configPMax = configuration.searchProperty(propertyName);
		pMax = configPMax->getDoubleValue();

		propertyName = "i2cAddress";
		i2cAddress = (long long)(configuration.getPropertyValue(
				propertyName,
				(long long)(i2cAddress)));

		propertyName = "useTemperatureSensor";
		useTemperatureSensor = configuration.getPropertyValue(
				propertyName,
				useTemperatureSensor);
	} catch (std::exception const &e) {
		auto str = fmt::format(_(
				"Could not read property \"{0}\" for device instance \"{1}\" because: {2}"),
				propertyName,instanceName,e.what());
		LOG4CXX_ERROR(logger, str);
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
	}

    pressureRange = fabs(pMax - pMin);
    pressureResolution = (pMax - pMin) / (AMS5915PressureRangeMaxCount - AMS5915PressureRangeMinCount);
	// Expected static error is assessed by the full measurement range.
	pressureErrorStatic = pressureErrorStaticFactor * pressureRange;

    LOG4CXX_INFO(logger,fmt::format(_("Driver instance \"{0}\" data: "),instanceName));
    LOG4CXX_INFO(logger,"	pMin (mBar) = " << pMin);
    LOG4CXX_INFO(logger,"	pMax (mBar)= " << pMax);
    LOG4CXX_INFO(logger,"	pressureRange (mBar) = " << pressureRange);
    LOG4CXX_INFO(logger,"	pressureResolution (mBar/bit) = " << pressureResolution);
    LOG4CXX_INFO(logger,"	pressureErrorStatic (mBar) = " << pressureErrorStatic);
	LOG4CXX_INFO(logger,"	portName = " << portName);
	LOG4CXX_INFO(logger,"	i2cAddress = 0x" << std::hex <<  uint32_t(i2cAddress) << std::dec);
	LOG4CXX_INFO(logger,"	useTemperatureSensor = " << useTemperatureSensor);
	LOG4CXX_INFO(logger,"	errorTimeout = " << ((errorTimeout.count() * decltype(errorTimeout)::period::num) / decltype(errorTimeout)::period::den));
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);
	if(calibrationDataParameters) {
		LOG4CXX_INFO(logger,fmt::format(_("	Calibration data file name = {0}"), calibrationDataFileName));
	}

}

void AMS5915Driver::driverThreadFunction() {

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
				// ioPort->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,fmt::format(_("Error in the main loop of driver instance \"{0}\": {1}"),
						instanceName,e.what()));
				// ioPort->close();

				std::this_thread::sleep_for(errorTimeout);
			}
		}
	}
}

void AMS5915Driver::processingMainLoop() {
	auto nextStartConversion = OEVClock::now();

	while (!getStopDriverThread()) {

		//std::this_thread::sleep_until(nextStartConversion + 60ms);

		readoutAMS5915();

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
