
/*
 * GliderVarioDriverBase.cpp
 *
 *  Created on: Oct 30, 2017
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

#define BUILDING_OEV_DRIVER 1

#include <system_error>
#include <sstream>
#include <typeinfo>

#include "CommonDefs.h"
#include "drivers/DriverBase.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.GliderVarioDriverBase");
	}
}
#endif

namespace openEV::drivers {

DriverBase::DriverBase (
	    char const *driverName,
		char const *description,
		char const *instanceName,
		DriverLibBase &driverLib
		)
: sensorCapabilities {0},
  driverName {driverName},
  description {description},
  instanceName {instanceName},
  driverLib {driverLib}
{
	initLogger();
}

void DriverBase::startup(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

	if (!isDriverThreadRunning && !driverThread.joinable()){
		driverThread = std::thread(DriverBase::driverThreadEntry,this);
	}

}


void DriverBase::driverThreadEntry (DriverBase* tis) {

	tis->isDriverThreadRunning = true;
	try {
		tis->driverThreadFunction();
	}
	catch (std::exception &e) {
		std::ostringstream str;
		str << "Uncaught exception in driver "
				<< tis->driverName << ":" << tis->instanceName
				<< ". Message = " << e.what();
		LOG4CXX_ERROR(logger,str.str());
	}
	catch (...) {
		std::ostringstream str;
		str << "Uncaught unknown exception in driver "
				<< tis->driverName << ":" << tis->instanceName;
		LOG4CXX_ERROR(logger,str.str());
	}

	tis->isDriverThreadRunning = false;
	tis->stopDriverThread = false;
}


void DriverBase::run() {

	isKalmanUpdateRunning = true;

}

void DriverBase::suspend() {

	isKalmanUpdateRunning = false;

}

void DriverBase::resume() {

	isKalmanUpdateRunning = true;

}

void DriverBase::shutdown() {

	if (!stopDriverThread) {
		stopDriverThread = true;
		if (driverThread.joinable()) {
			try {
				driverThread.join();

			} catch  (std::system_error& e) {
				;
			}
		}
	}

	varioMain = nullptr;

}

void DriverBase::readOrCreateConfigValue(
		Properties4CXX::Properties* calibrationDataParameters,
		char const* parameterName,
		double& value
		) {

	try {
		Properties4CXX::Property const * prop = calibrationDataParameters->searchProperty(parameterName);
		value = prop->getDoubleValue();
	} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
		calibrationDataParameters->addProperty(new Properties4CXX::PropertyDouble(parameterName,value));
	}
	catch (std::exception const &e) {}

}

void DriverBase::writeConfigValue (
		Properties4CXX::Properties* calibrationDataParameters,
		char const* parameterName,
		double value
		) {
	calibrationDataParameters->deletePropery(parameterName);
	calibrationDataParameters->addProperty(new Properties4CXX::PropertyDouble(parameterName,value));
}


void DriverBase::updateCalibrationData() {
	auto lastPredictionUpdate = varioMain->getLastPredictionUpdate();
	auto timeSinceLastCalibrationWrite = lastPredictionUpdate - lastCalibrationDataWriteTime;
	if (!calibrationWriterRunning && (timeSinceLastCalibrationWrite >= calibrationDataWriteInterval)) {
		calibrationWriterRunning = true;
		if (calibrationDataWriteThread.joinable()) {
			calibrationDataWriteThread.join();
		}
		lastCalibrationDataWriteTime = OEVClock::now();
		calibrationDataWriteThread = std::thread(&DriverBase::calibrationDataWriteFunc,this);
	}
}

void DriverBase::calibrationDataWriteFunc() {



	try {
		fillCalibrationDataParameters ();
	} catch (std::exception const &e) {
		LOG4CXX_ERROR(logger,"Error in " << __PRETTY_FUNCTION__
				<< ". Exception in fillCalibrationDataParameters (). Error = " << e.what());
	}
	catch (...) {
		LOG4CXX_ERROR(logger,"Error in " << __PRETTY_FUNCTION__
				<< ". Exception in fillCalibrationDataParameters (). Unknown exception");
	}

	try {
		std::ofstream of(calibrationDataFileName,of.out | of.trunc);
		if (of.good()) {
			calibrationDataParameters->writeOut(of);
		}
	} catch (std::exception const &e) {
		LOG4CXX_ERROR(logger,"Error in " << __PRETTY_FUNCTION__
				<< ". Cannot write calibration data. Error = " << e.what());
	}
	catch (...) {
		LOG4CXX_ERROR(logger,"Error in " << __PRETTY_FUNCTION__
				<< ". Cannot write calibration data. Unknown exception");
	}

	lastCalibrationDataWriteTime = OEVClock::now();

	calibrationWriterRunning = false;
}

void DriverBase::applyCalibrationData() {
	LOG4CXX_WARN(logger, "Cannot apply calibration data from file \""
			<< calibrationDataFileName
			<< "\" to device \"" << instanceName
			<< "\". Driver \"" << driverName
			<< "\" does not implement reading of calibration data.");
}
void DriverBase::fillCalibrationDataParameters () {
	LOG4CXX_WARN(logger, "Cannot read calibration data from device \""
			<< instanceName
			<< "\" for writing to update calibration data file \""
			<< calibrationDataUpdateFileName
			<< "\". Driver \"" << driverName
			<< "\" does not implement reading of calibration data.");
}

#if !defined DOXYGEN
DriverBase::SensorCapabilityHelperClass DriverBase::SensorCapabilityHelperObj;
#endif

void DriverBase::readCalibrationData() {

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

	}

}


void DriverBase::readCommonConfiguration(
			const Properties4CXX::Properties &configuration) {

	try {
		auto portNameConfig = configuration.searchProperty("portName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"PortName\" is a struct or a string list.");
		}

		portName = portNameConfig->getStringValue();

	} catch (std::exception const& e) {
		// A not found port name is not necessary issue since not all drivers require it.
		// A missing port name is dealt with later in the driver when necessary.
		if (typeid(e) != typeid(Properties4CXX::ExceptionPropertyNotFound)) {
			LOG4CXX_ERROR(logger, "Read configuration of portName \"" << portName
					<< "\" failed:"
					<< e.what());
		throw;
		}
	}

	try {
		long long durTicks;

		portName = configuration.getPropertyValue(
				std::string("portName"),
				"");

		{
			auto durMSec = std::chrono::duration_cast<std::chrono::milliseconds>(updateCyle);
			durTicks = durMSec.count();
		}
		durTicks = configuration.getPropertyValue(
				std::string("updateCycle"),
				durTicks);
		updateCyle = static_cast<OEVDuration>(std::chrono::milliseconds(durTicks));

		{
		auto durSec = std::chrono::duration_cast<std::chrono::seconds>(errorTimeout);
		durTicks = durSec.count();
		}
		durTicks = configuration.getPropertyValue(
				std::string("errorTimeout"),
				durTicks);
		errorTimeout = static_cast<OEVDuration>(std::chrono::seconds(durTicks));

		errorMaxNumRetries = configuration.getPropertyValue(
				std::string("errorMaxNumRetries"),
				static_cast<long long>(errorMaxNumRetries));

		calibrationDataFileName = configuration.getPropertyValue(
				std::string("calibrationDataFile"),
				"");
		useCalibrationDataFile = !calibrationDataFileName.empty();

		calibrationDataFileName = configuration.getPropertyValue(
				std::string("calibrationDataFile"),
				"");
		durTicks = configuration.getPropertyValue(
				std::string("calibrationDataUpdateCycle"),0LL);
		calibrationDataWriteInterval = static_cast<OEVDuration>(std::chrono::seconds(durTicks));
		useCalibrationDataFile = !calibrationDataFileName.empty() && durTicks > 0LL;

		loadCalibrationDataUpdateFileBeforeStatic = configuration.getPropertyValue(
				std::string("loadCalibrationDataUpdateFileBeforeStatic"),
				loadCalibrationDataUpdateFileBeforeStatic);

	} catch (std::exception const& e) {
		std::ostringstream str;

		str << "Read configuration of device \"" << instanceName
				<< "\" failed:"
				<< e.what();

		LOG4CXX_ERROR(logger, str.str().c_str());
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.str().c_str());
	}




}

}

std::ostream& operator << (std::ostream &o,openEV::drivers::DriverBase::SensorCapability ind) {
	o << openEV::drivers::DriverBase::SensorCapabilityHelperObj.getString (ind);
	return o;
}
