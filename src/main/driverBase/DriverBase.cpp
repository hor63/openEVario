
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

}

std::ostream& operator << (std::ostream &o,openEV::drivers::DriverBase::SensorCapability ind) {
	o << openEV::drivers::DriverBase::SensorCapabilityHelperObj.getString (ind);
	return o;
}


