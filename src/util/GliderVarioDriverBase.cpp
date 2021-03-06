/*
 * GliderVarioDriverBase.cpp
 *
 *  Created on: Oct 30, 2017
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2017  Kai Horstmann
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

#define BUILDING_OEV_DRIVER 1

#include <system_error>
#include <sstream>

#include "OEVCommon.h"
#include "drivers/GliderVarioDriverBase.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.GliderVarioDriverBase");
	}
}
#endif

namespace openEV::drivers {

GliderVarioDriverBase::GliderVarioDriverBase (
	    char const *driverName,
		char const *description,
		char const *instanceName,
		GliderVarioDriverLibBase &driverLib
		)
: sensorCapabilities {0},
  driverName {driverName},
  description {description},
  instanceName {instanceName},
  driverLib {driverLib}
{
	initLogger();
}

void GliderVarioDriverBase::startup(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

	if (!isDriverThreadRunning && !driverThread.joinable()){
		driverThread = std::thread(GliderVarioDriverBase::driverThreadEntry,this);
	}

}


void GliderVarioDriverBase::driverThreadEntry (GliderVarioDriverBase* tis) {

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


void GliderVarioDriverBase::run() {

	isKalmanUpdateRunning = true;

}

void GliderVarioDriverBase::suspend() {

	isKalmanUpdateRunning = false;

}

void GliderVarioDriverBase::resume() {

	isKalmanUpdateRunning = true;

}

void GliderVarioDriverBase::shutdown() {

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

void GliderVarioDriverBase::readOrCreateConfigValue(
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

void GliderVarioDriverBase::writeConfigValue (
		Properties4CXX::Properties* calibrationDataParameters,
		char const* parameterName,
		double value
		) {
	calibrationDataParameters->deletePropery(parameterName);
	calibrationDataParameters->addProperty(new Properties4CXX::PropertyDouble(parameterName,value));
}




#if !defined DOXYGEN
GliderVarioDriverBase::SensorCapabilityHelperClass GliderVarioDriverBase::SensorCapabilityHelperObj;
#endif

}

std::ostream& operator << (std::ostream &o,openEV::drivers::GliderVarioDriverBase::SensorCapability ind) {
	o << openEV::drivers::GliderVarioDriverBase::SensorCapabilityHelperObj.getString (ind);
	return o;
}


