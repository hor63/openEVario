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

namespace openEV {

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

	if (!isDriverThreadRunning && !driverThread.joinable()){
		this->varioMain = &varioMain;
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
		str << "Uncought exception in driver "
				<< tis->driverName << ":" << tis->instanceName
				<< ". Message = " << e.what();
		LOG4CXX_ERROR(logger,str.str());
	}
	catch (...) {
		std::ostringstream str;
		str << "Uncought unknown exception in driver "
				<< tis->driverName << ":" << tis->instanceName;
		LOG4CXX_ERROR(logger,str.str());
	}

	tis->isDriverThreadRunning = false;
	tis->stopDriverThread = false;
}


void GliderVarioDriverBase::run(GliderVarioMainPriv &varioMain) {

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

}


#if !defined DOXYGEN
GliderVarioDriverBase::SensorCapabilityHelperClass GliderVarioDriverBase::SensorCapabilityHelperObj;
#endif

}

std::ostream& operator << (std::ostream &o,openEV::GliderVarioDriverBase::SensorCapability ind) {
	o << openEV::GliderVarioDriverBase::SensorCapabilityHelperObj.getString (ind);
	return o;
}


