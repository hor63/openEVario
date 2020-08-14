/*
 * NmeaGPSDriver.cpp
 *
 *  Created on: Aug 13, 2020
 *      Author: kai_horstmann
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

#include "NmeaGPSDriver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.NmeaGPS");
	}
}

#endif

namespace openEV {

NmeaGPSDriver::NmeaGPSDriver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,NmeaGPSLib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(GPS_LATITUDE	);
	setSensorCapability(GPS_LONGITUDE	);
	setSensorCapability(GPS_ALTITUDE_MSL);
	setSensorCapability(STATIC_PRESSURE	);
	setSensorCapability(RUN_IDLE_LOOP	);

}


NmeaGPSDriver::~NmeaGPSDriver() {

	/// todo fill me

}


void NmeaGPSDriver::driverInit(GliderVarioMainPriv &varioMain) {

	/// todo fill me

}

void NmeaGPSDriver::readConfiguration (Properties4CXX::Properties const &configuration) {

	/// todo fill me

}

void NmeaGPSDriver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain) {

	/// todo fill me

}

void NmeaGPSDriver::run() {

	GliderVarioDriverBase::run();

}

void NmeaGPSDriver::suspend() {

	/// todo fill me

}

void NmeaGPSDriver::resume() {

	/// todo fill me

}

void NmeaGPSDriver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	/// todo fill me

}


void NmeaGPSDriver::driverThreadFunction() {

	/// todo fill me
}


} /* namespace openEV */
