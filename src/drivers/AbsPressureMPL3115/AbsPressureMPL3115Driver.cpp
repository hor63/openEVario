/*
 * AbsPressureMPL3115Driver.cpp
 *
 *  Created on: Feb 04, 2020
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

#include "AbsPressureMPL3115Driver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.MPL3115");
	}
}

#endif

namespace openEV::drivers::AbsPressureMPL3115 {

AbsPressureMPL3115Driver::AbsPressureMPL3115Driver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,AbsPressureMPL3115Lib::theOneAndOnly}
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


AbsPressureMPL3115Driver::~AbsPressureMPL3115Driver() {

	/// todo fill me

}


void AbsPressureMPL3115Driver::driverInit(GliderVarioMainPriv &varioMain) {

	/// todo fill me

}

void AbsPressureMPL3115Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

	/// todo fill me

}

void AbsPressureMPL3115Driver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain) {

	/// todo fill me

}

void AbsPressureMPL3115Driver::run() {

	GliderVarioDriverBase::run();

}

void AbsPressureMPL3115Driver::suspend() {

	/// todo fill me

}

void AbsPressureMPL3115Driver::resume() {

	/// todo fill me

}

void AbsPressureMPL3115Driver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	/// todo fill me

}


void AbsPressureMPL3115Driver::driverThreadFunction() {

	/// todo fill me
}


} /* namespace openEV */
