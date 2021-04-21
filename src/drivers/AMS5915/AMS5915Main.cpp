/*
 * AMS5915Main.cpp
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
#  include <config.h>
#endif

#include "kalman/GliderVarioStatus.h"
#include "AMS5915Lib.h"
#include "AMS5915Driver.h"
#include "drivers/sensorDriver.h"

using namespace openEV::drivers::AMS5915;

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.AMS5915");
	}
}
#endif /* defined HAVE_LOG4CXX_H */

extern "C" {

void OEV_DRIVER_PUBLIC driverLibInit(void) {

	static bool initialized = false;

	if (initialized) {
		return;
	}

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif

	LOG4CXX_INFO(logger,"Initialize the AMS5915 driver library");

	initialized = true;

}


openEV::drivers::GliderVarioDriverLibBasePtr OEV_DRIVER_PUBLIC getDriverLib() {

	driverLibInit();

	LOG4CXX_INFO(logger,"AMS5915: getDriverLib");

	return &AMS5915Lib::theOneAndOnly;
}


}
