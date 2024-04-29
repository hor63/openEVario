/* Simulation driver main module
 *
 * simulMain.cpp
 *
 *  Created on: Sep 24, 2017
 *      Author: hor
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
#  include "config.h"
#endif

#include "fmt/format.h"

#include "kalman/GliderVarioStatus.h"
#include "simulDriver.h"
#include "drivers/sensorDriver.h"

using namespace openEV::drivers;

static log4cxx::LoggerPtr logger = 0;

static void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.Simul");
	}
}

extern "C" {

void OEV_DRIVER_PUBLIC driverLibInit(void) {

	initLogger();

	LOG4CXX_INFO(logger,fmt::format(_("Initialize the {0} driver library"),"Simulation"));

}

GliderVarioDriverLibBasePtr OEV_DRIVER_PUBLIC getDriverLib() {

	initLogger();

	LOG4CXX_INFO(logger,"Simulation: getDriverLib()");

	return 0;
}


}
