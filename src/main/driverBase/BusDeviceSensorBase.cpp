/*
 * BusDeviceSensorBase.cpp
 *
 *  Created on: Jun 26, 2024
 *      Author: hor
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

#include "fmt/format.h"

#include "main/driverBase/BusDeviceSensorBase.h"

namespace openEV {
namespace drivers {

BusDeviceSensorBase::~BusDeviceSensorBase() {
}

BusDeviceSensorBase::BusDeviceSensorBase() {

}

void BusDeviceSensorBase::driverThreadFunction() {

	if (getIoPortPtr() == nullptr) {
		LOG4CXX_ERROR (logger,fmt::format(_(
				"No valid I/O port for driver instance \"{0}\". The driver is not operable"),instanceName));
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				getIoPortPtr()->open();
				numRetries = 0;
				processingMainLoop ();
				// getIoPortPtr()->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,fmt::format(_("Error in the main loop of driver instance \"{0}\": {1}"),
						instanceName,e.what()));

				getIoPortPtr()->close();

				std::this_thread::sleep_for(errorTimeout);
			}
		}

		if (getStopDriverThread()){
			LOG4CXX_INFO(logger,fmt::format(_("Driver instance \"{0}\" terminates due to shutdown command."),
				instanceName));
		} else {
			LOG4CXX_ERROR(logger,fmt::format(_("Driver instance \"{0}\" terminates because maximum number of successive error was exceeded."),
				instanceName));
		}
	}
}

void BusDeviceSensorBase::processingMainLoop() {

	setupSensor();

	while (!getStopDriverThread()) {
		try {
			processOneMeasurementCycle();

			numRetries = 0;
			lastErrno = 0;
		} catch (io::GliderVarioPortIOException const & e) {

			if (e.getErrno() == ENXIO) {
				// The target sensor is not reachable on the bus.
				// This is not an issue of the bus device itself but the sensor may not be reachable on its address on the bus
				// Therefore handle this issue here local to the sensor.
				if (e.getErrno() == lastErrno) {
					numRetries ++;
				} else {
					lastErrno = e.getErrno();
					numRetries = 1;
				}

				LOG4CXX_ERROR(logger,fmt::format(_("Error in the main loop of driver instance \"{0}\": {1}"),
						instanceName,e.what()));
				if (errorMaxNumRetries > 0 && numRetries >= errorMaxNumRetries){
					return;
				}

				std::this_thread::sleep_for(errorTimeout);

			} else {
				throw;
			}

		}
	}
}

void BusDeviceSensorBase::setupSensor() {

	// Do nothing here.
	// This method can be overridden when needed by a sensor class.

}

} /* namespace drivers */
} /* namespace openEV */
