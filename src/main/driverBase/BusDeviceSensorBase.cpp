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
	int numRetries = 0;

	if (getIoPortPtr() == nullptr) {
		LOG4CXX_ERROR (logger,fmt::format(_(
				"No valid I/O port for driver instance \"{0}\". The driver is not operable"),instanceName));
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				getIoPortPtr()->open();
				numRetries = 0;
				processingMainLoop ();
				// ioPort->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,fmt::format(_("Error in the main loop of driver instance \"{0}\": {1}"),
						instanceName,e.what()));

				// Do not close here: Particularly I2C ports are shared between multiple sensors.
				// An error on one sensor does not mean that the I2C bus or communications to other sensors is disturbed.
				// When this loop returns to the top ioPort->open() will call close() if necessary and try to re-open the port.
				// When there was no issue with the port, i.e. it is OPEN then noting happens an communications just continues.
				// ioPort->close();

				std::this_thread::sleep_for(errorTimeout);
			}
		}
	}
}

void BusDeviceSensorBase::processingMainLoop() {

	setupSensor();

	while (!getStopDriverThread()) {
		processOneMeasurementCycle();
	}
}

void BusDeviceSensorBase::setupSensor() {

}

} /* namespace drivers */
} /* namespace openEV */
