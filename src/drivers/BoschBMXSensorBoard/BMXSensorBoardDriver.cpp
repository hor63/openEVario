/*
 * BMXSensorBoardDriver.cpp
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

#include "BMXSensorBoardDriver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

#include "horOvIp-I2C-Bridge/BMX160net.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.BMXSensorBoard");
	}
}

#endif

namespace openEV {

BMXSensorBoardDriver::BMXSensorBoardDriver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,BMXSensorBoardLib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(ACCEL_X);
	setSensorCapability(ACCEL_Y);
	setSensorCapability(ACCEL_Z);
	setSensorCapability(GYRO_X);
	setSensorCapability(GYRO_Y);
	setSensorCapability(GYRO_Z);
	setSensorCapability(COMPASS_X);
	setSensorCapability(COMPASS_Y);
	setSensorCapability(COMPASS_Z);

	memset (&magTrimData,0,sizeof(magTrimData));
}


BMXSensorBoardDriver::~BMXSensorBoardDriver() {


}


void BMXSensorBoardDriver::driverInit() {

	/// todo fill me

}

void BMXSensorBoardDriver::readConfiguration (Properties4CXX::Properties const &configuration) {


	try {
		auto portNameConfig = configuration.searchProperty("PortName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"PortName\" is a struct or a string list.");
		}

		portName = portNameConfig->getStringValue();

		ioPort = dynamic_cast<io::StreamPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not a stream port.");
		}
	} catch (std::exception const& e) {
		LOG4CXX_ERROR(logger, "Read configuration of driver \"" << driverName
				<< "\" failed:"
				<< e.what());
	}

    errorTimeout = (long long)(configuration.getPropertyValue(std::string("errorTimeout"),(long long)(10)));
    errorMaxNumRetries = (long long)(configuration.getPropertyValue(std::string("errorTimeout"),(long long)(0)));


}

void BMXSensorBoardDriver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain) {

	/// todo fill me

}

void BMXSensorBoardDriver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	/// todo fill me

}


void BMXSensorBoardDriver::driverThreadFunction() {

	if (ioPort == nullptr) {
		LOG4CXX_ERROR (logger,"No valid I/O port for driver " << getDriverName()
				<< ". The driver is not operable");
	} else {
		while (1) {
			try {
				ioPort->open();
				/// todo Read from port and update Kalman filter.
			} catch (std::exception const& e) {
				LOG4CXX_ERROR(logger,"Error in main loop of driver \"" << getDriverName()
						<< "\":" << e.what());
				ioPort->close();

			}
		}
	}
}


} /* namespace openEV */
