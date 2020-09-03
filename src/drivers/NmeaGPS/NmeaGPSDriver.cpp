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

}


NmeaGPSDriver::~NmeaGPSDriver() {

	/// todo fill me

}


void NmeaGPSDriver::driverInit(GliderVarioMainPriv &varioMain) {

	/// todo fill me

}

void NmeaGPSDriver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_DEBUG(logger,"Driver" << driverName << " read configuraion");


	try {
		auto portNameConfig = configuration.searchProperty("portName");

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
		throw;
	}

    errorTimeout = configuration.getPropertyValue(std::string("errorTimeout"),(long long)(10));
    errorMaxNumRetries = configuration.getPropertyValue(std::string("errorTimeout"),(long long)(0));


}

void NmeaGPSDriver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain) {

	/// todo fill me

}

void NmeaGPSDriver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	// Nothing to do here

}


void NmeaGPSDriver::driverThreadFunction() {
	int numRetries = 0;

	if (ioPort == nullptr) {
		LOG4CXX_ERROR (logger,"No valid I/O port for driver " << getDriverName()
				<< ". The driver is not operable");
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				ioPort->open();
				numRetries = 0;
				processingMainLoop ();
				ioPort->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,"Error in main loop of driver \"" << getDriverName()
						<< "\":" << e.what());
				ioPort->close();

				sleep(errorTimeout);
			}
		}
	}

}

void NmeaGPSDriver::processingMainLoop () {

	uint8_t buf [128];

	while (!getStopDriverThread()) {
		LOG4CXX_DEBUG (logger,"Driver " << driverName << ": Read max. " << sizeof (buf) << " bytes from the port");

		auto rc = ioPort->read(buf,sizeof (buf));

		LOG4CXX_DEBUG (logger,"Driver " << driverName << ": ioPort->read returned " << rc);

	}
}
} /* namespace openEV */
