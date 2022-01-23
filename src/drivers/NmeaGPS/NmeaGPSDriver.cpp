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
#  include "config.h"
#endif

#include <fstream>

#include "NmeaGPSDriver.h"
#include "NMEA0813Protocol.h"
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

namespace openEV::drivers::NMEA0813 {

NmeaGPSDriver::NmeaGPSDriver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: DriverBase {driverName,description,instanceName,NmeaGPSLib::theOneAndOnly},
  nmeaSet{*this}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(GPS_LATITUDE	);
	setSensorCapability(GPS_LONGITUDE	);
	setSensorCapability(GPS_ALTITUDE_MSL);

}


NmeaGPSDriver::~NmeaGPSDriver() {

}


void NmeaGPSDriver::driverInit(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

	nmeaSet.setVarioMain(&varioMain);

}

void NmeaGPSDriver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger,"Driver" << driverName << " read configuraion");

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

    errorTimeout = configuration.getPropertyValue(
    		std::string("errorTimeout"),
			(long long)(errorTimeout));
    errorMaxNumRetries = configuration.getPropertyValue(
    		std::string("errorMaxNumRetries"),
			(long long)(errorMaxNumRetries));
    CEP = configuration.getPropertyValue(
    		std::string("CEP"),
			double (CEP));
    altStdDev = configuration.getPropertyValue(
    		std::string("altitudeStdDev"),
			CEP*1.5);
    maxStdDeviationPositionInitialization = configuration.getPropertyValue(
    		std::string("maxStdDeviationPositionInitialization"),
			double (maxStdDeviationPositionInitialization));
    maxStdDeviationAltitudeInitialization = configuration.getPropertyValue(
    		std::string("maxStdDeviationAltitudeInitialization"),
			double (maxStdDeviationAltitudeInitialization));
	maxStdDeviationPositionUpdate = configuration.getPropertyValue(
			std::string("maxStdDeviationPositionUpdate"),
			double (maxStdDeviationPositionUpdate));
	maxStdDeviationAltitudeUpdate = configuration.getPropertyValue(
			std::string("maxStdDeviationAltitudeUpdate"),
			double (maxStdDeviationAltitudeUpdate));

	LOG4CXX_INFO(logger,"	errorTimeout = " << errorTimeout);
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);
	LOG4CXX_INFO(logger,"	CEP = " << CEP);
	LOG4CXX_INFO(logger,"	altStdDev = " << altStdDev);
	LOG4CXX_INFO(logger,"	maxStdDeviationPositionInitialization = " << maxStdDeviationPositionInitialization);
	LOG4CXX_INFO(logger,"	maxStdDeviationAltitudeInitialization = " << maxStdDeviationAltitudeInitialization);
	LOG4CXX_INFO(logger,"	maxStdDeviationPositionUpdate = " << maxStdDeviationPositionUpdate);
	LOG4CXX_INFO(logger,"	maxStdDeviationAltitudeUpdate = " << maxStdDeviationAltitudeUpdate);

}

void NmeaGPSDriver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain) {

	/// Initialization is done later. GPS acquisition can take minutes.

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

	uint8_t buf [NMEASentence::maxLenSentence];
	NMEA0813Protocol nmeaProcessor(nmeaSet);

	while (!getStopDriverThread()) {
		LOG4CXX_TRACE (logger,"Driver " << driverName << ": Read max. " << sizeof (buf) << " bytes from the port");

		auto rc = ioPort->read(buf,sizeof(buf)-1);
		if (rc > 0 && rc < long(sizeof(buf))) {
			buf [rc] = 0;
			LOG4CXX_TRACE (logger,"Driver " << driverName << ": ioPort->read returned '" << buf << '\'');
			// Consume and process received data.
			nmeaProcessor.processSensorData(buf,rc);
		} else {
			LOG4CXX_DEBUG (logger,"Driver " << driverName << ": ioPort->read returned " << rc);
			// Either nothing was received or something implausible
			// Ignore and/or throw away.
		}

	}
}
} /* namespace openEV */
