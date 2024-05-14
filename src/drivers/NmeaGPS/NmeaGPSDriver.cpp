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

#include <fmt/format.h>

#include <fstream>

#include "NmeaGPSDriver.h"
#include "NMEA0813Protocol.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

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
	logger = log4cxx::Logger::getLogger("openEV.Drivers.NmeaGPS");
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(GPS_POSITION	);
	setSensorCapability(GPS_ALTITUDE_MSL);

}


NmeaGPSDriver::~NmeaGPSDriver() {

}


void NmeaGPSDriver::driverInit(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

	nmeaSet.setVarioMain(&varioMain);

	ioPort = getIoPort<decltype(ioPort)>(logger);

}

void NmeaGPSDriver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, fmt::format (_("{0}: for driver instance \"{1}\""),
			__PRETTY_FUNCTION__, instanceName));

	std::string propertyName;

	try {
		propertyName = "CEP";
		CEP = configuration.getPropertyValue(
				propertyName,
				double (CEP));
		propertyName = "altitudeStdDev";
		altStdDev = configuration.getPropertyValue(
				propertyName,
				CEP*1.5);
		propertyName = "maxStdDeviationPositionInitialization";
		maxStdDeviationPositionInitialization = configuration.getPropertyValue(
				propertyName,
				double (maxStdDeviationPositionInitialization));
		propertyName = "maxStdDeviationAltitudeInitialization";
		maxStdDeviationAltitudeInitialization = configuration.getPropertyValue(
				propertyName,
				double (maxStdDeviationAltitudeInitialization));
		propertyName = "maxStdDeviationPositionUpdate";
		maxStdDeviationPositionUpdate = configuration.getPropertyValue(
				propertyName,
				double (maxStdDeviationPositionUpdate));
		propertyName = "maxStdDeviationAltitudeUpdate";
		maxStdDeviationAltitudeUpdate = configuration.getPropertyValue(
				propertyName,
				double (maxStdDeviationAltitudeUpdate));
	} catch (std::exception const &e) {
		auto str = fmt::format(_(
				"Could not read property \"{0}\" for device instance \"{1}\" because: {2}"),
				propertyName,instanceName,e.what());
		LOG4CXX_ERROR(logger, str);
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
	}

	LOG4CXX_INFO(logger,"	errorTimeout = " << ((errorTimeout.count() * decltype(errorTimeout)::period::num) / decltype(errorTimeout)::period::den));
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
		LOG4CXX_ERROR (logger,fmt::format(_(
				"No valid I/O port for driver instance \"{0}\". The driver is not operable"),instanceName));
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				ioPort->open();
				numRetries = 0;
				processingMainLoop ();
				ioPort->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,fmt::format(_("Error in the main loop of driver instance \"{0}\": {1}"),
						instanceName,e.what()));
				ioPort->close();

				std::this_thread::sleep_for(errorTimeout);
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
