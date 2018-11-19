/*
 * IGCReaderDriver.cpp
 *
 *  Created on: Aug 15, 2018
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2018  Kai Horstmann
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

#include "IGCReaderDriver.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.IGCReader");
	}
}

#endif

namespace openEV {

IGCReaderDriver::IGCReaderDriver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,IGCReaderLib::theOneAndOnly}
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


IGCReaderDriver::~IGCReaderDriver() {

	closeIGCFile();

}


void IGCReaderDriver::driverInit() {

}

void IGCReaderDriver::readConfiguration (Properties4CXX::Properties const &configuration) {

	Properties4CXX::Property const *fileNameProp;

	try {

		fileNameProp = configuration.searchProperty("file");


	} catch (Properties4CXX::ExceptionBase const &e) {
		throw GliderVarioDriverLoadException(__FILE__,__LINE__,e.what());
	}

	igcFileName = fileNameProp->getStringValue();

	LOG4CXX_DEBUG (logger,"readConfiguration: Property \"file\" returns \"" << igcFileName << "\"");

	Properties4CXX::Property const *runSingleThreadDebugProp;

	try {

		runSingleThreadDebugProp = configuration.searchProperty("runSingleThreadDebug");

		runSingleThreadDebug = runSingleThreadDebugProp->getBoolValue();

	} catch (Properties4CXX::ExceptionBase const &e) {
	    runSingleThreadDebug = false;
	}


	LOG4CXX_DEBUG (logger,"readConfiguration: Property \"runSingleThreadDebug\" returns \"" << runSingleThreadDebug << "\"");


	Properties4CXX::Property const *runInRealTimeProp;

	try {

		runInRealTimeProp = configuration.searchProperty("runInRealTime");

		runInRealTime = runInRealTimeProp->getBoolValue();

	} catch (Properties4CXX::ExceptionBase const &e) {
		runInRealTime = false;
	}


	LOG4CXX_DEBUG (logger,"readConfiguration: Property \"runInRealTime\" returns \"" << runInRealTime << "\"");


}

void IGCReaderDriver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain) {

	openIGCFile();

	readIGCFile ();

	double baseIntervalSec = std::chrono::duration_cast<std::chrono::duration<double>>(varioMain.getProgramOptions().idlePredictionCycle).count() ;

	auto firstRec = bRecords.cbegin();

#define SQUARE(x) ((x)*(x))

	if (firstRec != bRecords.cend()) {
		varioStatus.altMSL = firstRec->second.altGPS;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) = 1000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) =
				SQUARE(4.0) * baseIntervalSec;

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_VERTICAL_SPEED,varioStatus.STATUS_IND_VERTICAL_SPEED) == 0.0f) {
			varioStatus.verticalSpeed = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_VERTICAL_SPEED,varioStatus.STATUS_IND_VERTICAL_SPEED) = 100.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_VERTICAL_SPEED,varioStatus.STATUS_IND_VERTICAL_SPEED) =
					SQUARE(3.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_THERMAL_SPEED,varioStatus.STATUS_IND_THERMAL_SPEED) == 0.0f) {
			varioStatus.thermalSpeed = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_THERMAL_SPEED,varioStatus.STATUS_IND_THERMAL_SPEED) = 100.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_THERMAL_SPEED,varioStatus.STATUS_IND_THERMAL_SPEED) =
					SQUARE(3.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_RATE_OF_SINK,varioStatus.STATUS_IND_RATE_OF_SINK) == 0.0f) {
			varioStatus.rateOfSink = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_RATE_OF_SINK,varioStatus.STATUS_IND_RATE_OF_SINK) = 50.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_RATE_OF_SINK,varioStatus.STATUS_IND_RATE_OF_SINK) =
					SQUARE(3.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ACC_VERTICAL,varioStatus.STATUS_IND_ACC_VERTICAL) == 0.0f) {
			varioStatus.accelVertical = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ACC_VERTICAL,varioStatus.STATUS_IND_ACC_VERTICAL) = 4.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ACC_VERTICAL,varioStatus.STATUS_IND_ACC_VERTICAL) =
					SQUARE(10.0) * baseIntervalSec;
		}


		varioStatus.longitude(firstRec->second.longitude);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_LONGITUDE_OFFS,varioStatus.STATUS_IND_LONGITUDE_OFFS) = 10000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_LONGITUDE_OFFS,varioStatus.STATUS_IND_LONGITUDE_OFFS) =
				SQUARE(3.0) * baseIntervalSec;

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_SPEED_GROUND_E,varioStatus.STATUS_IND_SPEED_GROUND_E) == 0.0f) {
			varioStatus.groundSpeedEast = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_SPEED_GROUND_E,varioStatus.STATUS_IND_SPEED_GROUND_E) = 100.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_SPEED_GROUND_E,varioStatus.STATUS_IND_SPEED_GROUND_E) =
					SQUARE(2.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_WIND_SPEED_E,varioStatus.STATUS_IND_WIND_SPEED_E) == 0.0f) {
			varioStatus.windSpeedEast = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_WIND_SPEED_E,varioStatus.STATUS_IND_WIND_SPEED_E) = 100.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_WIND_SPEED_E,varioStatus.STATUS_IND_WIND_SPEED_E) =
					SQUARE(3.0) * baseIntervalSec;
		}

		varioStatus.latitude(firstRec->second.latitude);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_LATITUDE_OFFS,varioStatus.STATUS_IND_LATITUDE_OFFS) = 10000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_LATITUDE_OFFS,varioStatus.STATUS_IND_LATITUDE_OFFS) =
				SQUARE(3.0) * baseIntervalSec;


		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_SPEED_GROUND_N,varioStatus.STATUS_IND_SPEED_GROUND_N) == 0.0f) {
			varioStatus.groundSpeedNorth = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_SPEED_GROUND_N,varioStatus.STATUS_IND_SPEED_GROUND_N) = 100.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_SPEED_GROUND_N,varioStatus.STATUS_IND_SPEED_GROUND_N) =
					SQUARE(2.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_WIND_SPEED_N,varioStatus.STATUS_IND_WIND_SPEED_N) == 0.0f) {
			varioStatus.windSpeedNorth = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_WIND_SPEED_N,varioStatus.STATUS_IND_WIND_SPEED_N) = 100.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_WIND_SPEED_N,varioStatus.STATUS_IND_WIND_SPEED_N) =
					SQUARE(3.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_TAS,varioStatus.STATUS_IND_TAS) == 0.0f) {
			varioStatus.trueAirSpeed = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_TAS,varioStatus.STATUS_IND_TAS) = 100.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_TAS,varioStatus.STATUS_IND_TAS) =
					SQUARE(3.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ACC_HEADING,varioStatus.STATUS_IND_ACC_HEADING) == 0.0f) {
			varioStatus.accelHeading = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ACC_HEADING,varioStatus.STATUS_IND_ACC_HEADING) = 4.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ACC_HEADING,varioStatus.STATUS_IND_ACC_HEADING) =
					SQUARE(3.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ACC_CROSS,varioStatus.STATUS_IND_ACC_CROSS) == 0.0f) {
			varioStatus.accelCross = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ACC_CROSS,varioStatus.STATUS_IND_ACC_CROSS) = 1.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ACC_CROSS,varioStatus.STATUS_IND_ACC_CROSS) =
					SQUARE(2.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) == 0.0f) {
			varioStatus.heading = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) = 90.0f * 90.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) =
					SQUARE(10.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ROTATION_Z,varioStatus.STATUS_IND_ROTATION_Z) == 0.0f) {
			varioStatus.yawRateZ = 0.0f;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROTATION_Z,varioStatus.STATUS_IND_ROTATION_Z) = 10.0f * 10.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ROTATION_Z,varioStatus.STATUS_IND_ROTATION_Z) =
					SQUARE(20.0) * baseIntervalSec;
		}

		// calculated factor to calculate the pressure at altGPS with a temperature lapse of 1K/100m
		double factAltGPS = altToPressureStdTemp(firstRec->second.altGPS,0.01) / P0StdAtmosphere;
		double factBaroHeight = altToPressureStdTemp(firstRec->second.altBaro) / P0StdAtmosphere;

		varioStatus.qff = P0StdAtmosphere * factBaroHeight / factAltGPS;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_QFF,varioStatus.STATUS_IND_QFF) = 100.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_QFF,varioStatus.STATUS_IND_QFF) =
				SQUARE(0.01) * baseIntervalSec; // Veryyyy slow (1mbar / 100sec)

#undef SQUARE

	}

}

void IGCReaderDriver::start(GliderVarioMainPriv &varioMain) {

	if (runSingleThreadDebug) {
		runDebugSingleThread (varioMain);
	} else {
		GliderVarioDriverBase::start(varioMain);
	}

}

void IGCReaderDriver::suspend() {

}

void IGCReaderDriver::resume() {

}

void IGCReaderDriver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

}

void IGCReaderDriver::openIGCFile() {

	if (!igcFile.is_open()) {
		igcFile.open(igcFileName.c_str(),std::ios_base::in|std::ios_base::binary);
		if (!igcFile) {
			std::ostringstream os;
			os << "Cannot open IGC file " << igcFileName;
			LOG4CXX_ERROR(logger,os.str());
			throw (GliderVarioDriverLoadException(__FILE__,__LINE__,os.str().c_str()));
		}

		LOG4CXX_INFO(logger,"Opened IGC file \""<< igcFileName << "\" successfully");
	}

}

void IGCReaderDriver::closeIGCFile() {

	if (igcFile.is_open()) {
		igcFile.close();
	}

	LOG4CXX_INFO(logger,"Closed IGC file \""<< igcFileName << "\".");

}

bool IGCReaderDriver::readLine() {

	// If a line without line terminator was the last line in the file the previous call returned true,
	// and the function is re-entered. The failbit is still set.
	if (!igcFile) {
		return false;
	}
	// read characters from the file until
	// - the file end is reached
	// - you hit a line terminator CR-LF
	for (lineLen = 0; lineLen < lineBufSize - 1; lineLen++) {
		int c = igcFile.get();

		if (!igcFile) {
			// read error or ordinary EOF
			lineBuffer [lineLen] = '\0';
			if (lineLen == 0) {
				return false;
			}

			break;
		}

		// line terminator
		if (c == '\r') {
			lineBuffer [lineLen] = '\0';

			// according to the specification lines are terminated with CR-LF.
			// When I find CR I expect LF too.
			c = igcFile.peek();
			if (c == '\n') {
				c = igcFile.get();
			}

			break;
		}
		if (c == '\n') {
			lineBuffer [lineLen] = '\0';

			break;
		}

		lineBuffer [lineLen] = char(c);
	}

	lineNum ++;

	return true;

}

void IGCReaderDriver::driverThreadFunction() {
}

void IGCReaderDriver::readIGCFile() {

	BRecord bRecord;

	if (!igcFile.is_open()) {
		openIGCFile();
	}

	// Read the first BRecord to initialize the start time of records in the IGC file,
	// and the Kalman status.
	while (readLine()) {
		if (*lineBuffer == 'I') {
			bRecordSectionProcessor.processIRecord(lineBuffer,lineLen);
		}
		if (*lineBuffer == 'B') {
			bRecordSectionProcessor.processBRecord(lineBuffer,lineLen,bRecord,startTimeDay);

			startTimeDay = std::chrono::duration_cast<std::chrono::duration<double>>(bRecord.timeSinceStart).count();

			// Now process the record again with the correct start time.
			bRecordSectionProcessor.processBRecord(lineBuffer,lineLen,bRecord,startTimeDay);
			bRecords.insert(BRecordsValue(bRecord.timeSinceStart,bRecord));

			// I have the inital B record, and with that the start time.
			// continue with the next loop.
			break;
		}
	}

	// Now continue reading the IGC file to the end
	while (readLine()) {
		if (*lineBuffer == 'I') {
			bRecordSectionProcessor.processIRecord(lineBuffer,lineLen);
		}
		if (*lineBuffer == 'B') {
			bRecordSectionProcessor.processBRecord(lineBuffer,lineLen,bRecord,startTimeDay);
			bRecords.insert(BRecordsValue(bRecord.timeSinceStart,bRecord));
		}
	}

}

void IGCReaderDriver::runDebugSingleThread(GliderVarioMainPriv& varioMain) {
}

} /* namespace OevGLES */
