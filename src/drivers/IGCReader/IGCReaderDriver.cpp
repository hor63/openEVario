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

#include "drivers/IGCReader/IGCReaderDriver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.IGCReader");
	}
}

#endif

namespace openEV::drivers::IGCReader {

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


void IGCReaderDriver::driverInit(GliderVarioMainPriv &varioMain) {

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

	if (runSingleThreadDebug ) {
		setSensorCapability(RUN_IDLE_LOOP);
	} else {
		clearSensorCapability(RUN_IDLE_LOOP);
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
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain) {

	openIGCFile();

	readIGCFile ();

	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	auto firstRec = bRecords.cbegin();

#define SQUARE(x) ((x)*(x))

	if (firstRec != bRecords.cend()) {
		varioStatus.altMSL = firstRec->second.altGPS;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) = 1000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) =
				SQUARE(15.0) * baseIntervalSec;


		varioStatus.longitude(firstRec->second.longitude);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_LONGITUDE_OFFS,varioStatus.STATUS_IND_LONGITUDE_OFFS) = 1000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_LONGITUDE_OFFS,varioStatus.STATUS_IND_LONGITUDE_OFFS) =
				SQUARE(10.0) * baseIntervalSec;

		varioStatus.latitude(firstRec->second.latitude);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_LATITUDE_OFFS,varioStatus.STATUS_IND_LATITUDE_OFFS) = 1000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_LATITUDE_OFFS,varioStatus.STATUS_IND_LATITUDE_OFFS) =
				SQUARE(10.0) * baseIntervalSec;


		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) == 0.0f) {
			varioStatus.heading = calcInitialHeading();
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) = 10.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) =
					SQUARE(5.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_QFF,varioStatus.STATUS_IND_QFF) == 0.0f) {
			// calculated factor to calculate the pressure at altGPS with a temperature lapse of 1K/100m
			double factAltGPS = altToPressureStdTemp(firstRec->second.altGPS,-0.01) / PressureStdMSL;
			double factBaroHeight = altToPressureStdTemp(firstRec->second.altBaro) / PressureStdMSL;

			varioStatus.qff = PressureStdMSL * factBaroHeight / factAltGPS;
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_QFF,varioStatus.STATUS_IND_QFF) = 100.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_QFF,varioStatus.STATUS_IND_QFF) =
					SQUARE(0.1) * baseIntervalSec; // Veryyyy slow (10mbar / 100sec)
		}

#undef SQUARE

	}

}

void IGCReaderDriver::startup(GliderVarioMainPriv &varioMain) {

	if (!runSingleThreadDebug) {
		GliderVarioDriverBase::startup(varioMain);
	} else {
		// store the pointer here. I need it for run()
		this->varioMain = &varioMain;
	}

}

void IGCReaderDriver::run() {

	if (runSingleThreadDebug) {
		runDebugSingleThread (*varioMain);
	}
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

	std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point lastUpdateTime = startTime;
	GliderVarioTransitionMatrix transMatrix;

	GliderVarioStatus **currentStatus = 0;
	GliderVarioStatus **nextStatus = 0;
	GliderVarioTransitionMatrix *transitionMatrix = 0;
	GliderVarioMeasurementVector *measurementVector = 0;
	FloatType idleLoopIncrement = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	varioMain.getAndLockInternalStatusForDebug(
			currentStatus,
			nextStatus,
			transitionMatrix,
			measurementVector);

	startTime = std::chrono::system_clock::now();
	lastUpdateTime = startTime;

	for (auto it = bRecords.cbegin(); it != bRecords.cend();it++) {

		BRecord const &bRecord = it->second;

		while (lastUpdateTime < (startTime + bRecord.timeSinceStart) ) {

			transMatrix.updateStatus(**currentStatus,**nextStatus,idleLoopIncrement);
			lastUpdateTime += varioMain.getProgramOptions().idlePredictionCycle;

			GliderVarioStatus *tmp = *currentStatus;
			*currentStatus = *nextStatus;
			*nextStatus = tmp;

		}

		if (bRecord.gpsIsValid) {
			double posVariance = bRecord.posAccuracy * bRecord.posAccuracy;
			GliderVarioMeasurementUpdater::GPSLatitudeUpd(bRecord.latitude,posVariance,*measurementVector,**currentStatus);
			(*currentStatus)->normalizeStatus();
			GliderVarioMeasurementUpdater::GPSLongitudeUpd(bRecord.longitude,posVariance,*measurementVector,**currentStatus);
			(*currentStatus)->normalizeStatus();
			GliderVarioMeasurementUpdater::GPSAltitudeUpd(bRecord.altGPS,bRecord.altGPSAccuracy*bRecord.altGPSAccuracy,*measurementVector,**currentStatus);
			(*currentStatus)->normalizeStatus();

		}

		GliderVarioMeasurementUpdater::staticPressureUpd(
				bRecord.pressure,
				25.0f-(bRecord.altBaro/100.0f), // Assume I fly at 25 ground temp, and 1°C/100m temperature lapse.
				1.5f,  // assume about 10m inaccuracy
				*measurementVector,
				**currentStatus);
		(*currentStatus)->normalizeStatus();


	}

	varioMain.releaseCurrentStatus();

}

FloatType IGCReaderDriver::calcInitialHeading() {

	FloatType rc = 0.0f;

	auto it = bRecords.cbegin();

	if (it != bRecords.cend()) {

		auto prevLatM = it->second.latitude * 3600 * LEN_LAT_ARC_SEC;
		auto prevLonM = it->second.longitude * 3600 * LEN_LAT_ARC_SEC * FastMath::fastCos(it->second.latitude) ;
		auto prevDurSec = (std::chrono::duration_cast<std::chrono::duration<double>>(it->second.timeSinceStart)).count();

		for (++it;it != bRecords.cend();++it) {
			auto latM = it->second.latitude * 3600 * LEN_LAT_ARC_SEC;
			auto lonM = it->second.longitude * 3600 * LEN_LAT_ARC_SEC * FastMath::fastCos(it->second.latitude);
			auto diffLonM = lonM - prevLonM;
			auto diffLatM = latM - prevLatM;

			auto durSec = (std::chrono::duration_cast<std::chrono::duration<double>>(it->second.timeSinceStart)).count();

			auto diffDurSec = durSec - prevDurSec;

			auto speedMperSec = sqrt(diffLatM*diffLatM + diffLonM*diffLonM) / diffDurSec;

			if (speedMperSec > 10.0) {
				rc = FastMath::fastATan2(diffLonM,diffLatM);
				break;
			}

			prevLatM = latM;
			prevLonM = lonM;
			prevDurSec = durSec;

		}
	}
	return rc;

}

} /* namespace openEV */
