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
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.IGCReader");
	}
#endif /* HAVE_LOG4CXX_H */

}


IGCReaderDriver::~IGCReaderDriver() {

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

}

void IGCReaderDriver::initializeStatus(GliderVarioStatus &varioStatus) {

	openIGCFile();

}

void IGCReaderDriver::start(GliderVarioMainPriv *varioMain) {
	this->varioMain = varioMain;

}

void IGCReaderDriver::stop() {

	varioMain = 0;
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
	}

}

void IGCReaderDriver::closeIGCFile() {

	if (igcFile.is_open()) {
		igcFile.close();
	}

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

	}

	lineNum ++;

	return true;

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

#warning Initialize the Kalman filter status with the first B record.

			startTimeDay = std::chrono::duration_cast<std::chrono::duration<double,std::ratio<1,1>>>(bRecord.timeSinceStart).count();

			// Now process the record again with the correct start time.
			bRecordSectionProcessor.processBRecord(lineBuffer,lineLen,bRecord,startTimeDay);
			bRecords.insert(BRecordsValue(bRecord.timeSinceStart,bRecord));
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

} /* namespace OevGLES */
