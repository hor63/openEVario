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


} /* namespace OevGLES */
