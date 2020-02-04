/*
 * IGCReaderLib.cpp
 *
 *  Created on: Aug 15, 2018
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2017  Kai Horstmann
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

#include "OEVCommon.h"


#include "IGCReaderLib.h"
#include "IGCReaderDriver.h"

namespace openEV {

static std::string const igcReaderDriverName = "IGCReader";
static std::string const igcReaderDriverLibName = "IGCReaderLib";

IGCReaderLib IGCReaderLib::theOneAndOnly;


static GliderVarioDriverBase* getNewIGCReaderInstance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (igcReaderDriverName.compare(driverName)) {
		return 0;
	}
	return new IGCReaderDriver (driverName,description,instanceName);
}

IGCReaderLib::IGCReaderLib()
	: GliderVarioDriverLibBase{igcReaderDriverLibName.c_str(),"Driver library for simulation and test drivers based on IGC file recordings"}
{


}


IGCReaderLib::~IGCReaderLib() {

}

void IGCReaderLib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {igcReaderDriverName,"Simulation and test driver reading recorded flight data from an IGC file",getNewIGCReaderInstance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
