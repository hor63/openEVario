/*
 * BMXSensorBoardLib.cpp
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

#include "OEVCommon.h"


#include "BMXSensorBoardLib.h"
#include "BMXSensorBoardDriver.h"

namespace openEV::drivers::BoschBMX160 {

static std::string const BMXSensorBoardDriverName = "BMX160SensorBoard";
static std::string const BMXSensorBoardDriverLibName = "BMXSensorBoardLib";

BMXSensorBoardLib BMXSensorBoardLib::theOneAndOnly;


static GliderVarioDriverBase* getNewBMXSensorBoardInstance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (BMXSensorBoardDriverName.compare(driverName)) {
		return 0;
	}
	return new BMXSensorBoardDriver (driverName,description,instanceName);
}

BMXSensorBoardLib::BMXSensorBoardLib()
	: GliderVarioDriverLibBase{
		BMXSensorBoardDriverLibName.c_str(),
		"Driver library for Bosch BMX IMUs on the horImuBoard sensor board"}
{


}


BMXSensorBoardLib::~BMXSensorBoardLib() {

}

void BMXSensorBoardLib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {
		BMXSensorBoardDriverName,
		"Driver for Bosch BMX160 IMUs on the horImuBoard sensor board",
		getNewBMXSensorBoardInstance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
