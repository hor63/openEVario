/*
 * MPU9150Lib.cpp
 *
 *  Created on: Jun 07, 2021
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2021  Kai Horstmann
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

#include "CommonDefs.h"


#include "MPU-9150Lib.h"
#include "MPU-9150Driver.h"

namespace openEV::drivers::TDK_MPU9150 {

static std::string const MPU9150DriverName = "MPU-9150";
static std::string const MPU9150DriverLibName = "MPU9150Lib";

MPU9150Lib MPU9150Lib::theOneAndOnly;


static DriverBase* getNewMPU9150Instance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (MPU9150DriverName.compare(driverName)) {
		return 0;
	}
	return new MPU9150Driver (driverName,description,instanceName);
}

MPU9150Lib::MPU9150Lib()
	: DriverLibBase{
		MPU9150DriverLibName.c_str(),
		"Driver library for TDK/Invensense MPU-9150 IMU"}
{

}


MPU9150Lib::~MPU9150Lib() {

}

void MPU9150Lib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {
		MPU9150DriverName,
		"Driver for TDK/Invensense MPU-9150 IMU",
		getNewMPU9150Instance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
