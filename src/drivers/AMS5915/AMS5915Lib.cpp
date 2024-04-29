/*
 * AMS5915Lib.cpp
 *
 *  Created on: Apr 21 2021
 *      Author: hor
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
#  include "config.h"
#endif

#include "CommonDefs.h"


#include "AMS5915Lib.h"
#include "AMS5915Driver.h"

namespace openEV::drivers::AMS5915 {

static std::string const AMS5915DriverName = "AMS5915";
static std::string const AMS5915DriverLibName = "AMS5915";

AMS5915Lib AMS5915Lib::theOneAndOnly;


static DriverBase* getNewAMS5915Instance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (AMS5915DriverName.compare(driverName)) {
		return 0;
	}
	return new AMS5915Driver (driverName,description,instanceName);
}

AMS5915Lib::AMS5915Lib()
	: DriverLibBase{
		AMS5915DriverLibName.c_str(),
		_("Differential and gage pressure Amsys sensors AMS5915")}
{


}


AMS5915Lib::~AMS5915Lib() {

}

void AMS5915Lib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {
		AMS5915DriverName,
		"Differential and gage sensor AMS5915",
		getNewAMS5915Instance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
