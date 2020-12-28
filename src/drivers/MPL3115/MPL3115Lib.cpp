/*
 * MPL3115Lib.cpp
 *
 *  Created on: Dec 28, 2020
 *      Author: hor
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


#include "MPL3115Lib.h"
#include "MPL3115Driver.h"

namespace openEV::drivers::MPL3115 {

static std::string const MPL3115A2DriverName = "MPL3115A2";
static std::string const MPL3115DriverLibName = "MPL3115";

MPL3115Lib MPL3115Lib::theOneAndOnly;


static GliderVarioDriverBase* getNewMPL3115Instance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (MPL3115A2DriverName.compare(driverName)) {
		return 0;
	}
	return new MPL3115Driver (driverName,description,instanceName);
}

MPL3115Lib::MPL3115Lib()
	: GliderVarioDriverLibBase{
		MPL3115DriverLibName.c_str(),
		"Absolute atmospheric pressure sensors MPL3115"}
{


}


MPL3115Lib::~MPL3115Lib() {

}

void MPL3115Lib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {
		MPL3115A2DriverName,
		"Absolute atmospheric pressure sensor MPL3115A2",
		getNewMPL3115Instance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
