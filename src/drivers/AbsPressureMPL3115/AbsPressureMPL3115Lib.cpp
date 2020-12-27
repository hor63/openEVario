/*
 * AbsPressureMPL3115Lib.cpp
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


#include "AbsPressureMPL3115Lib.h"
#include "AbsPressureMPL3115Driver.h"

namespace openEV::drivers::AbsPressureMPL3115 {

static std::string const AbsPressureMPL3115DriverName = "MPL3115A2";
static std::string const AbsPressureMPL3115A2DriverLibName = "MPL3115";

AbsPressureMPL3115Lib AbsPressureMPL3115Lib::theOneAndOnly;


static GliderVarioDriverBase* getNewAbsPressureMPL3115Instance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (AbsPressureMPL3115DriverName.compare(driverName)) {
		return 0;
	}
	return new AbsPressureMPL3115Driver (driverName,description,instanceName);
}

AbsPressureMPL3115Lib::AbsPressureMPL3115Lib()
	: GliderVarioDriverLibBase{
		AbsPressureMPL3115A2DriverLibName.c_str(),
		"Absolute atmospheric pressure sensors MPL3115"}
{


}


AbsPressureMPL3115Lib::~AbsPressureMPL3115Lib() {

}

void AbsPressureMPL3115Lib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {
		AbsPressureMPL3115DriverName,
		"Absolute atmospheric pressure sensor MPL3115A2",
		getNewAbsPressureMPL3115Instance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
