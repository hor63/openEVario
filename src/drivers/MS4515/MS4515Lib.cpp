/*
 * MS4515Lib.cpp
 *
 *  Created on: Jan 17 2021
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
#  include <config.h>
#endif

#include "OEVCommon.h"


#include "MS4515Lib.h"
#include "MS4515Driver.h"

namespace openEV::drivers::MS4515 {

static std::string const MS4515DODriverName = "MS4515DO";
static std::string const MS4515DriverLibName = "MS4515";

MS4515Lib MS4515Lib::theOneAndOnly;


static GliderVarioDriverBase* getNewMS4515Instance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (MS4515DODriverName.compare(driverName)) {
		return 0;
	}
	return new MS4515Driver (driverName,description,instanceName);
}

MS4515Lib::MS4515Lib()
	: GliderVarioDriverLibBase{
		MS4515DriverLibName.c_str(),
		"Absolute atmospheric pressure sensors MS4515"}
{


}


MS4515Lib::~MS4515Lib() {

}

void MS4515Lib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {
		MS4515DODriverName,
		"Differential and gage sensor MS4515DO",
		getNewMS4515Instance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
