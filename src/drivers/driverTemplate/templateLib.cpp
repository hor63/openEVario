/*
 * templateLib.cpp
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
#  include "config.h"
#endif

#include "OEVCommon.h"


#include "templateLib.h"
#include "templateDriver.h"

namespace openEV::drivers::templ {

static std::string const templateDriverName = "template";
static std::string const templateDriverLibName = "templateLib";

templateLib templateLib::theOneAndOnly;


static DriverBase* getNewtemplateInstance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (templateDriverName.compare(driverName)) {
		return 0;
	}
	return new templateDriver (driverName,description,instanceName);
}

templateLib::templateLib()
	: GliderVarioDriverLibBase{
		templateDriverLibName.c_str(),
		_("Template for a new driver library")}
{


}


templateLib::~templateLib() {

}

void templateLib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {
		templateDriverName,
		"template for new driver",
		getNewtemplateInstance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
