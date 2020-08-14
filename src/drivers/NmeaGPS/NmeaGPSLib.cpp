/*
 * NmeaGPSLib.cpp
 *
 *  Created on: Aug 13, 2020
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


#include "NmeaGPSLib.h"
#include "NmeaGPSDriver.h"

namespace openEV {

static std::string const NmeaGPSDriverName = "NmeaGPS";
static std::string const NmeaGPSDriverLibName = "NmeaGPSLib";

NmeaGPSLib NmeaGPSLib::theOneAndOnly;


static GliderVarioDriverBase* getNewNmeaGPSInstance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (NmeaGPSDriverName.compare(driverName)) {
		return 0;
	}
	return new NmeaGPSDriver (driverName,description,instanceName);
}

NmeaGPSLib::NmeaGPSLib()
	: GliderVarioDriverLibBase{
		NmeaGPSDriverLibName.c_str(),
		"NmeaGPS driver for satellite navigation devices emitting NMEA 0813 sentences on serial ports or Bluetooth"}
{


}


NmeaGPSLib::~NmeaGPSLib() {

}

void NmeaGPSLib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {
	GliderVarioDriverList::DriverListItem listItem {
		NmeaGPSDriverName,
		"NmeaGPS driver for satellite navigation devices emitting NMEA 0813 sentences on serial ports or Bluetooth",
		getNewNmeaGPSInstance};

	gliderVarioDriverList.addDriver(listItem);
}


} /* namespace openEV */
