/*
 * GliderVarioDriverBase.cpp
 *
 *  Created on: Oct 30, 2017
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


#define BUILDING_OEV_DRIVER 1

#include <system_error>

#include "OEVCommon.h"
#include "drivers/GliderVarioDriverBase.h"

namespace openEV {

void GliderVarioDriverBase::driverThreadEntry (GliderVarioDriverBase* tis) {
	tis->driverThreadFunction();
}


void GliderVarioDriverBase::start(GliderVarioMainPriv &varioMain) {

	if (isRunning) {
		stop();
	}

	isRunning = true;
	driverThread = std::thread(GliderVarioDriverBase::driverThreadEntry,this);
	this->varioMain = &varioMain;

}

void GliderVarioDriverBase::stop() {

	if (isRunning) {
		isRunning = false;
		if (driverThread.joinable()) {
			try {
				driverThread.join();

			} catch  (std::system_error& e) {
				;
			}
		}
	}

}


#if !defined DOXYGEN
GliderVarioDriverBase::SensorCapabilityHelperClass GliderVarioDriverBase::SensorCapabilityHelperObj;
#endif

}

std::ostream& operator << (std::ostream &o,openEV::GliderVarioDriverBase::SensorCapability ind) {
	o << openEV::GliderVarioDriverBase::SensorCapabilityHelperObj.getString (ind);
	return o;
}


