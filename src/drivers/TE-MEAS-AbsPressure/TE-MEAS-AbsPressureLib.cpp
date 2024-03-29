/*
 * TE_MEAS_AbsPressureLib.cpp
 *
 *  Created on: Apr 26, 2021
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
#  include "config.h"
#endif

#include "CommonDefs.h"

#include "TE-MEAS-AbsPressureLib.h"
#include "TE-MEAS-AbsPressureDriver.h"

namespace openEV::drivers::TE_MEAS_AbsPressure {

static std::string const TE_MEAS_AbsPressureDriverLibName = "TE_MEAS_AbsPressure";

TE_MEAS_AbsPressureLib TE_MEAS_AbsPressureLib::theOneAndOnly;

static DriverBase* getNewTE_MEAS_AbsPressureInstance (
	    char const *driverName,
		char const *description,
		char const *instanceName) {

	if (!strcmp(DriverNames[DRIVER_TYPE_MS5803],driverName)) {
		return new MS5803Driver (driverName,description,instanceName);
	}

	if (!strcmp(DriverNames[DRIVER_TYPE_MS5607],driverName)) {
		return new MS5607Driver (driverName,description,instanceName);
	}

	if (!strcmp(DriverNames[DRIVER_TYPE_MS5611],driverName)) {
		return new MS5611Driver (driverName,description,instanceName);
	}

	if (!strcmp(DriverNames[DRIVER_TYPE_MS5637],driverName)) {
		return new MS5637Driver (driverName,description,instanceName);
	}

	if (!strcmp(DriverNames[DRIVER_TYPE_MS5805],driverName)) {
		return new MS5805Driver (driverName,description,instanceName);
	}

	if (!strcmp(DriverNames[DRIVER_TYPE_MS5837],driverName)) {
		return new MS5837Driver (driverName,description,instanceName);
	}

	if (!strcmp(DriverNames[DRIVER_TYPE_MS5839],driverName)) {
		return new MS5839_MS5840Driver (driverName,description,instanceName);
	}

	if (!strcmp(DriverNames[DRIVER_TYPE_MS5840],driverName)) {
		return new MS5839_MS5840Driver (driverName,description,instanceName);
	}

	return 0;
}

TE_MEAS_AbsPressureLib::TE_MEAS_AbsPressureLib()
	: DriverLibBase{
		TE_MEAS_AbsPressureDriverLibName.c_str(),
		"Absolute atmospheric pressure sensors from TE Connectivity MEAS"}
{

}

TE_MEAS_AbsPressureLib::~TE_MEAS_AbsPressureLib() {

}

void TE_MEAS_AbsPressureLib::addDrivers(GliderVarioDriverList &gliderVarioDriverList) {

	for (int i = 0; i < DriverNamesNum; ++i) {
		std::string driverDescr = "Absolute atmospheric pressure sensor ";

		driverDescr += DriverNames[i];

		GliderVarioDriverList::DriverListItem listItem {
		DriverNames[i],
		driverDescr,
		getNewTE_MEAS_AbsPressureInstance};

		gliderVarioDriverList.addDriver(listItem);
	}
}

} /* namespace openEV */
