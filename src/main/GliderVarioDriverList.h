/*
 * GliderVarioDriverList.h
 *
 *  Created on: 04.02.2018
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2018  Kai Horstmann
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

#ifndef MAIN_GLIDERVARIODRIVERLIST_H_
#define MAIN_GLIDERVARIODRIVERLIST_H_

#include <memory>
#include <map>
#include <string>

#include "drivers/GliderVarioDriverLibBase.h"
#include "drivers/GliderVarioDriverBase.h"


namespace openEV {

/** \brief Administration of driver libraries, and the driver list
 *
 * Administration, loading, storing, iteration through, and access to the list of sensor drivers
 *
 * This class maintains mainly three lists:
 * - List of driver libraries \ref driverLibList. These are listed in the configuration variable "driverSharedLibs"
 * - List of driver instances \ref driverInstanceList.
 *
 */
class GliderVarioDriverList {
public:

	typedef void (*DriverInitProc) ();
	typedef GliderVarioDriverLibBase* (*GetDriverLibProc)();

	typedef struct {
		GliderVarioDriverLibBase* libObj;
		std::string shLibName;
		void *shLibHandle;
		DriverInitProc driverInit;
		GetDriverLibProc getDriverLib;
	} DriverLibListItem;

	typedef std::map<std::string,DriverLibListItem> DriverLibList;
	typedef std::map<std::string,GliderVarioDriverBase *> DriverInstanceList;


	GliderVarioDriverList() {
		// TODO Auto-generated constructor stub

	}
	virtual ~GliderVarioDriverList();

	/** \brief Add a driver list item to the global list of avialable drivers
	 *
	 * @param driverListItem Driver list item to be added to \ref driverList.
	 */
	void addDriver (GliderVarioDriverLibBase::DriverListItem const& driverListItem);


protected:

	DriverLibList driverLibList;

	GliderVarioDriverLibBase::DriverList driverList;

	DriverInstanceList driverInstanceList;


};

} /* namespace openEV */

#endif /* MAIN_GLIDERVARIODRIVERLIST_H_ */
