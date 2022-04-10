/*
 * GliderVarioDriverLibBase.h
 *
 *  Created on: Feb 6, 2018
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

#ifndef DRIVERS_GLIDERVARIODriverLIBBASE_H_
#define DRIVERS_GLIDERVARIODriverLIBBASE_H_

#include <map>
#include <string>

#include "CommonDefs.h"

namespace openEV {
// Forward declarations because the complex include hierarchy can have side effects on the order of includes
class GliderVarioDriverList;

namespace drivers {

// Forward declarations because the complex include hierarchy can have side effects on the order of includes
class DriverBase;

/** \brief Driver library base class.
 *
 * This is the base class for the driver library class. Each driver shared library returns an object of this class.
 * Primarily it enumerates the drivers of the library. One shared library may support several drivers.
 * Examples are the MPU-9x50 from TDK, formerly Invensense (MPU-9250, MPU-9150) which can are very similar.
 *
 */
class OEV_MAIN_PUBLIC DriverLibBase {
public:


	char const *getLibName () const {
		return libName.c_str();
	}

	char const *getDescription () const {
		return description.c_str();
	}

	/** \brief Add the drivers implemented by this library to the global driver list
	 *
	 * The method must be overridden by a library implementation. It will call \ref GliderVarioDriverList::addDriver() for each driver it implements.
	 *
	 * @param gliderVarioDriverList Global administration of sensor drivers.
	 */
	virtual void addDrivers(GliderVarioDriverList &gliderVarioDriverList) = 0;

protected:
	DriverLibBase(
			char const * libName,
			char const * description)
	: libName {libName},
	  description {description}
	{

	}
	virtual ~DriverLibBase();

	/** \brief Driver list of this library
	 * The driver list consists of a pair with the driver name (key) and description (mapped value).
	 */
	std::string libName;
	std::string description;


};

typedef DriverLibBase* GliderVarioDriverLibBasePtr;

} /* namespace drivers */
} /* namespace openEV */

#include "main/GliderVarioDriverList.h"
#include "DriverBase.h"


#endif /* DRIVERS_GLIDERVARIODriverLIBBASE_H_ */
