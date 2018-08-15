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

#include "OEVCommon.h"

#include "GliderVarioDriverBase.h"

namespace openEV {

/// Function pointer type to create a new driver instance
typedef GliderVarioDriverBase* (*GetNewDriverInstance) (
	    char const *driverName,
		char const *description,
		char const *instanceName);

typedef struct {
	std::string driverName;
	std::string description;
	GetNewDriverInstance getNewDriverInstance;
} DriverListItem;

typedef std::map<std::string,DriverListItem> TDriverList;

/** \brief Driver library base class.
 *
 * This is the base class for the driver library class. Each driver shared library returns an object of this class.
 * Primarily it enumerates the drivers of the library. One shared library may support several drivers.
 * Examples are the MPU-9x50 from TDK, formerly Invensense (MPU-9250, MPU-9150) which can are very similar.
 *
 */
class OEV_UTILS_PUBLIC GliderVarioDriverLibBase {
public:

	/** \brief Return the iterator through the list of available drivers from this library
	 *
	 * @return Iterator pointing to the begin of the list of available drivers
	 */
	TDriverList::const_iterator getCBegin() {
		return driverList.cbegin();
	}

	/** \brief Return a driver instance
	 *
	 * @param driverName
	 * @param instanceName
	 * @return
	 */
	GliderVarioDriverBase* getNewDriverInstance(std::string const &driverName,char const *instanceName) {
		auto it = driverList.find(driverName);

		if (it == driverList.end()) {
			return 0;
		}
		else {
			return it->second.getNewDriverInstance(
					it->second.driverName.c_str(),
					it->second.description.c_str(),
					instanceName);
		}
	}

	char const *getLibName () const {
		return libName.c_str();
	}

	char const *getDescription () const {
		return description.c_str();
	}


protected:
	GliderVarioDriverLibBase(
			char const * libName,
			char const * description)
	: libName {libName},
	  description {description}
	{

	}
	virtual ~GliderVarioDriverLibBase();

	/** \brief Driver list of this library
	 * The driver list consists of a pair with the driver name (key) and description (mapped value).
	 */
	TDriverList driverList;

	std::string libName;
	std::string description;


};

typedef GliderVarioDriverLibBase* GliderVarioDriverLibBasePtr;

} /* namespace openEV */

#endif /* DRIVERS_GLIDERVARIODriverLIBBASE_H_ */
