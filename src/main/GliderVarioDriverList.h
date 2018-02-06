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

/** \brief Administration of the driver list
 *
 * Administration, loading, storing, iteration through, and access to the list of sensor drivers
 *
 */
class GliderVarioDriverList {
public:
	GliderVarioDriverList() {
		// TODO Auto-generated constructor stub

	}
	virtual ~GliderVarioDriverList();



protected:

	std::map<std::string,GliderVarioDriverLibBasePtr> driverLibList;
	std::map<std::string,GliderVarioDriverBasePtr> driverList;


};



} /* namespace openEV */

#endif /* MAIN_GLIDERVARIODRIVERLIST_H_ */
