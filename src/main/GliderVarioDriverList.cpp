/*
 * GliderVarioDriverList.cpp
 *
 *  Created on: 04.02.2018
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016  Kai Horstmann
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

#if defined HAVE_CONFIG_H
#	include "config.h"
#endif

#include "GliderVarioDriverList.h"
#include "drivers/GliderVarioDriverLibBase.h"

namespace openEV {

GliderVarioDriverList::~GliderVarioDriverList() {

}

void GliderVarioDriverList::addDriver (GliderVarioDriverLibBase::DriverListItem const& driverListItem){
	GliderVarioDriverLibBase::DriverList::value_type newListItem {
		driverListItem.driverName,
		driverListItem
	};

	driverList.insert(newListItem);
}

} /* namespace openEV */
