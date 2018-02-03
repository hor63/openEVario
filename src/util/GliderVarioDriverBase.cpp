/*
 * GliderVarioDriverBase.cpp
 *
 *  Created on: Oct 30, 2017
 *      Author: kai_horstmann
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


#define BUILDING_OEV_DRIVER 1

#include "OEVCommon.h"
#include "drivers/GliderVarioDriverBase.h"

namespace openEV {

GliderVarioDriverBase::SensorCapabilityHelperClass GliderVarioDriverBase::SensorCapabilityHelperObj;

}

std::ostream& operator << (std::ostream &o,openEV::GliderVarioDriverBase::SensorCapability ind) {
	o << openEV::GliderVarioDriverBase::SensorCapabilityHelperObj.getString (ind);
	return o;
}


#if defined HAVE_LOG4CXX_H
std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioDriverBase::_SensorCapability e) {
	std::ostream &o = b;
	return operator << (o,e.e);
}
#endif /* #if defined HAVE_LOG4CXX_H */
