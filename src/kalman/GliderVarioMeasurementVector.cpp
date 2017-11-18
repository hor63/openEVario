/*
 * GliderVarioMeasurementVector.cpp
 *
 *  Created on: Jan 31, 2016
 *      Author: hor
 *
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "GliderVarioMeasurementVector.h"

namespace openEV {

GliderVarioMeasurementVector::MeasureComponentIndexHelperClass GliderVarioMeasurementVector::MeasureComponentIndexHelperObj;

GliderVarioMeasurementVector::~GliderVarioMeasurementVector() {

}

} /* namespace openEV */

std::ostream& operator << (std::ostream &o, openEV::GliderVarioMeasurementVector::MeasureComponentIndex ind) {
    o << openEV::GliderVarioMeasurementVector::MeasureComponentIndexHelperObj.getString(ind);
    return o;
}

#if defined HAVE_LOG4CXX_H
std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::GliderVarioMeasurementVector::_MeasureComponentIndex e) {
	std::ostream &o = b;
	return operator << (o,e.e);
}
#endif /* #if defined HAVE_LOG4CXX_H */
