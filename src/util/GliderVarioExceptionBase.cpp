/*
 * GliderVarioExceptionBase.cpp
 *
 *  Created on: 03.02.2018
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "GliderVarioExceptionBase.h"
#include <sstream>

namespace openEV {

GliderVarioExceptionBase::~GliderVarioExceptionBase() {
	// Nothing explicit to do for me here.
}

GliderVarioExceptionBase::GliderVarioExceptionBase(
		char const *source,
		int line,
		char const *description)
:source{source},
 line{line},
 description{description}
{

	std::ostringstream ostr;
	ostr << "Exception at " << source << "[" << line << "] : " << description;
	whatString = ostr.str();

}

const char* GliderVarioExceptionBase::what() const noexcept{
	return whatString.c_str();
}


} /* namespace openEV */
