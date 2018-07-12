/*
 * GliderVarioExceptionBase.h
 *
 *  Created on: 03.02.2018
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

#ifndef UTIL_GLIDERVARIOEXCEPTIONBASE_H_
#define UTIL_GLIDERVARIOEXCEPTIONBASE_H_

#include <exception>
#include <string>

#include "OEVCommon.h"


namespace openEV {

/** \brief Base class of openEVario exceptions
 *
 * Base class of all openEVario exceptions
 *
 * This class is derived from the C++ standard library <a ref="http://en.cppreference.com/w/cpp/error/exception">std::exception</a>
 *
 */
class OEV_UTILS_PUBLIC GliderVarioExceptionBase: public std::exception {
public:

	/**
	 * Constructor
	 *
	 * @param source Source file where the exception is thrown
	 * @param Line where the exception is thrown
	 * @param description Description of the exception
	 */
	GliderVarioExceptionBase(
			char const *source,
			int line,
			char const *description);
	virtual ~GliderVarioExceptionBase();

	virtual const char* what() const noexcept override;

	char const * getSource() const {
		return source.c_str();
	}

	int getLine() const {
		return line;
	}

	char const * getDescription() const {
		return description.c_str();
	}

protected:

	/// Source file
	std::string source;
	/// Line where the exception is thrown
	int line;
	/// Description of the exception
	std::string description;
	/// Prepared string returned by \ref what()
	std::string whatString;
};

class OEV_UTILS_PUBLIC GliderVarioFatalConfigException :public GliderVarioExceptionBase {
public:
	GliderVarioFatalConfigException (
			char const *source,
			int line,
			char const *description)
		:GliderVarioExceptionBase {source,line,description}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioDriverLoadException :public GliderVarioExceptionBase {
public:
	GliderVarioDriverLoadException (
			char const *source,
			int line,
			char const *description)
		:GliderVarioExceptionBase {source,line,description}
	{}

};




} /* namespace openEV */

#endif /* UTIL_GLIDERVARIOEXCEPTIONBASE_H_ */
