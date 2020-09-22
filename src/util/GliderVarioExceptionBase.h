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

#include <cerrno>
#include <cstring>

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
	 * @param line where the exception is thrown
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
	/// Prepared string returned by what()
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

namespace io {

class OEV_UTILS_PUBLIC GliderVarioPortException :public GliderVarioExceptionBase {
public:
	GliderVarioPortException (
			char const *source,
			int line,
			char const *description)
		:GliderVarioExceptionBase {source,line,description}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortConfigException :public GliderVarioPortException {
public:
	GliderVarioPortConfigException (
			char const *source,
			int line,
			char const *description)
		:GliderVarioPortException {source,line,description}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortIOException :public GliderVarioPortException {
public:
	/**
	 * \brief Extended constructor storing the system error code of the failed IO operation
	 *
	 * @param source Source file; usually __FILE__
	 * @param line source line number; usually __LINE__
	 * @param description Descriptive text of the error causing the exception
	 * @param errNo System error number obtained by [errno](https://en.cppreference.com/w/cpp/error/errno)
	 * 		If the parameter is omitted or the default value is passed the constructor retrieves the latest error code from
	 * 		[errno](https://en.cppreference.com/w/cpp/error/errno) itself.
	 *
	 * \see [errno](https://en.cppreference.com/w/cpp/error/errno)
	 */
	GliderVarioPortIOException (
			char const *source,
			int line,
			char const *description,
			int errNo = -1)
		:GliderVarioPortException {source,line,description},
		 errNo {errNo}
	{
		if (this->errNo == -1) {
			this->errNo = errno;
		}
		errStr = std::strerror(errNo);
	}

	/** \brief Return the system error code causing this exception.
	 *
	 * @return System error code causing this exception
	 * \see [errno](https://en.cppreference.com/w/cpp/error/errno)
	 */
	int getErrno() const {
		return errNo;
	}

	/** \brief Obtain the string of the error code \a errno.
	 *
	 * @return System error string
	 *
	 * \see [strerror()](https://en.cppreference.com/w/cpp/string/byte/strerror)
	 */
	std::string const &getErrStr() const {
		return errStr;
	}
protected:
	/** \brief errno causing the I/O error
	 *
	 *  \see \ref getErrno()
	 * \see [errno](https://en.cppreference.com/w/cpp/error/errno)
	 */
	int errNo;

	/** \brief Systen error string of the I/O error
	 *
	 * \see \ref getErrStr()
	 * \see [strerror()](https://en.cppreference.com/w/cpp/string/byte/strerror)
	 */
	std::string errStr;

};

class OEV_UTILS_PUBLIC GliderVarioPortNotOpenException :public GliderVarioPortIOException {
public:
	GliderVarioPortNotOpenException (
			char const *source,
			int line)
		:GliderVarioPortIOException {source,line,"Port has not been opened.",EBADF}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortLocalPortUndefined :public GliderVarioPortIOException {
public:
	GliderVarioPortLocalPortUndefined (
			char const *source,
			int line)
		:GliderVarioPortIOException {source,line,"Local port number is undefined.",EBADF}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortPeerPortUndefined :public GliderVarioPortIOException {
public:
	GliderVarioPortPeerPortUndefined (
			char const *source,
			int line)
		:GliderVarioPortIOException {source,line,"Peer port number is undefined.",EBADF}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortPeerAddressUndefined :public GliderVarioPortIOException {
public:
	GliderVarioPortPeerAddressUndefined (
			char const *source,
			int line)
		:GliderVarioPortIOException {source,line,"Peer address is undefined.",EBADF}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortBufferTooSmallForDatagram :public GliderVarioPortIOException {
public:
	GliderVarioPortBufferTooSmallForDatagram (
			char const *source,
			int line)
		:GliderVarioPortIOException {source,line,"Buffer too small for datagram.",EBADF}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortNotConnected :public GliderVarioPortIOException {
public:
	GliderVarioPortNotConnected (
			char const *source,
			int line)
		:GliderVarioPortIOException {source,line,"The port is not connected or the destination address is undefined.",EBADF}
	{}

};


class OEV_UTILS_PUBLIC GliderVarioPortOpenException :public GliderVarioPortIOException {
public:
	/// \see GliderVarioPortIOException::GliderVarioPortIOException()
	GliderVarioPortOpenException (
			char const *source,
			int line,
			char const *description,
			int errNo = -1)
		:GliderVarioPortIOException {source,line,description,errNo}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortDontExistException :public GliderVarioPortIOException {
public:
	/// \see GliderVarioPortIOException::GliderVarioPortIOException()
	GliderVarioPortDontExistException (
			char const *source,
			int line,
			char const *description,
			int errNo = -1)
		:GliderVarioPortIOException {source,line,description,errNo}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortIsNoTTY :public GliderVarioPortIOException {
public:
	/// \see GliderVarioPortWriteException::GliderVarioPortWriteException()
	GliderVarioPortIsNoTTY (
			char const *source,
			int line,
			char const *description)
		:GliderVarioPortIOException {source,line,description,ENOTTY}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortReadException :public GliderVarioPortIOException {
public:
	/// \see GliderVarioPortIOException::GliderVarioPortIOException()
	GliderVarioPortReadException (
			char const *source,
			int line,
			char const *description,
			int errNo = -1)
		:GliderVarioPortIOException {source,line,description,errNo}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortReadEndOfFileException :public GliderVarioPortReadException {
public:
	/// \see GliderVarioPortReadException::GliderVarioPortReadException()
	GliderVarioPortReadEndOfFileException (
			char const *source,
			int line,
			char const *description)
		:GliderVarioPortReadException {source,line,description,0}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortWriteException :public GliderVarioPortIOException {
public:
	/// \see GliderVarioPortIOException::GliderVarioPortIOException()
	GliderVarioPortWriteException (
			char const *source,
			int line,
			char const *description,
			int errNo = -1)
		:GliderVarioPortIOException {source,line,description,errNo}
	{}

};

class OEV_UTILS_PUBLIC GliderVarioPortWriteEndOfFileException :public GliderVarioPortWriteException {
public:
	/// \see GliderVarioPortWriteException::GliderVarioPortWriteException()
	GliderVarioPortWriteEndOfFileException (
			char const *source,
			int line,
			char const *description)
		:GliderVarioPortWriteException {source,line,description,0}
	{}

};

} // namespace io



} /* namespace openEV */

#endif /* UTIL_GLIDERVARIOEXCEPTIONBASE_H_ */
