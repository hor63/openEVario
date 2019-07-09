/*
 * PortBase.h
 *
 *  Created on: Jun 20, 2019
 *      Author: hor
 *
 *  Definition of class PortBase, the base class of all IO ports
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2019  Kai Horstmann
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

#ifndef UTIL_PORTBASE_H_
#define UTIL_PORTBASE_H_

#include <string>
#include <map>
#include <memory>

#include "Properties4CXX/Properties.h"

#include "OEVCommon.h"

#include "GliderVarioExceptionBase.h"

namespace openEV {
namespace io {

/** \brief Base class for all I/O ports used throughout openEVario
 *
 * Purpose of this class is to manage port types (implementations of I/O ports like TCP, serial, I2C),
 * and manage the actual ports centrally. Provide a synchronization mechanism for concurrent access to ports.
 *
 * This class is **not** supposed to implement a comprehensive abstraction model for all I/O operations.
 * Some basic abstraction for streamed I/O is provided with the sub-class \ref StreamPort
 * For others, particularly I2C the users of the port have to do the heavy lifting themselves.
 *
 */
class OEV_UTILS_PUBLIC PortBase {
public:

	typedef PortBase* (*PortConstructor)(char const* portName,Properties4CXX::Properties const &portProp);
	typedef std::map<std::string,PortConstructor> PortTypeMap;
	typedef PortTypeMap::value_type PortTypeMapValue;
	typedef PortTypeMap::const_iterator PortTypeCIter;

	typedef std::shared_ptr<PortBase> PortBasePtr;
	typedef std::map<std::string,PortBasePtr> PortMap;
	typedef PortMap::value_type PortMapValue;

	virtual ~PortBase();

	std::string const &getPortName() const {
		return portName;
	}

	std::string const &getPortType() const {
		return portType;
	}

	/** \brief Is the port descendant of class \ref StreamPort
	 *
	 * @return true when the port is a streaming port.
	 */
	bool isStreamPort() const {
		return streamPort;
	}

	/** \brief Read the configuration and create all ports defined in the section "IOPorts"
	 *
	 * Read the properties. Scan the sub-node "IOPorts" when it exists. If not issue a warning.
	 *
	 * Try to create all ports listed in the section by calling \ref loadSinglePort()
	 * If one port creation fails write error messages into the log file but continue scanning the section.
	 * An erroneous port will not be added to the list however.
	 * A device which uses the port may raise a fatal error when it cannot get the port it requires.
	 *
	 * @param properties Root node of the program properties
	 */
	static void loadPorts(Properties4CXX::Properties const &properties);

	/** \brief Read the configuration and create one port defined in a node under the structure "IOPorts"
	 *
	 * Read the property structure.
	 *
	 * Read the port port type and look it up in the port type list.
	 * If the port type does not exist write an error message into the log and return.
	 * Else call the port creation function and add the port to the port list.
	 * 	 *
	 * @param property Property node of the port Property structure
	 */
	static void loadSinglePort (Properties4CXX::Property const &property);

private:
	static PortTypeMap typeMap;
	static PortMap portMap;

	std::string portName;
	std::string portType;

	bool streamPort = false;


protected:

	static void addPortType (const char* portType, PortConstructor portConstruct);
	static void addPort (PortBase& port);

	PortBase(
			char const* portName,
			char const* portType
			);

	void setStreamPort(bool streamPort) {
		this->streamPort = streamPort;
	}

};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_PORTBASE_H_ */
