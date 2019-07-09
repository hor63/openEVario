/*
 * PortBase.cpp
 *
 *  Created on: Jun 20, 2019
 *      Author: hor
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
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "PortBase.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.PortBase");
	}
}

#endif


namespace openEV {
namespace io {

PortBase::PortTypeMap PortBase::typeMap;
PortBase::PortMap PortBase::portMap;

PortBase::PortBase(
		char const* portName,
		char const* portType
		)
:portName{portName}
{
#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__
			<< "(portName=" << portName
			<< ", portType=" << portType << ')');

}

PortBase::~PortBase() {

}

void PortBase::addPortType (const char* portType, PortConstructor portConstruct) {

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_INFO(logger,__PRETTY_FUNCTION__
			<< "(portType=" << portType);

	auto rc = typeMap.emplace(portType,portConstruct);
	if (!rc.second) {
		std::ostringstream os;
		os << "Port type '" << portType << "' is defined more than once";
		LOG4CXX_ERROR(logger,os.str());
		throw GliderVarioPortException (__FILE__,__LINE__,os.str().c_str());
	}
}

void PortBase::addPort (PortBase& port) {

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_INFO(logger,__PRETTY_FUNCTION__
			<< "(portName=" << port.portName
			<< ", portType=" << port.portType << ')');

	auto rc = portMap.emplace(port.portName,&port);
	if (!rc.second) {
		std::ostringstream os;
		os << "Port '" << port.portName << "' is defined more than once";
		LOG4CXX_ERROR(logger,os.str());
		throw GliderVarioPortException (__FILE__,__LINE__,os.str().c_str());
	}
}

void PortBase::loadSinglePort (Properties4CXX::Property const &property) {

	LOG4CXX_INFO(logger,"Load port '" << property.getPropertyName() << '\'');

	if (property.isStruct()) {
		try {
			Properties4CXX::Properties const &portStruct = property.getPropertiesStructure();
			Properties4CXX::Property const *typeProp = portStruct.searchProperty("type");

			// look up the port type
			auto portTypeIter = typeMap.find(typeProp->getStringValue());
			if (portTypeIter == typeMap.end()) {
				LOG4CXX_ERROR(logger,"Type '" << typeProp->getStringValue() << "' of port '" << property.getPropertyName()
						<< "' does not exist.");
				LOG4CXX_ERROR(logger,"  Port '" << property.getPropertyName() << "' will not be created.");
			} else { // if (portTypeIter == typeMap.end())
				PortConstructor portConstructor = portTypeIter->second;
				portConstructor(property.getStrValue(),portStruct);
			} // if (portTypeIter == typeMap.end())

		} catch (Properties4CXX::ExceptionPropertyNotFound const &) {
			LOG4CXX_ERROR(logger,"Port '" << property.getPropertyName() << "' has no 'type' property.");
			LOG4CXX_ERROR(logger,"  Port '" << property.getPropertyName() << "' will not be created.");
		}
	} else { // if (property.isStruct())
		LOG4CXX_ERROR(logger,"Port configuration '" << property.getPropertyName() << "' is not a structure.");
		LOG4CXX_ERROR(logger,"  Port '" << property.getPropertyName() << "' will not be created.");
	} //if (property.isStruct())

}
void PortBase::loadPorts(Properties4CXX::Properties const &properties) {

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	try {
		auto ioPorts = properties.searchProperty("IOPorts");
		if (ioPorts->isStruct()) {
			Properties4CXX::Properties const &portList = ioPorts->getPropertiesStructure();
			auto portPropIter = portList.getFirstProperty();
			while (portPropIter != portList.getListEnd()) {
				Properties4CXX::Property const &portProp = portList.getPropertyFromIterator(portPropIter);
				loadSinglePort(portProp);

				portPropIter++;
			}
		} else {
			LOG4CXX_ERROR(logger,"Property 'IOPorts' is not a structure. No I/O ports will be created.");
		}
	} catch (Properties4CXX::ExceptionPropertyNotFound const& e) {
		LOG4CXX_WARN(logger,"Properties section 'IOPorts' does not exist. No I/O ports will be created.");
	}

}


} /* namespace io */
} /* namespace openEV */
