
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
#  include "config.h"
#endif

#include <sstream>
#include <cstring>

#include "fmt/format.h"

#include "CommonDefs.h"
#include "util/io/PortBase.h"


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

static std::string const DevicePropertyName = "device";
static std::string const BlockingPropertyName = "blocking";

#if !defined DOXYGEN
PortBase::StatusEnumHelperClass PortBase::StatusEnumHelperObj;
#endif
PortBase::PortTypeMap PortBase::typeMap;
PortBase::PortMap PortBase::portMap;

PortBase::PortBase(
		char const* portName,
		char const* portType
		) :
	portName{portName},
	deviceName{portName}, // set deviceName by default to portName.
	portType{portType}
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
			<< "(portType = " << portType);

	auto rc = typeMap.emplace(portType,portConstruct);
	if (!rc.second) {
		auto str = fmt::format(_("Port type \"{0}\" is defined more than once"),portType);
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioPortException (__FILE__,__LINE__,str.c_str());
	}
}

void PortBase::addPort (PortBasePtr port) {

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_INFO(logger, fmt::format(_("{0} with portName={1}, portType={2}"),
			__PRETTY_FUNCTION__,port->portName,port->portType));

	auto rc = portMap.emplace(port->portName,port);
	if (!rc.second) {
		auto str = fmt::format(_("Port \"{0}\" is defined more than once"),port->portName);
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioPortConfigException (__FILE__,__LINE__,str.c_str());
	}
}

void PortBase::loadSinglePort (
		Properties4CXX::Properties const &globalProperties,
		Properties4CXX::Property const &portProperty,
		std::string const &portName) {

	PortBase* newPort = nullptr;
	LOG4CXX_INFO(logger,fmt::format(_("Loading port \"{0}\""),portProperty.getPropertyName()));

	if (portProperty.isStruct()) {
		Properties4CXX::Properties const &portStruct = portProperty.getPropertiesStructure();
		try {
			Properties4CXX::Property const *typeProp = portStruct.searchProperty("type");

			LOG4CXX_DEBUG(logger, "port type = " << typeProp->getStringValue());

			// look up the port type
			auto portTypeIter = typeMap.find(typeProp->getStringValue());
			if (portTypeIter == typeMap.end()) {
				LOG4CXX_ERROR(logger,fmt::format(_("Type \"{0}\" of port \"{1}\" does not exist."),
						typeProp->getStringValue(),portProperty.getPropertyName()));
				LOG4CXX_ERROR(logger,fmt::format(_("  Port \"{0}\" will not be created."),portProperty.getPropertyName()));
			} else { // if (portTypeIter == typeMap.end())
				PortConstructor portConstructor = portTypeIter->second;
				newPort = portConstructor(portName.c_str(),portStruct);
			} // if (portTypeIter == typeMap.end())

		} catch (Properties4CXX::ExceptionPropertyNotFound const &) {
			LOG4CXX_ERROR(logger,fmt::format(_("Port \"{0}\" has no \"type\" property."),portProperty.getPropertyName()));
			LOG4CXX_ERROR(logger,fmt::format(_("  Port \"{0}\" will not be created."),portProperty.getPropertyName()));
		}
	} else { // if (property.isStruct())
		LOG4CXX_ERROR(logger,fmt::format(_("Port configuration property \"{0}\" is not a structure."),portProperty.getPropertyName()));
		LOG4CXX_ERROR(logger,fmt::format(_("  Port \"{0}\" will not be created."),portProperty.getPropertyName()));
	} //if (property.isStruct())

	if (newPort) {
		// The port was created.
		PortBasePtr portPtr (newPort);
		Properties4CXX::Properties const &portStruct = portProperty.getPropertiesStructure();

		// Configure the port
		// If an exception is thrown here, no worries.
		// The shared pointer object portPtr will take care of deleting the dangling port object.
		portPtr->configurePort(globalProperties,portStruct);

		// Add it to the list of ports.
		addPort (portPtr);

		// Load default properties
		newPort->deviceName = portStruct.getPropertyValue(DevicePropertyName,newPort->getPortName().c_str());
		newPort->blocking = portStruct.getPropertyValue(BlockingPropertyName,true);

		LOG4CXX_INFO(logger,fmt::format(_("Port \"{0}\" was successfully created and configured."),portProperty.getPropertyName()));
		LOG4CXX_DEBUG(logger, fmt::format(_("  deviceName = \"{0}\"."),newPort->deviceName));
		LOG4CXX_DEBUG(logger, fmt::format(_("  blocking = {0}"),newPort->blocking));

	}
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
				try {
					Properties4CXX::Property const &portProp = portList.getPropertyFromIterator(portPropIter);
					auto portName = portPropIter->first;
					loadSinglePort(properties,portProp,portName);
				} catch (GliderVarioPortConfigException const& e) {
					LOG4CXX_ERROR (logger, fmt::format(_(
							"Exception when configuring port \"{0}\" in source {1} at line {2}: {3}"),
							portPropIter->second->getPropertyName(),e.getSource(), e.getLine(), e.getDescription()));
				}
				catch (std::exception const& e) {
					LOG4CXX_ERROR (logger, fmt::format(_(
							"Exception when configuring port \"{0}\": {1}"),
							portPropIter->second->getPropertyName(),e.what()));
				}
				portPropIter++;
			}
		} else {
			LOG4CXX_ERROR(logger,_("Property \"IOPorts\" is not a structure. No I/O ports will be created."));
		}
	} catch (Properties4CXX::ExceptionPropertyNotFound const& e) {
		LOG4CXX_WARN(logger,_("Properties section \"IOPorts\" does not exist. No I/O ports will be created."));
	}

}

PortBase* PortBase::getPortByName(std::string const & portName) {

	auto rc = portMap.find(portName);

	if (rc == portMap.end()) {
		auto str = fmt::format(_("{0}: Cannot find IO port \"{1}\""),__PRETTY_FUNCTION__,portName);
		LOG4CXX_ERROR (logger,str);
		throw GliderVarioPortConfigException(__FILE__,__LINE__,str.c_str());
	}

	return rc->second.get();

}

void PortBase::open() {
	DeviceHandleAccess devLock(*this);

	if (status != CLOSED && status != OPEN) {
		this->close();
	}

	if (status == CLOSED) {

		openInternal();

		status = OPEN;
		numSameErrorOccurred = 0;
		lastErrno = 0;

	}

	LOG4CXX_INFO(logger,fmt::format(_("Port {0} of type {1} is open." ),portName, portType));

}

void PortBase::openInternal() {

	deviceHandle = ::open(getDeviceName().c_str(),deviceOpenFlags);

	if (deviceHandle == -1) {
		int err = errno;


		auto str = fmt::format(_("Port {0} of type {1}: Cannot open device \"{2}\""),portName, portType, deviceName);
		LOG4CXX_ERROR(logger,fmt::format(_("Port {0} of type {1}: Cannot open device \"{2}\". errno = {3}: {4}"),
				portName, portType, deviceName, err, strerror(err)));

		setErrno(err);

		if (err == ENOENT) {
			status = (status == ERR_IO_PERM) ? ERR_IO_PERM : ERR_NO_DEVICE;
			throw GliderVarioPortDeviceDontExistException (__FILE__,__LINE__,str.c_str(),err);
		} else {
			status = (status == ERR_IO_PERM) ? ERR_IO_PERM : ERR_IO_TEMP;
			throw GliderVarioPortOpenException (__FILE__,__LINE__,str.c_str(),err);
		}

	} else {
		LOG4CXX_INFO(logger,fmt::format(_("Port {0} of type {1}: Opened device \"{2}\""),portName, portType, deviceName));
	}

}

void PortBase::close() noexcept {
	DeviceHandleAccess devLock(*this);

	closeInternal();
	status = CLOSED;

	LOG4CXX_INFO(logger,fmt::format(_("Port {0} of type {1}: Closed device \"{2}\""),portName, portType, deviceName));

}

void PortBase::closeInternal() noexcept {
	if (deviceHandle > 0) {
		::close(deviceHandle);
	}
	deviceHandle = 0;
}

void PortBase::setErrno(int errn) {

	if (errn == lastErrno) {
		numSameErrorOccurred ++;
	} else {
		// Reset the counter.
		numSameErrorOccurred = 0;
	}
	lastErrno = errn;

	if (maxNumSameErrorOccurred != 0 && numSameErrorOccurred >= maxNumSameErrorOccurred && status != ERR_IO_PERM) {
		status = ERR_IO_PERM;
		LOG4CXX_ERROR(logger,fmt::format(_(
				"I/O Port {0}: Error code {1} has been set {2} times. The status is being set to permanent error."),
				portName,errn,numSameErrorOccurred));
	}
}


} /* namespace io */
} /* namespace openEV */
