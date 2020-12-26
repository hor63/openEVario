/*
 * I2CPort.cpp
 *
 *  Created on: Dec 19, 2020
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

#include <errno.h>
#include <unistd.h>

#include <sys/ioctl.h>

#if HAVE_I2C_DEV_H
#	include <linux/i2c-dev.h>
#endif

#if HAVE_I2C_H
#	include <linux/i2c.h>
#endif

#if HAVE_SMBUS_H
// Treat the prototypes in i2c/smbus.h as C prototypes, not C++
extern "C" {
#	include <i2c/smbus.h>
}
#endif

#include <sstream>

#include "util/io/I2CPort.h"
#include <GliderVarioExceptionBase.h>

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.I2CPort");
	}
}

#endif

namespace openEV {
namespace io {

I2CPort::I2CPort(char const* portName)
		:PortBase {portName,I2CPortType}
{
	i2cPort = true;
	initLogger();
}

I2CPort::~I2CPort() {
}


void I2CPort::configurePort(
		const Properties4CXX::Properties &globalConfiguration,
		const Properties4CXX::Properties &portConfiguration) {
}

PortBase* I2CPort::i2cPortConstructor(const char *portName,
		const Properties4CXX::Properties &portProp) {

	return new I2CPort(portName);
}

void I2CPort::registerI2CPortType() {
	addPortType(I2CPortType,i2cPortConstructor);
}

void I2CPort::openInternal() {

	PortBase::openInternal();

	DeviceHandleAccess devLock(*this);

	unsigned long funcs = 0;
	auto rc = ioctl(devLock.deviceHandle, I2C_FUNCS, &funcs);
	if (rc != 0) {
		auto errN = errno;
		GliderVarioPortOpenException e (__FILE__, __LINE__, "I2CPort::openInternal(): Cannot retrieve I2C capabilities. Not an I2C device?",errN);
		LOG4CXX_ERROR(logger,"Error opening device " << getDeviceName()
				<< ". Cannot retrieve I2C capabilites. errno = " << errN << ": " << e.getErrStr());

		throw e;
	}

#if HAVE_I2C_H
	if (funcs & I2C_FUNC_I2C) {
		useRawI2C = true;
	}
#endif

#if HAVE_SMBUS_H
	if (funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK) {
		useSmBusI2CBlockFunctions = true;
	}

	if ((funcs & (I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE) == (I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE))) {
		useSimpleByteSmBusFunctions = true;
	}

	if ((funcs & (I2C_FUNC_SMBUS_READ_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_BYTE_DATA) == (I2C_FUNC_SMBUS_READ_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))) {
		useSimpleByteAddressSmBusFunctions = true;
	}

	if (funcs & I2C_FUNC_10BIT_ADDR) {
		allow10BitAddress = true;
	}
#endif // if HAVE_SMBUS_H

	LOG4CXX_DEBUG(logger,"Device " << getDeviceName() << " functions = " << std::hex << funcs << std::dec << ": "
			<< " useRawI2C = " << useRawI2C
			<< ", useSmBusI2CBlockFunctions = " << useSmBusI2CBlockFunctions
			<< ", useSimpleByteSmBusFunctions = " << useSimpleByteSmBusFunctions
			<< ", useSimpleByteAddressSmBusFunctions = " << useSimpleByteAddressSmBusFunctions
			<< ", allow10BitAddress = " << allow10BitAddress
			);

}

void I2CPort::writeByte(uint16_t devAddr, uint8_t data) {
	DeviceHandleAccess devLock(*this);
	int rc = 0;
	bool is10BitAddr (check10BitAddr(devAddr));

	// Actually prefer direct I2C ioctl: smbus calls are just a wrapper around SMBUS ioctls,
	// and in most real I2C drivers these are again wrappers around raw I2C calls.
	if (useRawI2C) {
			struct i2c_msg msg;
			struct i2c_rdwr_ioctl_data ioctlData = {
					&msg,1
			};

			msg.addr = devAddr;
			msg.buf = &data;
			msg.flags = 0;
			if (is10BitAddr) {
				msg.flags |= I2C_M_TEN;
			}
			msg.len = 1;

			rc = ioctl(devLock.deviceHandle,I2C_RDWR,&ioctlData);
			if (rc != 0) {
				auto errN = errno;
				GliderVarioPortIOException e (__FILE__, __LINE__, "I2CPort::writeByte ioctl-I2C_RDWR",errN);
				LOG4CXX_ERROR(logger,"I2CPort::writeByte error " << getDeviceName()
						<< ". errno = " << errN << ": " << e.getErrStr());
				throw e;
			}

		} else if (useSimpleByteSmBusFunctions && !is10BitAddr) {
		rc = i2c_smbus_write_byte(devLock.deviceHandle, data);
		if (rc != 0) {
			auto errN = errno;
			GliderVarioPortIOException e (__FILE__, __LINE__, "I2CPort::writeByte",errN);
			LOG4CXX_ERROR(logger,"I2CPort::writeByte error " << getDeviceName()
					<< ". errno = " << errN << ": " << e.getErrStr());
			throw e;
		}

	} else {
		GliderVarioPortException e (__FILE__, __LINE__, "I2CPort::writeByte: Neither SMBus nor direct I2C is supported.");
		LOG4CXX_ERROR(logger,"I2CPort::writeByte " << getDeviceName()
				<< ": Neither SMBus nor direct I2C is supported." << getDeviceName());
		throw e;
	}

}

void I2CPort::writeBlock(uint16_t devAddr, uint8_t *data, uint16_t dataLen) {
}

uint8_t I2CPort::readByte(uint16_t devAddr) {
	uint8_t rc = 0;
	DeviceHandleAccess devLock(*this);

	return rc;
}

uint8_t I2CPort::readByteAtRegAddrByte(uint16_t devAddr, uint8_t regAddr) {
	uint8_t rc = 0;
	DeviceHandleAccess devLock(*this);

	return rc;
}

void I2CPort::readBlock(uint16_t devAddr, uint8_t *data, uint16_t dataLen) {
	DeviceHandleAccess devLock(*this);
}

void I2CPort::readBlockAtRegAddrByte(uint16_t devAddr, uint8_t regAddr,
		uint8_t *data, uint16_t dataLen) {
	DeviceHandleAccess devLock(*this);
}

void I2CPort::writeReadBlock(uint16_t devAddr, uint8_t *writeData,
		uint16_t writeDataLen, uint8_t *readData, uint16_t readDataLen) {
}

bool I2CPort::check10BitAddr(uint16_t devAddr) {
	bool rc = false;

	if (devAddr > 0B1111111) {
		if (!allow10BitAddress) {
			GliderVarioPortI2C10BitAddrException e(__FILE__, __LINE__, getDeviceName().c_str(), devAddr);

			LOG4CXX_ERROR(logger,"I2CPort::check10BitAddr: Error using 10-bit I2C address " << std::hex << devAddr << std::dec
					<< " with device " << getDeviceName());
			throw e;
		}

		rc = true;
	}

	return rc;
}

} /* namespace io */
} /* namespace openEV */
