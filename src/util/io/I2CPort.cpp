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
#  include "config.h"
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
		std::stringstream str;

		str << "Error opening device " << getDeviceName()
						<< ". Cannot retrieve I2C capabilites.";
		GliderVarioPortOpenException e (__FILE__, __LINE__,  str.str().c_str(),errN);

		str << " errno = " << errN << ": " << e.getErrStr();

		LOG4CXX_ERROR(logger,str.str());

		this->close();

		status = ERR_IO_PERM;

		throw e;
	}

#if HAVE_I2C_H
	if (funcs & I2C_FUNC_I2C) {
		useRawI2C = true;
	}
#endif

#if HAVE_SMBUS_H
	if ((funcs & I2C_FUNC_SMBUS_I2C_BLOCK) == I2C_FUNC_SMBUS_I2C_BLOCK) {
		useSmBusI2CBlockFunctions = true;
	}

	if ((funcs & I2C_FUNC_SMBUS_BYTE) == I2C_FUNC_SMBUS_BYTE) {
		useSimpleByteSmBusFunctions = true;
	}

	if ((funcs & I2C_FUNC_SMBUS_BYTE_DATA) == I2C_FUNC_SMBUS_BYTE_DATA) {
		useSimpleByteAddressSmBusFunctions = true;
	}

	if (funcs & I2C_FUNC_10BIT_ADDR) {
		allow10BitAddress = true;
	}
#endif // if HAVE_SMBUS_H

	LOG4CXX_DEBUG(logger,__FUNCTION__ << "Device " << getDeviceName() << " functions = 0x" << std::hex << funcs << std::dec << ": "
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
		if (rc < 0) {
			auto errN = errno;
			std::ostringstream str;

			str << __FUNCTION__ << " ioctl-I2C_RDWR error " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Wrote 0x" << std::hex << uint32_t(data) << std::dec
				<< " with ioctl to " << getDeviceName());

	} else if (useSimpleByteSmBusFunctions && !is10BitAddr) {
		rc = i2c_smbus_write_byte(devLock.deviceHandle, data);
		if (rc < 0) {
			auto errN = -rc;
			std::ostringstream str;

			str << __FUNCTION__ << " i2c_smbus_write_byte error " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Wrote 0x" << std::hex << uint32_t(data) << std::dec
				<< " with i2c_smbus_write_byte to " << getDeviceName());

	} else {
		std::ostringstream str;

		str << __FUNCTION__ << " " << getDeviceName()
						<< ": Neither SMBus nor direct I2C is supported.";
		GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
		LOG4CXX_ERROR(logger,str.str());
		throw e;
	}

}

void I2CPort::writeBlock(uint16_t devAddr, uint8_t *data, uint16_t dataLen) {
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
		msg.buf = data;
		msg.flags = 0;
		if (is10BitAddr) {
			msg.flags |= I2C_M_TEN;
		}
		msg.len = dataLen;

		rc = ioctl(devLock.deviceHandle,I2C_RDWR,&ioctlData);
		if (rc < 0) {
			auto errN = errno;
			std::ostringstream str;

			str << __FUNCTION__ << " error ioctl-I2C_RDWR " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Wrote " <<  dataLen
				<< " bytes with ioctl to " << getDeviceName());

	} else {
		// Depending on the data length employ different SMBUS calls to mimic a plain block write
		switch (dataLen) {
		case 0:
			LOG4CXX_WARN(logger,__FUNCTION__ << ": Why the hell do you want to send 0 (ZERO!) bytes to device 0x"
					<< std::hex << uint32_t(devAddr) << std::dec << '?');
			break;
		case 1:

			if (useSimpleByteSmBusFunctions) {

				rc = i2c_smbus_write_byte(devLock.deviceHandle, *data);
				if (rc < 0) {
					auto errN = -rc;
					std::ostringstream str;

					str << __FUNCTION__ << " error i2c_smbus_write_byte " << getDeviceName()
									<< ".";
					GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

					str << " errno = " << errN << ": " << e.getErrStr();
					LOG4CXX_ERROR(logger,str.str());
					throw e;
				}

				LOG4CXX_TRACE(logger, __FUNCTION__ << ": Wrote 0x" << std::hex << uint32_t(*data) << std::dec
						<< " with i2c_smbus_write_byte to " << getDeviceName());

			} else {
				std::ostringstream str;

				str << __FUNCTION__ << " " << getDeviceName()
								<< ": Neither SMBus nor direct I2C is supported.";
				GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
				LOG4CXX_ERROR(logger,str.str());
				throw e;
			}

			break;
		case 2:

			if (useSimpleByteAddressSmBusFunctions) {

				rc = i2c_smbus_write_byte_data(devLock.deviceHandle,data[0],data[1]);
				if (rc < 0) {
					auto errN = -rc;
					std::ostringstream str;

					str << __FUNCTION__ << " error i2c_smbus_write_byte_data " << getDeviceName()
									<< ".";
					GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

					str << " errno = " << errN << ": " << e.getErrStr();
					LOG4CXX_ERROR(logger,str.str());
					throw e;
				}

				LOG4CXX_TRACE(logger, __FUNCTION__ << ": Wrote 0x" << std::hex << uint32_t(data[1])
						<< " to register 0x" << uint32_t(data[0]) << std::dec
						<< " with i2c_smbus_write_byte_data to " << getDeviceName());

			} else {
				std::ostringstream str;

				str << __FUNCTION__ << " " << getDeviceName()
								<< ": Neither SMBus nor direct I2C is supported.";
				GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
				LOG4CXX_ERROR(logger,str.str());
				throw e;
			}

			break;
		default:

			if (useSmBusI2CBlockFunctions) {

				rc = i2c_smbus_write_i2c_block_data(devLock.deviceHandle,data[0],dataLen-1,data+1);
				if (rc < 0) {
					auto errN = -rc;
					std::ostringstream str;

					str << __FUNCTION__ << " error i2c_smbus_write_i2c_block_data " << getDeviceName()
									<< ".";
					GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

					str << " errno = " << errN << ": " << e.getErrStr();
					LOG4CXX_ERROR(logger,str.str());
					throw e;
				}
				LOG4CXX_TRACE(logger, __FUNCTION__ << ": Wrote " << dataLen
						<< " bytes with i2c_smbus_write_i2c_block_data to " << getDeviceName());

			} else {
				std::ostringstream str;

				str << __FUNCTION__ << " " << getDeviceName()
								<< ": Neither SMBus nor direct I2C is supported.";
				GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
				LOG4CXX_ERROR(logger,str.str());
				throw e;
			}


			break;
		}
	}
}

uint8_t I2CPort::readByte(uint16_t devAddr) {
	uint8_t data= 0;
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
		msg.flags = I2C_M_RD;
		if (is10BitAddr) {
			msg.flags |= I2C_M_TEN;
		}
		msg.len = 1;

		rc = ioctl(devLock.deviceHandle,I2C_RDWR,&ioctlData);
		if (rc < 0) {
			auto errN = errno;
			std::ostringstream str;

			str << __FUNCTION__ << " error ioctl-I2C_RDWR " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Read 0x" << std::hex << uint32_t(data) << std::dec
				<< " with ioctl from " << getDeviceName());

	} else if (useSimpleByteSmBusFunctions && !is10BitAddr) {

		rc = i2c_smbus_read_byte(devLock.deviceHandle);
		if (rc < 0) {
			auto errN = -rc;
			std::ostringstream str;

			str << __FUNCTION__ << " error i2c_smbus_read_byte " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		} else {
			data = uint8_t(rc);
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Read 0x" << std::hex << uint32_t(data) << std::dec
				<< " with i2c_smbus_read_byte from " << getDeviceName());

	} else {
		std::ostringstream str;

		str << __FUNCTION__ << " " << getDeviceName()
						<< ": Neither SMBus nor direct I2C is supported.";
		GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
		LOG4CXX_ERROR(logger,str.str());
		throw e;
	}

	return data;
}

uint8_t I2CPort::readByteAtRegAddrByte(uint16_t devAddr, uint8_t regAddr) {
	uint8_t data = 0;
	DeviceHandleAccess devLock(*this);
	int rc = 0;
	bool is10BitAddr (check10BitAddr(devAddr));

	// Actually prefer direct I2C ioctl: smbus calls are just a wrapper around SMBUS ioctls,
	// and in most real I2C drivers these are again wrappers around raw I2C calls.
	if (useRawI2C) {
		struct i2c_msg msg[2];
		struct i2c_rdwr_ioctl_data ioctlData = {
				msg,2
		};

		msg[0].addr = devAddr;
		msg[0].buf = &regAddr;
		msg[0].flags = 0;
		if (is10BitAddr) {
			msg[0].flags |= I2C_M_TEN;
		}
		msg[0].len = 1;

		msg[1] = msg[0];
		msg[1].flags |= I2C_M_RD;
		msg[1].buf = &data;

		rc = ioctl(devLock.deviceHandle,I2C_RDWR,&ioctlData);
		if (rc < 0) {
			auto errN = errno;
			std::ostringstream str;

			str << __FUNCTION__ << " error ioctl-I2C_RDWR " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Read 0x" << std::hex << uint32_t(data)
				<< " with ioctl from register 0x" << uint32_t(regAddr) << std::dec << " from " << getDeviceName());

	} else if (useSimpleByteAddressSmBusFunctions && !is10BitAddr) {

		rc = i2c_smbus_read_byte_data(devLock.deviceHandle,regAddr);
		if (rc < 0) {
			auto errN = -rc;
			std::ostringstream str;

			str << __FUNCTION__ << " error i2c_smbus_read_byte_data " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		} else {
			data = uint8_t(rc);
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Read 0x" << std::hex << uint32_t(data)
				<< " with i2c_smbus_read_byte_data from register 0x" << uint32_t(regAddr) << std::dec << " from " << getDeviceName());

	} else {
		std::ostringstream str;

		str << __FUNCTION__ << " " << getDeviceName()
						<< ": Neither SMBus nor direct I2C is supported.";
		GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
		LOG4CXX_ERROR(logger,str.str());
		throw e;
	}

	return data;
}

void I2CPort::readBlock(uint16_t devAddr, uint8_t *data, uint16_t dataLen) {
	DeviceHandleAccess devLock(*this);
	int rc = 0;
	bool is10BitAddr (check10BitAddr(devAddr));

	// Only the direct I2C ioctl supports reading a plain block at once without addressing a register.
	if (useRawI2C) {
		struct i2c_msg msg;
		struct i2c_rdwr_ioctl_data ioctlData = {
				&msg,1
		};

		msg.addr = devAddr;
		msg.buf = data;
		msg.flags = I2C_M_RD;
		if (is10BitAddr) {
			msg.flags |= I2C_M_TEN;
		}
		msg.len = dataLen;

		rc = ioctl(devLock.deviceHandle,I2C_RDWR,&ioctlData);
		if (rc < 0) {
			auto errN = errno;
			std::ostringstream str;

			str << __FUNCTION__ << " error ioctl-I2C_RDWR " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Read " << dataLen
				<< " bytes with ioctl from " << getDeviceName());

	} else {
		std::ostringstream str;

		str << __FUNCTION__ << " " << getDeviceName()
						<< ": Direct I2C is not supported.";
		GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
		LOG4CXX_ERROR(logger,str.str());
		throw e;
	}
}

void I2CPort::readBlockAtRegAddrByte(
		uint16_t devAddr, uint8_t regAddr,
		uint8_t *data, uint16_t dataLen) {
	DeviceHandleAccess devLock(*this);
	int rc = 0;
	bool is10BitAddr (check10BitAddr(devAddr));

	// Actually prefer direct I2C ioctl: smbus calls are just a wrapper around SMBUS ioctls,
	// and in most real I2C drivers these are again wrappers around raw I2C calls.
	if (useRawI2C) {
		struct i2c_msg msg[2];
		struct i2c_rdwr_ioctl_data ioctlData = {
				msg,2
		};

		msg[0].addr = devAddr;
		msg[0].buf = &regAddr;
		msg[0].flags = 0;
		if (is10BitAddr) {
			msg[0].flags |= I2C_M_TEN;
		}
		msg[0].len = 1;

		msg[1] = msg[0];
		msg[1].flags |= I2C_M_RD;
		msg[1].buf = data;
		msg[1].len = dataLen;

		rc = ioctl(devLock.deviceHandle,I2C_RDWR,&ioctlData);
		if (rc < 0) {
			auto errN = errno;
			std::ostringstream str;

			str << __FUNCTION__ << " error ioctl-I2C_RDWR " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Read " << dataLen
				<< " bytes with ioctl from register 0x"
				<< std::hex << uint32_t(regAddr) << std::dec << " from " << getDeviceName());

	} else if (useSmBusI2CBlockFunctions && !is10BitAddr) {

		rc = i2c_smbus_read_i2c_block_data(devLock.deviceHandle,regAddr,dataLen,data);
		if (rc < 0) {
			auto errN = -rc;
			std::ostringstream str;

			str << __FUNCTION__ << " error i2c_smbus_read_i2c_block_data " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Read " << dataLen
				<< " bytes with i2c_smbus_read_i2c_block_data from register 0x"
				<< std::hex << uint32_t(regAddr) << std::dec << " from " << getDeviceName());

	} else {
		std::ostringstream str;

		str << __FUNCTION__ << " " << getDeviceName()
						<< ": Neither SMBus nor direct I2C is supported.";
		GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
		LOG4CXX_ERROR(logger,str.str());
		throw e;
	}

}

void I2CPort::writeReadBlock(uint16_t devAddr, uint8_t *writeData,
		uint16_t writeDataLen, uint8_t *readData, uint16_t readDataLen) {
	DeviceHandleAccess devLock(*this);
	int rc = 0;
	bool is10BitAddr (check10BitAddr(devAddr));

	// Writing and reading blocks of arbitrary length in one transfer only works for raw I2C.
	// SMBus does not support that.
	if (useRawI2C) {
		struct i2c_msg msg[2];
		struct i2c_rdwr_ioctl_data ioctlData = {
				msg,2
		};

		msg[0].addr = devAddr;
		msg[0].buf = writeData;
		msg[0].flags = 0;
		if (is10BitAddr) {
			msg[0].flags |= I2C_M_TEN;
		}
		msg[0].len = writeDataLen;

		msg[1] = msg[0];
		msg[1].flags |= I2C_M_RD;
		msg[1].buf = readData;
		msg[1].len = readDataLen;

		rc = ioctl(devLock.deviceHandle,I2C_RDWR,&ioctlData);
		if (rc < 0) {
			auto errN = errno;
			std::ostringstream str;

			str << __FUNCTION__ << " error ioctl-I2C_RDWR " << getDeviceName()
							<< ".";
			GliderVarioPortIOException e (__FILE__, __LINE__, str.str().c_str(),errN);

			str << " errno = " << errN << ": " << e.getErrStr();
			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		LOG4CXX_TRACE(logger, __FUNCTION__ << ": Wrote "
				<< writeDataLen << " bytes, read " << readDataLen
				<< " bytes with ioctl to and from " << getDeviceName());

	} else {
		std::ostringstream str;

		str << __FUNCTION__ << " " << getDeviceName()
						<< ": Direct I2C is not supported.";
		GliderVarioPortException e (__FILE__, __LINE__, str.str().c_str());
		LOG4CXX_ERROR(logger,str.str());
		throw e;
	}

}

bool I2CPort::check10BitAddr(uint16_t devAddr) {
	bool rc = false;

	if (devAddr > 0B1111111) {
		if (!allow10BitAddress) {
			std::ostringstream str;

			str << "I2CPort::check10BitAddr: Error using 10-bit I2C address " << std::hex << devAddr << std::dec
					<< " is not supported for device " << getDeviceName();

			GliderVarioPortI2C10BitAddrException e(__FILE__, __LINE__, str.str().c_str(), devAddr);

			LOG4CXX_ERROR(logger,str.str());
			throw e;
		}

		rc = true;
	}

	return rc;
}

} /* namespace io */
} /* namespace openEV */
