/*
 * I2CPort.h
 *
 *  Created on: Dec 19, 2020
 *      Author: hor
 *
 *  Definition of class StreamPort, the base class of all stream based port
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

#ifndef UTIL_IO_I2CPORT_H_
#define UTIL_IO_I2CPORT_H_

#if HAVE_I2C_DEV_H

#define HAVE_I2C_PORT 1

#include <cstdint>

#include <mutex>

#include "util/GliderVarioExceptionBase.h"
#include "util/io/PortBase.h"

namespace openEV {
namespace io {

/** \brief Port driver for I2C and SMBUS devices.
 *
 * The class deals with varying capabilities of available headers, and libraries in the system
 * as well as the available functionality of the respective devices. \n
 * Capabilities of devices and device drivers vary therefore I need to check the
 * capabilities of each opened device at runtime.
 */
class OEV_UTILS_PUBLIC I2CPort: public PortBase {
public:

	/// The driver type name
	static constexpr char const* I2CPortType = "I2CMaster";

	I2CPort(char const* portName);
	virtual ~I2CPort();

	/// \see \ref PortBase::configurePort
	virtual void configurePort(
			const Properties4CXX::Properties &globalConfiguration,
			const Properties4CXX::Properties &portConfiguration) override;

	/** \brief Used to construct object of this class when ports are loaded from configuration
	 *
	 * @param portName Name of the port instance
	 * @param portProp Properties of the port structure. Not used here.
	 * @return New object of class I2CPort
	 */
	static PortBase* i2cPortConstructor(
			char const* portName,
			Properties4CXX::Properties const &portProp);

	/** \brief Register this driver with name \ref I2CPortType,
	 * and object construction function \ref i2cPortConstructor()
	 *
	 * Called by \ref GliderVarioMainPriv::registerPortDrivers()
	 */
	static void registerI2CPortType();

	/** \brief Write a single byte to the device
	 *
	 * Where the byte is written to and the effect is device and context specific
	 *
	 * @param devAddr 7 or 10 bit address (if supported) of the device on the I2C bus
	 * @param data One byte of data
	 * @throws GliderVarioPortException
	 * @throws GliderVarioPortIOException
	 * @throws GliderVarioPortI2C10BitAddr
	 */
	void writeByte (uint16_t devAddr,uint8_t data);

	/** \brief Write an array of bytes to the device
	 *
	 * Typically an I2C block write includes the address of the start register
	 * Please consult how to address the register bank of your device
	 *
	 * No written length is returned. The function always writes the complete block to the device.
	 * If not the entire length can be written an exception is thrown.
	 *
	 * To write a single byte or a block to a specific address use this function too.
	 * Simply increment the data buffer size, and write the register address into byte #0 of \p data.
	 *
	 * @param devAddr 7 or 10 bit address (if supported) of the device on the I2C bus
	 * @param[in] data Pointer to buffer containing data
	 * @param dataLen Length of \p data
	 * @throws GliderVarioPortException
	 * @throws GliderVarioPortIOException
	 * @throws GliderVarioPortI2C10BitAddr
	 */
	void writeBlock (uint16_t devAddr,uint8_t* data,uint16_t dataLen);

	/** \brief Read a single byte from the device
	 *
	 * Where the byte is read from and the content is device and context specific
	 *
	 * @param devAddr 7 or 10 bit address (if supported) of the device on the I2C bus
	 * @return The data byte
	 * @throws GliderVarioPortException
	 * @throws GliderVarioPortIOException
	 * @throws GliderVarioPortI2C10BitAddr
	 */
	uint8_t readByte (uint16_t devAddr);

	/** \brief Read a single byte from a specific register address from the device.
	 *
	 * Reading from a specific address consists of writing the register address as single byte to the device,
	 * send a re-start and then start reading a byte from the device in one transfer transaction.
	 * The byte sent to the device is interpreted as the register pointer
	 *
	 * @param devAddr 7 or 10 bit address (if supported) of the device on the I2C bus
	 * @param regAddr One-byte register address from which the block is read
	 * @return The data byte
	 * @throws GliderVarioPortException
	 * @throws GliderVarioPortIOException
	 * @throws GliderVarioPortI2C10BitAddr
	 */
	uint8_t readByteAtRegAddrByte (uint16_t devAddr,uint8_t regAddr);

	/** \brief Read a block of data from the device.
	 *
	 * @param devAddr 7 or 10 bit address (if supported) of the device on the I2C bus
	 * @param[out] data Pointer to a buffer to accept the data
	 * @param dataLen Length of bytes to be read. The buffer \p data must of course be large enough.
	 * @throws GliderVarioPortException
	 * @throws GliderVarioPortIOException
	 * @throws GliderVarioPortI2C10BitAddr
	 */
	void readBlock (uint16_t devAddr,uint8_t* data,uint16_t dataLen);

	/** \brief Read a block of data from a specific register address onward.
	 *
	 * Reading from a specific address consists of writing the register address as single byte to the device,
	 * send a re-start and then start reading from the device in one transfer transaction.
	 * The byte sent to the device is interpreted as the initial register pointer
	 *
	 * @param devAddr 7 or 10 bit address (if supported) of the device on the I2C bus
	 * @param regAddr One-byte register address from which the block is read
	 * @param[out] data Pointer to a buffer to accept the data
	 * @param dataLen Length of bytes to be read. The buffer \p data must of course be large enough.
	 * @throws GliderVarioPortException
	 * @throws GliderVarioPortIOException
	 * @throws GliderVarioPortI2C10BitAddr
	 */
	void readBlockAtRegAddrByte (uint16_t devAddr,uint8_t regAddr,uint8_t* data,uint16_t dataLen);

	/** \brief Combine sending a block of data and receiving a block of data within one transfer.
	 *
	 * This function can be useful when the device has register banks larger than 256 bytes, e.g. memory.
	 * In this case readBlockAtRegAddrByte() or readByteAtRegAddrByte() cannot be used because
	 * these functions support one-byte addresses only. However most sensors use one-byte register addresses.
	 *
	 * You are responsible to fill the write buffer with the register address in the correct endianess.
	 *
	 * @param devAddr 7 or 10 bit address (if supported) of the device on the I2C bus
	 * @param[in] writeData Pointer to a buffer holding the send data.
	 * @param writeDataLen Length of the send data in bytes
	 * @param[out] readData Pointer to a buffer to receive the read data
	 * @param readDataLen Number of bytes to the read. The \p readData buffer must be large enough to hold the data.
	 * @throws GliderVarioPortException
	 * @throws GliderVarioPortIOException
	 * @throws GliderVarioPortI2C10BitAddr
	 */
	void writeReadBlock (uint16_t devAddr,uint8_t* writeData,uint16_t writeDataLen,uint8_t* readData,uint16_t readDataLen);


private:

	bool useRawI2C = false;
	bool useSmBusI2CBlockFunctions = false;
	bool useSimpleByteSmBusFunctions = false;
	bool useSimpleByteAddressSmBusFunctions = false;

	bool allow10BitAddress = false;

protected:

	/** \brief Open the I2C device and obtain capabilities information
	 *
	 * Call PortBase::openInternal, and then obtain device capabilities next.
	 *
	 * This is also a test if the device understands I2C IOCTLs at all.
	 *
	 */
	virtual void openInternal () override;

	/** \brief Check if the device address is longer than 7 bit and if the device supports 10-bit addresses
	 *
	 * Check if \p devAddr is a 10-bit I2C address, and return \p true when true.
	 * When the address is a 10-bit address but the Linux device driver does not support 10-bit addresses
	 * throw \ref GliderVarioPortI2C10BitAddrException
	 *
	 * @param devAddr Address of the I2C slave
	 * @throws GliderVarioPortI2C10BitAddrException
	 */
	bool check10BitAddr(uint16_t devAddr);

};

} /* namespace io */
} /* namespace openEV */

#endif // #if HAVE_I2C_DEV_H


#endif /* UTIL_IO_I2CPORT_H_ */
