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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <map>
#include <memory>
#include <mutex>

#include "Properties4CXX/Properties.h"

#include "OEVCommon.h"

#include "util/GliderVarioExceptionBase.h"

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

	/** \brief Function pointer type to a static constructor function for a specific subclass.
	 *
	 * This function pointer is stored together with the driver type name in the portTypeMap.
	 */
	typedef PortBase* (*PortConstructor)(char const* portName,Properties4CXX::Properties const &portProp);
	/// Container type for port types. Key is the port type name, value is a function pointer to a function which constructs an
	/// object of the respective port type and returns a pointer to it.
	typedef std::map<std::string,PortConstructor> PortTypeMap;
	/// Value type of the port type container. It is a std::pair of key and value.
	typedef PortTypeMap::value_type PortTypeMapValue;
	/// Constant iterator over the port type container
	typedef PortTypeMap::const_iterator PortTypeCIter;

	/// Intelligent pointer to a port object maintaining a reference counter.
	typedef std::shared_ptr<PortBase> PortBasePtr;
	/// Container storing all ports which were created based on the configuration. Key is the port name. Value is a PortBasePtr.
	typedef std::map<std::string,PortBasePtr> PortMap;
	/// Value type of the port container. It is a std::pais of key and value
	typedef PortMap::value_type PortMapValue;

	/// Under Linux all device nodes and files handles opened with open() and sockets created with socket() are uniformly int.
	typedef int DeviceHandleType;

	/** \brief Access to the device handle
	 *
	 * The device handle is a carefully protected resource of the PortBase class.
	 * Any access to the device handle is exclusively synchronized by a mutex.
	 * Even direct subclasses of PortBase must use this helper to gain access to the handle.
	 *
	 */
	class DeviceHandleAccess {
	public:
		/// Writable reference to the device handle in the PortBase object
		PortBase::DeviceHandleType &deviceHandle;
		/// Reference to the port for which this object was created.
		PortBase& port;
		/// When the constructor returns the PortBase::devHandleMutex is locked for the current thread thus synchronizing the access to the handle,
		/// and therefore all operations on the port itself.
		DeviceHandleAccess (PortBase& port) :
			port {port},
			deviceHandle {port.deviceHandle}
		{
			port.devHandleMutex.lock();
		}
		/// When the object is deleted exclusive access to the device handle is released. Other threads can gain access to the handle and
		/// perform I/O operations.
		~DeviceHandleAccess ()
		{
			port.devHandleMutex.unlock();
		}
	};


	/// \brief Possible statuses of the port
#if defined DOXYGEN
	enum StatusEnum {
#else
    OEV_ENUM(StatusEnum,
#endif
			CLOSED,			///< Device is in the initial or properly closed status
			OPEN,			///< Device is open and ready for communications
			ERR_IO_TEMP,	///< IO error occurred, but temporary and (hopefully) recoverable when re-trying read or write
			ERR_IO_PERM,	///< A non-recoverable IO error occurred
			ERR_NO_DEVICE,	///< The device does not exist. This is recoverable when the device is
							///< hot-pluggable, e.g. USB based serial ports or I2C devices.
							///< This status is also assumed when a network address cannot be reached.
			ERR_WRONG_TYPE	///< The device is not of the expected type, e.g. a file where a TTY device node is expected.
							///< This error is non-recoverable.
#if defined DOXYGEN
	};
#else
	);
#endif

protected:

    /** \brief Protected constructor, can only be invoked by sub-classes.
     *
     * @param portName Name of the port as being used in the configuration
     * @param portType Name of the port type.
     */
	PortBase(
			char const* portName,
			char const* portType
			);

public:


	virtual ~PortBase();

	/// Name of the port as defined in the configuration
	std::string const &getPortName() const {
		return portName;
	}

	/// The device name of the port.
	std::string const & getDeviceName () {
		return deviceName;
	}

	/// Name of the port type. The port type name is one registered in #typeMap.
	std::string const &getPortType() const {
		return portType;
	}

	/// Is the port blocking when no read data are available
	bool isBlocking() {
		return blocking;
	}

	/** \brief Is the port descendant of class \ref StreamPort?
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
	 * Try to create all ports listed in the section by calling loadSinglePort()
	 * If one port creation fails write error messages into the log file but continue scanning the section.
	 * An erroneous port will not be added to the list however.
	 * A device which uses the port may raise a fatal error when it cannot get the port it requires.
	 *
	 * @param properties Root node of the program properties
	 * @throws GliderVarioPortConfigException
	 */
	static void loadPorts(Properties4CXX::Properties const &properties);

	/** \brief Open the device port
	 *
	 * This is the public wrapper function.
	 * It handles the status handling, and locking the mutex.
	 * If needed it calls the device specific openInternal().
	 *
	 * @throws GliderVarioPortOpenException
	 * @throws GliderVarioPortDontExistException
	 */
	void open();

	/** \brief Close the port
	 *
	 * Tries to close the port if it was open before.
	 * When the port was not open before no action is taken.
	 * Regardless of the previous #status is reset in any case to CLOSED.
	 * No exception is thrown in any case. This function is also a cleanup.
	 *
	 * This is the public wrapper function.
	 * It handles the status handling, and locking the mutex.
	 * If needed it calls the device specific closeInternal().
	 */
	void close() noexcept;

	/** \brief Read a block of data from the device
	 *
	 * This is the common read function which handles mutex locking, and part of error handling
	 * The actual read function is implemented by the device dependent function readInternal()
	 *
	 * @param[out] buf Pointer to the buffer which receives the data
	 * @param bufLen Length of the buffer in bytes
	 * @param readFullBufSize When true the function will keep reading until \p buf is filled
	 * up to \p bufLen bytes unless an error occurs
	 * @return Number of bytes read. If non-blocking mode is assumed it returns 0 when no data is available.
	 * An end-of-file condition is here signaled by a GliderVarioPortReadEndOfFileException instead of returning 0.
	 * @throws GliderVarioPortNotOpenException, GliderVarioPortReadException, GliderVarioPortReadEndOfFileException
	 *
	 * \see readInternal()
	 */
	ssize_t read(void *buf,size_t bufLen,bool readFullBufSize = true);

	/** \brief Write a block of data to the device
	 *
	 * This is the common write function which handles mutex locking, and part of error handling
	 * The actual write function is implemented by the device dependent function writeInternal()
	 *
	 * @param[in] buf Pointer to the buffer which contains the data
	 * @param bufLen Length of the data in buf in bytes
	 * @param writeFullBufSize When true the function will keep writing until \p bufLen bytes are written.
	 * @return Number of bytes written. If non-blocking mode is assumed it returns 0 when no data could be written
	 * without blocking.
	 * An end-of-file condition is signaled by a GliderVarioPortReadEndOfFileException instead of returning 0.
	 * @throws GliderVarioPortNotOpenException, GliderVarioPortWriteException, GliderVarioPortWriteEndOfFileException
	 *
	 * \see writeInternal()
	 */
	ssize_t write(void const *buf,size_t bufLen,bool writeFullBufSize = true);

	/** \brief Tries to recover the status of the port in case of an error.
	 *
	 * The actions taken depend on the operational status of the port.
	 * Default action is to close(), and ignore any exceptions, and then open() again.
	 */
	virtual void recoverError();

    /** \brief Configure the port with the configuration settings
     *
     * Each driver implementation must override this method since it is completely implementation dependent.
     *
     * @param globalConfiguration The root configurations list. Used to retrieve global and other options
     * @param portConfiguration The configuration list of the current port structure
     * @throws GliderVarioPortConfigException
     */
	virtual void configurePort (Properties4CXX::Properties const& globalConfiguration, Properties4CXX::Properties const& portConfiguration) = 0;
private:
	/// Map of all port types. All port type classes register themselves here.
	static PortTypeMap typeMap;
	/// Mao of all ports. Ports are defined in the configuration.
	static PortMap portMap;

	/// Name of the port as defined in the configuration
	std::string portName;

	/// The device name of the port.
	std::string deviceName;

	/// Name of the port type. The port type name is one registered in #typeMap.
	std::string portType;

	/// Are read request blocking when no data is available?
	/// If the flag is observed depends on the driver type.
	bool blocking = false;

	/** \brief Handle to the device being used.
	 *
	 * Every relevant devices (files or device nodes as well as sockets are represented by a handle of type int.
	 * The handle is private. Access is only possible through the accessor class \ref DeviceHandleAccess.
	 */
	DeviceHandleType deviceHandle = 0;

	/** \brief Synchronization object to serialize access to #deviceHandle.
	 *
	 * This object is employed by class \ref DeviceHandleAccess to synchronize concurrent access to #deviceHandle.
	 */
	std::recursive_mutex devHandleMutex;

	/** \brief Read the configuration and create one port defined in a node under the structure "IOPorts"
	 *
	 * Read the property structure.
	 *
	 * Read the port port type and look it up in the port type list.
	 * If the port type does not exist write an error message into the log and return.
	 * Else call the port creation function and add the port to the port list.
	 *
	 * @param globalProperties List of global properties. Allows access to the complete configuration.
	 * Comes handy to retrieve global options
	 * @param portProperty Property node containing the port Property structure
	 * @throws GliderVarioPortConfigException
	 */
	static void loadSinglePort (
			Properties4CXX::Properties const &globalProperties,
			Properties4CXX::Property const &portProperty);


protected:

	/** \brief Registers a port type
	 *
	 * @param portType Name of the port type. This name has to be specified in the configuration when defining ports.
	 * @param portConstruct Function pointer to a function which constructs an object of the respective port type.
	 * @throws GliderVarioPortException
	 */
	static void addPortType (const char* portType, PortConstructor portConstruct);

	/** \brief Add a port to the portMap.
	 *
	 * Usually called by loadSinglePort(), but can be called from elsewhere in the program.
	 *
	 * @param port Shared pointer to the port. Management of the PortBase object is assumed by me here.
	 * Never delete the object yourself. Let the share pointer take care of it.
	 * @throws GliderVarioPortConfigException when the port name is not unique in the configuration.
	 */
	static void addPort (PortBasePtr port);

	/** \brief Device specific open function
	 *
	 * This function is the device specific open function. It is intended to be overridden.
	 * The status handling, and locking the mutex is handled by the wrapper function open()
	 *
	 * The default implementation opens the device with ::open().
	 *
	 * @throws GliderVarioPortOpenException
	 * @throws GliderVarioPortDontExistException
	 *
	 * \see Linux Programmer's Manual: [open(2)](http://man7.org/linux/man-pages/man2/open.2.html)
	 */
	virtual void openInternal();

	/** \brief Device specific close function
	 *
	 * This function is the device specific close function. It is intended to be overridden.
	 * The status handling, and locking the mutex is handled by the wrapper function close()
	 *
	 * The default implementation calls ::close()
	 *
	 * \see Linux Programmer's Manual: [close(2)](http://man7.org/linux/man-pages/man2/close.2.html)
	 */
	virtual void closeInternal() noexcept;


	/** \brief Device specific read function
	 *
	 * This is the device specific read function. It is intended to be overridden.
	 * The status handling, and locking the mutex is handled by the wrapper function read()
	 *
	 * The default implementation calls ::read().
	 *
	 * @param[out] buf Pointer to the buffer which receives the data
	 * @param bufLen Length of the buffer in bytes
	 * @param readFullBufSize When true the function will keep reading until \p buf is filled
	 * up to \p bufLen bytes unless an error occurs
	 * @return Number of bytes read. If non-blocking mode is assumed it returns 0 when no data is available.
	 * An end-of-file condition is here signaled by a GliderVarioPortReadEndOfFileException instead of returning 0.
	 * @throws GliderVarioPortReadException, GliderVarioPortReadEndOfFileException
	 *
	 * \see Linux Programmer's Manual: [read(2)](http://man7.org/linux/man-pages/man2/read.2.html)
	 * \see read()
	 */
	virtual ssize_t readInternal(void *buf,size_t bufLen,bool readFullBufSize);

	/** \brief Device specific write function
	 *
	 * This is the device specific write function. It is intended to be overridden.
	 * The status handling, and locking the mutex is handled by the wrapper function write()
	 *
	 * The default implementation calls ::write().
	 *
	 * @param[in] buf Pointer to the buffer which contains the data
	 * @param bufLen Length of the data in buf in bytes
	 * @param writeFullBufSize When true the function will keep writing until \p bufLen bytes are written.
	 * @return Number of bytes written. If non-blocking mode is assumed it returns 0 when no data could be written
	 * without blocking.
	 * An end-of-file condition is signaled by a GliderVarioPortReadEndOfFileException instead of returning 0.
	 * @throws GliderVarioPortWriteException, GliderVarioPortWriteEndOfFileException
	 *
	 * \see Linux Programmer's Manual: [write(2)](http://man7.org/linux/man-pages/man2/write.2.html)
	 * \see write()
	 */
	virtual ssize_t writeInternal(void const *buf,size_t bufLen,bool writeFullBufSize);

	/** \brief Flags for system call open(2)
	 *
	 * Default is O_RDWR
	 *
	 * \see Linux Programmer's Manual: [open(2)](http://man7.org/linux/man-pages/man2/open.2.html)
	 *
	 */
	int deviceOpenFlags = O_RDWR;

	/** \brief Current operational status of the port
	 *
	 * Possible values \see StatusEnum
	 */
	StatusEnum status = CLOSED;

	/** \brief Is this port a stream port?
	 *
	 * By default streamPort is false.
	 *
	 * This flag is being set only by the sub-class \ref StreamPort.
	 */
	bool streamPort = false;

};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_PORTBASE_H_ */
