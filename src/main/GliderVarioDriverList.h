/*
 * GliderVarioDriverList.h
 *
 *  Created on: 04.02.2018
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

#ifndef MAIN_GLIDERVARIODRIVERLIST_H_
#define MAIN_GLIDERVARIODRIVERLIST_H_

#include <memory>
#include <map>
#include <string>

#include "Properties4CXX/Properties.h"

#include "OEVCommon.h"
#include "util/GliderVarioExceptionBase.h"
#include "main/ProgramOptions.h"
#include "kalman/GliderVarioStatus.h"


namespace openEV {

class GliderVarioDriverLibBase;
class GliderVarioDriverBase;
class GliderVarioMainPriv;

/** \brief Administration of driver libraries, and the driver list
 *
 * Administration, loading, storing, iteration through, and access to the list of sensor drivers
 *
 * This class maintains mainly three lists:
 * - List of driver libraries \ref driverLibList. These are listed in the configuration variable "driverSharedLibs"
 * - List of driver instances \ref driverInstanceList.
 *
 */
class GliderVarioDriverList {
public:

	/// Function pointer type to create a new driver instance
	typedef GliderVarioDriverBase* (*GetNewDriverInstance) (
		    char const *driverName,
			char const *description,
			char const *instanceName);

	typedef struct {
		std::string const driverName;
		std::string const description;
		GetNewDriverInstance const getNewDriverInstance;
	} DriverListItem;

	typedef std::map<std::string,DriverListItem> DriverList;


	typedef void (*DriverInitProc) ();
	typedef GliderVarioDriverLibBase* (*GetDriverLibProc)();

	typedef struct {
		GliderVarioDriverLibBase* libObj;
		std::string shLibName;
		void *shLibHandle;
		DriverInitProc driverInit;
		GetDriverLibProc getDriverLib;
	} DriverLibListItem;

	typedef std::map<std::string,DriverLibListItem> DriverLibList;
	typedef std::shared_ptr<GliderVarioDriverBase> GliderVarioDriverBasePtr;
	typedef std::map<std::string,GliderVarioDriverBasePtr> DriverInstanceList;


	GliderVarioDriverList(ProgramOptions &programOptions);
	virtual ~GliderVarioDriverList();

	/** \brief Open the driver libraries and initialize them
	 *
	 * Tries to open the drivers as shared libraries.
	 *
	 * The list of driver libs is in the configuration variable "driverSharedLibs".
	 *
	 */
	void loadDriverLibs(Properties4CXX::Properties const &configuration);

	/** \brief Load the drivers defined in the configuration.
	 *
	 * Iterate over the list of driver instances "drivers" in the \ref configuration.
	 * Call \ref loadDriverInstance() for each instance name.
	 *
	 */
	void loadDriverInstances(Properties4CXX::Properties const &configuration);

	/** \brief Add a driver list item to the global list of available drivers
	 *
	 * @param driverListItem Driver list item to be added to \ref driverList.
	 */
	void OEV_MAIN_PUBLIC addDriver (DriverListItem const& driverListItem);

	/** \brief Each driver initializes a part of the Kalman filter with initial sensor readings
	 *
	 * @param currentStatus Kalman status to be initialized
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
	 */
	void initializeKalmanStatus(
			GliderVarioStatus &currentStatus,
			GliderVarioMainPriv &varioMain);

	/** \brief Start data capturing from all drivers
	 *
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
	 */
	void startDrivers (GliderVarioMainPriv &varioMain);

	/** \brief Stop data capturing from all drivers
	 *
	 * This call waits for every driver to stop. It may not return immediately.
	 *
	 */
	void stopDrivers ();

protected:

	DriverLibList driverLibList;

	DriverList driverList;

	DriverInstanceList driverInstanceList;

	ProgramOptions &programOptions;

	/** \brief Load a driver shared library.
	 *
	 * Load the driver libs and add them to \ref driverList.
	 * Let the libs add their driver implementations to \ref driverList.
	 *
	 */
	void loadDriverLib(char const *driverLibName);

	/** \brief Create a driver instance
	 *
	 * Read the configuration for the driver instance.
	 * Look up the driver from the configuration.
	 * Create a driver instance.
	 * Store the driver instance in the list of driver instances.
	 *
	 * @param driverInstanceName Name of the driver instance.
	 * @param configuration Configuration of the program. Must contain a structure named as the driverInstanceName.
	 */
	void loadDriverInstance(char const *driverInstanceName, Properties4CXX::Properties const &configuration);

};

} /* namespace openEV */

#include "drivers/GliderVarioDriverLibBase.h"
#include "drivers/GliderVarioDriverBase.h"


#endif /* MAIN_GLIDERVARIODRIVERLIST_H_ */
