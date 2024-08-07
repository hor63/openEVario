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

#include <map>
#include <string>
#include <thread>

#include "CommonDefs.h"
#include "Properties4CXX/Properties.h"

#include "util/GliderVarioExceptionBase.h"
#include "main/ProgramOptions.h"
#include "kalman/GliderVarioStatus.h"
#include "kalman/GliderVarioMeasurementVector.h"


namespace openEV {

namespace drivers {
	class DriverLibBase;
	class DriverBase;
}
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
	typedef drivers::DriverBase* (*GetNewDriverInstance) (
		    char const *driverName,
			char const *description,
			char const *instanceName);

	typedef struct {
		std::string const driverName;
		std::string const description;
		GetNewDriverInstance const getNewDriverInstance;
	} DriverListItem;

	typedef std::map<std::string,DriverListItem> DriverList;


	typedef void (*DriverLibInitProc) ();
	typedef drivers::DriverLibBase* (*GetDriverLibProc)();

	typedef struct {
		drivers::DriverLibBase* libObj;
		std::string shLibName;
		void *shLibHandle;
		DriverLibInitProc driverLibInit;
		GetDriverLibProc getDriverLib;
	} DriverLibListItem;

	typedef std::map<std::string,DriverLibListItem> DriverLibList;
	typedef std::shared_ptr<drivers::DriverBase> GliderVarioDriverBasePtr;
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
	 * Iterate over the list of driver instances "drivers" in \p configuration.
	 * Call \ref loadDriverInstance() for each instance name.
	 *
	 */
	void loadDriverInstances(Properties4CXX::Properties const &configuration);

	/** \brief Add a driver list item to the global list of available drivers
	 *
	 * @param driverListItem Driver list item to be added to \ref driverList.
	 */
	void OEV_MAIN_PUBLIC addDriver (DriverListItem const& driverListItem);

	/** Initialize drivers
	 *
	 * Some drivers may need more initialization after loading the configuration.
	 * This call is called between \ref loadDriverInstances which also loads the configuration,
	 * and \ref startupDrivers.
	 * Thus this function is called after all driver instances have been loaded.
	 *
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
	 */
	void initDrivers (GliderVarioMainPriv &varioMain);

	/** \brief Startup the driver threads of all drivers
	 *
	 * Called between \ref initDrivers() and \ref initializeKalmanStatus().
	 *
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
	 */
	void startupDrivers (GliderVarioMainPriv &varioMain);

	/** \brief Each driver initializes a part of the Kalman filter with initial sensor readings
	 *
	 * Called after \ref startupDrivers()
	 *
	 * @param[in,out] currentStatus Kalman status to be initialized
	 * @param[in,out] measurements Current measurement vector.
	 * Used for cross-referencing measurements of other drivers during the initialization phase
	 * @param[in,out] varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
	 */
	void initializeKalmanStatus(
			GliderVarioStatus &currentStatus,
			GliderVarioMeasurementVector &measurements,
			GliderVarioMainPriv &varioMain);

	/** \brief Starts the from now on continuously running calibration data update thread
	 *
	 * The thread runs \ref calibrationDataUpdateThreadFunc()
	 */
	void startupCalibrationDataUpdateThread();

	/** \brief Start data capturing from all drivers
	 *
	 */
	void runDrivers ();

	/** \brief Stop data capturing from all drivers
	 *
	 * This call waits for every driver to stop. It may not return immediately.
	 *
	 */
	void stopDrivers ();

	/** \brief Is any driver implementing the idle loop internally for debugging purposes?
	 *
	 * Run through the list of drivers, and check if any implements the capability GliderVarioDriverBase::RUN_IDLE_LOOP
	 *
	 * @return
	 */
	bool isDriverRunningIdleLoop();

	DriverInstanceList const & getDriverInstanceList() const {
		return driverInstanceList;
	}

protected:

	DriverLibList driverLibList;

	DriverList driverList;

	DriverInstanceList driverInstanceList;

	ProgramOptions &programOptions;

	/** \brief Time when the next update cycle is planned.
	 *
	 * This timestamp is the basis for the calculation of the next update cycle
	 * of the driver instances.
	 *
	 * This one timestamp eliminates divergent update times of instances due to runtime of writing
	 * the calibration data sequentially, and minimizes the use of the expensive OEVClock::now() function
	 */
	OEVClock::time_point nextCalibrationDataUpdate;

	/** \brief The independent calibration data writer thread
	 *
	 * The calibration data write cycles are slow (>> 1sec) and not time critical or critical
	 * regarding accuracy of the actual writing interval.
	 * Therefore only one thread residing in this class is being implemented.
	 *
	 * The executing function is \ref calibrationDataUpdateThreadFunc().
	 */
	std::thread calibrationDataUpdateThread;

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


	/** \brief Calibration date update thread function.
	 *
	 * The thread which is running this function is owned by \ref calibrationDataUpdateThread.
	 *
	 * This function never returns.
	 * If it does not find any driver instance with a calibration update it will still run an update cycle of
	 * 1 min and tries again to find any driver instance with an update cycle.
	 *
	 * \see calibrationDataUpdateThread
	 */
	void calibrationDataUpdateThreadFunc();

	/** \brief Static entry function for the \ref calibrationDataUpdateThread
	 *
	 * std::thread::thread does not allow a non-static class method.
	 * This static method is a work-around this restriction.
	 * It just calls \ref calibrationDataUpdateThreadFunc().
	 *
	 * @param tis
	 */
	static void calibrationDataUpdateThreadEntry (GliderVarioDriverList* tis);

};

} /* namespace openEV */

#include "drivers/DriverLibBase.h"
#include "drivers/DriverBase.h"


#endif /* MAIN_GLIDERVARIODRIVERLIST_H_ */
