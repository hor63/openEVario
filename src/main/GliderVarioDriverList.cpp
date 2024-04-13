/*
 * GliderVarioDriverList.cpp
 *
 *  Created on: 04.02.2018
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016  Kai Horstmann
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

#if defined HAVE_CONFIG_H
#	include "config.h"
#endif

// Used for opening shared libraries
#include <dlfcn.h>

#include "fmt/format.h"

#include "GliderVarioDriverList.h"
#include "drivers/DriverLibBase.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;
#endif

namespace openEV {

GliderVarioDriverList::GliderVarioDriverList(ProgramOptions &programOptions)
	:programOptions {programOptions}
{
#if defined HAVE_LOG4CXX_H
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Main.GliderVarioDriverList");
	}
#endif /* HAVE_LOG4CXX_H */

}

GliderVarioDriverList::~GliderVarioDriverList() {

}

void GliderVarioDriverList::addDriver (DriverListItem const& driverListItem){
	DriverList::value_type newListItem {
		driverListItem.driverName,
		driverListItem
	};

	LOG4CXX_INFO(logger,fmt::format(_("Adding new driver \"{0}\": {1}"),driverListItem.driverName, driverListItem.description));

	driverList.insert(newListItem);
}


void GliderVarioDriverList::loadDriverLibs(Properties4CXX::Properties const &configuration) {

	// get the driver lib list from the configuration
	Properties4CXX::Property const * driverLibNames;

	try {
	driverLibNames = configuration.searchProperty ("driverSharedLibs");
	} catch (Properties4CXX::ExceptionBase const& e) {
		auto str = fmt::format(_("Error searching mandatory configuration parameter \"{0}\": {1}"),"driverSharedLibs",e.what());
		LOG4CXX_FATAL(logger,str);
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str() );
	}

	if (!driverLibNames) {
		auto str = fmt::format(_("Configuration does not contain mandatory parameter \"{0}\""),"driverSharedLibs");
		LOG4CXX_FATAL(logger,str);

		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
	}

	LOG4CXX_INFO(logger,fmt::format(_("List of dynamically loaded driver libraries : {0}"), driverLibNames->getStrValue()));

	if (driverLibNames->isString()) {
		// If this single driver load fails I cannot ignore a load failure.
		// Therefore an exception will fall through.

		loadDriverLib(driverLibNames->getStrValue());
	} else {
		if (driverLibNames->isList()) {
			Properties4CXX::PropertyValueList const & nameList = driverLibNames->getPropertyValueList();
			auto nameIter = nameList.cbegin();

			while (nameIter != nameList.cend()) {

				// If necessary swallow an exception here and carry on loading drivers.
				try {
					loadDriverLib(nameIter->c_str());
				} catch (GliderVarioDriverLoadException& e) {
					if (programOptions.terminateOnDriverLoadError) {
						throw;
					}
				}

				nameIter++;
			}
		} else {
			auto str = fmt::format(_("Configuration variable \"{0}\" is neither a string nor a string list"),"driverSharedLibs");
			LOG4CXX_FATAL(logger,str);
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
		}
	}


}

void GliderVarioDriverList::loadDriverInstances(Properties4CXX::Properties const &configuration) {

	// get the driver lib list from the configuration
	Properties4CXX::Property const * driverNames;

	try {
	driverNames = configuration.searchProperty ("drivers");
	} catch (Properties4CXX::ExceptionBase const& e) {
		auto str = fmt::format(_("Error searching mandatory configuration parameter \"{0}\": {1}"),"drivers",e.what());
		LOG4CXX_FATAL(logger,str);
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str() );
	}

	if (!driverNames) {
		auto str = fmt::format(_("Configuration does not contain mandatory parameter \"{0}\""),"drivers");
		LOG4CXX_FATAL(logger,str);

		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
	}

	if (driverNames->isString()) {
		// If this single driver load fails I cannot ignore a load failure.
		// Therefore an exception will fall through.
		loadDriverInstance(driverNames->getStrValue(),configuration);
	} else {
		if (driverNames->isList()) {
			Properties4CXX::PropertyValueList const & nameList = driverNames->getPropertyValueList();
			auto nameIter = nameList.cbegin();

			while (nameIter != nameList.cend()) {

				// If necessary swallow an exception here and carry on loading drivers.
				try {
					loadDriverInstance(nameIter->c_str(),configuration);
				} catch (GliderVarioDriverLoadException& e) {
					if (programOptions.terminateOnDriverLoadError) {
						LOG4CXX_FATAL(logger,fmt::format(_("Failed to create device \"{0}\" because: {1}"),nameIter->c_str(),e.what()));
						LOG4CXX_FATAL(logger,_("Program will be terminated."));
						LOG4CXX_FATAL(logger,_("Bye bye!"));
						throw;
					} else {
						LOG4CXX_ERROR(logger,fmt::format(_("Failed to create device \"{0}\" because: {1}"),nameIter->c_str(),e.what()));
						LOG4CXX_ERROR(logger,fmt::format(_("Deleting device \"{0}\" from the device list"),nameIter->c_str()));
					}
				}

				nameIter++;
			}
		} else {
			auto str = fmt::format(_("Configuration variable \"{0}\" is neither a string nor a string list"),"drivers");
			LOG4CXX_FATAL(logger,str);
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
		}
	}


}

void GliderVarioDriverList::loadDriverLib(const char* driverLibName) {


	char const* errStr = NULL;
	DriverLibListItem listItem;

	LOG4CXX_INFO(logger,fmt::format(_("Loading driver library \"{0}\""),driverLibName));

	listItem.shLibName = driverLibName;
	// Clear the error information before calling dlopen.
	(void) dlerror();
	listItem.shLibHandle = dlopen(driverLibName,RTLD_NOW);

	LOG4CXX_DEBUG(logger,"dlopen for DLL \"" << driverLibName << "\" returns " << listItem.shLibHandle);

	if (listItem.shLibHandle == nullptr) {
		errStr = dlerror();
		if (errStr == nullptr) {
			errStr = _("No error information available");
		}

		auto str = fmt::format( _("Error loading shared library \"{0}\": {1}"),driverLibName,errStr);

		LOG4CXX_ERROR(logger,str);
		throw GliderVarioDriverLoadException(__FILE__,__LINE__,str.c_str());

	}

	char const* symName = "driverLibInit";
	// According to manual pages clear the error before calling dlsym
	// The symbol may have a value NULL, thus a returned NULL value is not necessarily an real error.
	(void) dlerror();
	void * sym = dlsym(listItem.shLibHandle,symName);
	if (sym == nullptr) {
		errStr = dlerror();
		if (errStr == nullptr) {
			errStr = _("No error information available");
		}
		auto str = fmt::format(_("Error loading symbol \"{0}\" from shared library library \"{1}\": {2}"),
				symName, driverLibName,errStr);

		LOG4CXX_ERROR(logger,str);

		dlclose(listItem.shLibHandle);

		throw GliderVarioDriverLoadException(__FILE__,__LINE__,str.c_str());

	}

	listItem.driverLibInit = reinterpret_cast<DriverLibInitProc> (sym);


	symName = "getDriverLib";
	(void) dlerror();
	sym = dlsym(listItem.shLibHandle,symName);
	if (sym == nullptr) {
		errStr = dlerror();
		if (errStr == nullptr) {
			errStr = _("No error information available");
		}
		auto str = fmt::format(_("Error loading symbol \"{0}\" from shared library library \"{1}\": {2}"),
				symName, driverLibName,errStr);

		LOG4CXX_ERROR(logger,str);

		dlclose(listItem.shLibHandle);

		throw GliderVarioDriverLoadException(__FILE__,__LINE__,str.c_str());

	}

	listItem.getDriverLib = reinterpret_cast<GetDriverLibProc>(sym);

	listItem.driverLibInit();
	listItem.libObj = listItem.getDriverLib();

	if (listItem.libObj == nullptr) {
		auto str = fmt::format(_("Library \"{0}\" returned NULL for the library object."),driverLibName);

		LOG4CXX_ERROR(logger,str);

		dlclose(listItem.shLibHandle);

		throw GliderVarioDriverLoadException(__FILE__,__LINE__,str.c_str());

	}

	LOG4CXX_INFO(logger,fmt::format(_("Obtained library object \"{0}\": {1}"), driverLibName, listItem.libObj->getDescription()));

	{
		DriverLibList::value_type newListItem {listItem.libObj->getLibName(),listItem};
		driverLibList.insert(newListItem);
	}

	listItem.libObj->addDrivers(*this);


}

void GliderVarioDriverList::loadDriverInstance(char const *driverInstanceName, Properties4CXX::Properties const &configuration) {

	Properties4CXX::Property const * driverConfig;

	LOG4CXX_INFO(logger,fmt::format(_("Loading driver instance \"{0}\""),driverInstanceName));

	try {
		driverConfig = configuration.searchProperty(driverInstanceName);
	} catch (Properties4CXX::ExceptionBase const& e) {
		auto str = fmt::format(_("Could not find property for driver instance \"{0}\""),driverInstanceName);
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioDriverLoadException(__FILE__,__LINE__,str.c_str() );
	}

	if (!driverConfig->isStruct()) {
		auto str =  fmt::format(_("Property for driver instance \"{0}\" is not a structure"),driverInstanceName);
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioDriverLoadException(__FILE__,__LINE__,str.c_str());
	}

	Properties4CXX::Properties const &driverConfigStruct = driverConfig->getPropertiesStructure();

	Properties4CXX::Property const * driverName;
	try {
		driverName =  driverConfigStruct.searchProperty("driver");
	} catch (Properties4CXX::ExceptionBase const& e) {
		auto str = fmt::format(_("Could not find property \"{0}\" within structure \"{1}\"."), "driver",driverInstanceName);
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioDriverLoadException(__FILE__,__LINE__,str.c_str());
	}

	LOG4CXX_INFO (logger,fmt::format(_("Driver of driver instance {0} is {1}"),driverInstanceName,driverName->getStringValue()));

	auto driverIter = driverList.find(driverName->getStringValue());
	if (driverIter == driverList.end()) {
		auto str = fmt::format(_("Driver \"{0}\" for instance {1} does not exist."),
				driverName->getStringValue(),driverName->getStringValue());

		LOG4CXX_ERROR(logger,str);

		throw GliderVarioDriverLoadException(__FILE__,__LINE__,str.c_str());
	}

	auto driverInstance = driverIter->second.getNewDriverInstance(
			driverIter->second.driverName.c_str(),
			driverIter->second.description.c_str(),driverInstanceName);

	if (!driverInstance) {
		auto str = fmt::format(_("Instance {0} of driver \"{1}\" could not be created."),
				driverInstanceName,driverIter->second.driverName);
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioDriverLoadException (__FILE__,__LINE__,str.c_str());
	}

	LOG4CXX_INFO (logger,fmt::format(
			_("Created driver instance \"{0}\" for driver \"{1}\" from library \"{2}\""),
			driverInstance->getInstanceName(),driverInstance->getDriverName(),
			driverInstance->GetDriverLib().getLibName()));

	DriverInstanceList::value_type newInstanceListItem {driverInstanceName,driverInstance};


	driverInstance->readCommonConfiguration(driverConfigStruct);
	driverInstance->readConfiguration(driverConfigStruct);
	driverInstance->readCalibrationData();

	// If you are here no major errors occurred during creation of the device=driver instance
	// and while reading the configuration.
	// Add the instance to the list.
	driverInstanceList.insert(newInstanceListItem);

	// Check if the driver will run the idle loop itself. Otherwise the main program runs the idle loop.
	if (driverInstance->hasSensorCapability(drivers::DriverBase::RUN_IDLE_LOOP) ) {

#if defined HAVE_LOG4CXX_H
		if (!programOptions.runIdleLoop) {
			LOG4CXX_WARN(logger,fmt::format(
					_("Driver instance \"{0}\" implements the idle loop. Another driver implements the idle loop too."),
					driverInstance->getInstanceName()));
		} else {
			LOG4CXX_DEBUG(logger,"Driver instance \"" << driverInstance->getInstanceName() << "\" implements the idle loop." );
		}
#endif /* defined HAVE_LOG4CXX_H */
		programOptions.runIdleLoop = false;
	}

}

void GliderVarioDriverList::initDrivers (GliderVarioMainPriv &varioMain) {
	auto iter = driverInstanceList.begin();

	while (iter != driverInstanceList.end()) {
		try {
			iter->second->driverInit(varioMain);
			iter ++;
		} catch (std::exception const &e) {

			if (programOptions.terminateOnDriverLoadError) {
				LOG4CXX_FATAL(logger,fmt::format(_("Failed to initialize device \"{0}\" because: {1}"),iter->second->getInstanceName(), e.what()));
				LOG4CXX_FATAL(logger,_("Program will be terminated."));
				LOG4CXX_FATAL(logger,_("Bye bye!"));
				throw;
			} else {
				LOG4CXX_ERROR(logger,fmt::format(_("Failed to initialize device \"{0}\" because: {1}"),iter->second->getInstanceName(), e.what()));
				LOG4CXX_ERROR(logger,fmt::format(_("Deleting device \"{0}\" from the device list"),iter->second->getInstanceName()));
				auto delIter = iter;
				iter ++;

				driverInstanceList.erase(delIter);
			}

		}

	}
}

void GliderVarioDriverList::startupDrivers (GliderVarioMainPriv &varioMain) {

	for (DriverInstanceList::value_type const& i: driverInstanceList) {
		i.second->startup(varioMain);
	}

}

void GliderVarioDriverList::initializeKalmanStatus(
		GliderVarioStatus &currentStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain) {

	for (DriverInstanceList::value_type const& i: driverInstanceList) {
		i.second->initializeStatus(currentStatus,measurements,varioMain);

	}

}

void GliderVarioDriverList::startupCalibrationDataUpdateThread() {

	if (!calibrationDataUpdateThread.joinable()) {
		calibrationDataUpdateThread = std::thread(GliderVarioDriverList::calibrationDataUpdateThreadEntry,this);
	}

}

void GliderVarioDriverList::runDrivers () {


	for (DriverInstanceList::value_type const& i: driverInstanceList) {
		i.second->run();
	}

}

void GliderVarioDriverList::stopDrivers () {

	for (DriverInstanceList::value_type const& i: driverInstanceList) {
		i.second->shutdown();

	}

}

bool GliderVarioDriverList::isDriverRunningIdleLoop() {


	for (DriverInstanceList::value_type const& i: driverInstanceList) {
		if (i.second->hasSensorCapability(drivers::DriverBase::RUN_IDLE_LOOP)) {
			return true;
		}
	}

	return false;
}

void GliderVarioDriverList::calibrationDataUpdateThreadFunc() {
	auto now = OEVClock::now();
	constexpr OEVDuration oneHour = static_cast<OEVDuration>(std::chrono::hours(1));
	OEVClock::time_point nextCycleTime = now + oneHour;

	for (DriverInstanceList::value_type const& i :driverInstanceList) {
		i.second->setCalibrationUpdateNextTime(now);
	}

	// Enter the endless loop
	for (;;) {

		// Run through all driver instances and get the lowest next wakeup time
		// The maximim cycle time is one hour, even if there is no driver that requires an update.
		for (DriverInstanceList::value_type const& i :driverInstanceList) {

			if (i.second->getDoCyclicUpdateCalibrationDataFile() &&
					i.second->getNextCalibrationDataWriteTime() < nextCycleTime) {

				LOG4CXX_DEBUG(logger,__FUNCTION__ << " Driver instance " << i.second->getInstanceName()
						<< " next cycle time is the smallest so far = "
						<< timePointToString(i.second->getNextCalibrationDataWriteTime()));

				nextCycleTime = i.second->getNextCalibrationDataWriteTime();
			}
		}

		LOG4CXX_DEBUG(logger,"\tSleep until " << timePointToString(nextCycleTime));
		std::this_thread::sleep_until(nextCycleTime);

		// Run through all driver instances, and
		for (DriverInstanceList::value_type const& i :driverInstanceList) {

			if (i.second->getDoCyclicUpdateCalibrationDataFile() &&
					i.second->getNextCalibrationDataWriteTime() <= nextCycleTime) {

				LOG4CXX_DEBUG(logger, "\tCalibration data of driver instance "
						<< i.second->getInstanceName() << " are being written now");

				i.second->updateAndWriteCalibrationData();
				i.second->setCalibrationUpdateNextTime(nextCycleTime);
			}
		}

		// Setup the default for the next update cycle.
		nextCycleTime += oneHour;
	}
	// Now collect the update times


}

void GliderVarioDriverList::calibrationDataUpdateThreadEntry (GliderVarioDriverList* tis) {
	tis->calibrationDataUpdateThreadFunc();
}

} /* namespace openEV */
