
/*
 * GliderVarioDriverBase.cpp
 *
 *  Created on: Oct 30, 2017
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2022  Kai Horstmann
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

#define BUILDING_OEV_DRIVER 1

#include <cerrno>
#include <system_error>
#include <sstream>
#include <typeinfo>

#include "fmt/format.h"

#include "drivers/DriverBase.h"

namespace openEV::drivers {

DriverBase::DriverBase (
	    char const *driverName,
		char const *description,
		char const *instanceName,
		DriverLibBase &driverLib
		)
: sensorCapabilities {0},
  driverName {driverName},
  description {description},
  instanceName {instanceName},
  driverLib {driverLib}
{
#if defined HAVE_LOG4CXX_H
		logger = log4cxx::Logger::getLogger("openEV.Drivers.DriverBase");
#endif
}

DriverBase::~DriverBase () {
}

void DriverBase::startup(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

	if (!isDriverThreadRunning && !driverThread.joinable()){
		driverThread = std::thread(DriverBase::driverThreadEntry,this);
	}

}


void DriverBase::driverThreadEntry (DriverBase* tis) {

	tis->isDriverThreadRunning = true;
	try {
		tis->driverThreadFunction();
	}
	catch (std::exception &e) {
		LOG4CXX_ERROR(tis->logger,fmt::format (_(
				"Uncaught exception in driver/instance \"{0}\"/\"{1}\". Message = {2}"),
				tis->driverName, tis->instanceName, e.what()));
	}
	catch (...) {
		LOG4CXX_ERROR(tis->logger,fmt::format (_(
				"Uncaught exception from unknown class/type in driver/instance \"{0}\"/\"{1}\""),
				tis->driverName, tis->instanceName));
	}

	LOG4CXX_INFO(tis->logger,fmt::format (_(
			"Driver/instance \"{0}\"/\"{1}\" left the driver thread function. The instance is now defunct."),
			tis->driverName, tis->instanceName));

	tis->isDriverThreadRunning = false;
	tis->stopDriverThread = false;
}


void DriverBase::run() {

	isKalmanUpdateRunning = true;

}

void DriverBase::suspend() {

	isKalmanUpdateRunning = false;

}

void DriverBase::resume() {

	isKalmanUpdateRunning = true;

}

void DriverBase::shutdown() {

	if (!stopDriverThread) {
		stopDriverThread = true;
		if (driverThread.joinable()) {
			try {
				driverThread.join();

			} catch  (std::system_error& e) {
				;
			}
		}
	}

	varioMain = nullptr;

}

void DriverBase::readOrCreateConfigValue(
		Properties4CXX::Properties& calibrationDataParameters,
		char const* parameterName,
		double& value
		) {

	try {
		Properties4CXX::Property const * prop = calibrationDataParameters.searchProperty(parameterName);
		value = prop->getDoubleValue();
	} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
		calibrationDataParameters.addProperty(new Properties4CXX::PropertyDouble(parameterName,value));
	}
	catch (std::exception const &e) {}

}

void DriverBase::writeConfigValue (
		Properties4CXX::Properties& calibrationDataParameters,
		char const* parameterName,
		double value
		) {
	calibrationDataParameters.deletePropery(parameterName);
	calibrationDataParameters.addProperty(new Properties4CXX::PropertyDouble(parameterName,value));
}

void DriverBase::updateAndWriteCalibrationData() {

	try {
		fillCalibrationDataParameters ();

		std::ofstream of(calibrationDataUpdateFileName,of.out | of.trunc);
		if (!of.good()) {
			auto err = errno;
			auto str = fmt::format(_(
					"Cannot open calibration update file \"{0}\" for driver instance \"{1}\". Error: {2}"),
					calibrationDataUpdateFileName, instanceName, strerror(err));
			throw GliderVarioDriverCalibrationFileException(__FILE__, __LINE__, str.c_str());
		}
		calibrationDataParameters->writeOut(of);
	} catch (std::exception const &e) {
		LOG4CXX_ERROR(logger,fmt::format(_(
				"Error in {0} for driver instance \"{1}\". Error = {2}"),
				__PRETTY_FUNCTION__, instanceName, e.what()));
		LOG4CXX_ERROR(logger,_("Disable further calibration data updates."));
		doCyclicUpdateCalibrationDataFile = false;
	}
	catch (...) {
		LOG4CXX_ERROR(logger,fmt::format(_(
				"Error in {0} for driver instance {1}. Unknown exception."),__PRETTY_FUNCTION__,instanceName));
		LOG4CXX_ERROR(logger,_("Disable further calibration data updates."));
		// Something went completely wrong
		doCyclicUpdateCalibrationDataFile = false;
	}

}

void DriverBase::applyCalibrationData() {
	LOG4CXX_WARN(logger, fmt::format(_(
			"Cannot apply calibration data from file \"{0}\" to device \"{1}\"."
			"Driver \"{2}\" does not implement reading of calibration data."),
			calibrationDataFileName, instanceName, driverName));

	useCalibrationDataFile = false;
}
void DriverBase::fillCalibrationDataParameters () {
	LOG4CXX_WARN(logger, fmt::format(_(
			"Cannot read calibration data from device \"{0}\""
			" for writing to update calibration data file \"{1}\"."
			" Driver \"{2}\" does not implement reading of calibration data."),
			instanceName,calibrationDataUpdateFileName,driverName));

	doCyclicUpdateCalibrationDataFile = false;
}

void DriverBase::readCalibrationData() {

	if (useCalibrationDataFile || doCyclicUpdateCalibrationDataFile) {

		std::string locCalibDataFileName;

		LOG4CXX_DEBUG(logger,__FUNCTION__
				<< ": calibrationDataUpdateFileName = " << calibrationDataUpdateFileName
				<< ", calibrationDataFileName = " << calibrationDataFileName);

		if (loadCalibrationDataUpdateFileBeforeStatic && !calibrationDataUpdateFileName.empty()) {
			locCalibDataFileName = calibrationDataUpdateFileName;
		} else {
			locCalibDataFileName = calibrationDataFileName;
		}

		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Calibration file name = " << locCalibDataFileName);

		calibrationDataParameters = std::unique_ptr<Properties4CXX::Properties>(new Properties4CXX::Properties(locCalibDataFileName));

	// Read the calibration data file, and extract the initial parameters
		try {
			// Nest another try-catch block in case that both initial and update file names are defined.
			try {
				calibrationDataParameters->readConfiguration();
				LOG4CXX_INFO(logger,fmt::format(_("Read from calibration data file \"{0}\""),locCalibDataFileName));
			} catch (std::exception const &e) {
				if (useCalibrationDataFile && !calibrationDataUpdateFileName.empty()) {
					// Both file names are defined. So one more try left.
					std::string failedFileName = locCalibDataFileName;
					if (loadCalibrationDataUpdateFileBeforeStatic) {
						locCalibDataFileName = calibrationDataFileName;
					} else {
						locCalibDataFileName = calibrationDataUpdateFileName;
					}
					LOG4CXX_INFO(logger,fmt::format(_(
							"Cannot read calibration data from file \"{0}\". Trying alternative {1}"),
							failedFileName, locCalibDataFileName));
					// Re-create an empty set of calibration data and try to read the other file.
					calibrationDataParameters = std::unique_ptr<Properties4CXX::Properties>(new Properties4CXX::Properties(locCalibDataFileName));
					calibrationDataParameters->readConfiguration();
					LOG4CXX_INFO(logger,fmt::format(_("Read from alternative calibration data file \"{0}\""), locCalibDataFileName));
				} else {
					// The end of trying to reading the configuration data.
					throw;
				}

			}
		} catch (std::exception const &e) {
			LOG4CXX_WARN(logger,fmt::format(_(
					"Device instance \"{0}\": Error reading calibration data from file \"{1}\": {2}"),
					instanceName, locCalibDataFileName,e.what()));
			LOG4CXX_WARN(logger,fmt::format(_(
					"Device instance \"{0}\": Using builtin default calibration data"),instanceName));
			// Re-create empty calibration data.
			calibrationDataParameters = std::unique_ptr<Properties4CXX::Properties>(new Properties4CXX::Properties());
		}

		// Now apply them driver specific to the driver.
		applyCalibrationData();

	} // if (useCalibrationDataFile || useCalibrationDataUpdateFile) {
}


void DriverBase::readCommonConfiguration(
			const Properties4CXX::Properties &configuration) {

	try {
		long long durTicks;

		portName = configuration.getPropertyValue(
				std::string("portName"),
				"");

		{
			auto durMSec = std::chrono::duration_cast<std::chrono::milliseconds>(updateCyle);
			durTicks = durMSec.count();
		}
		durTicks = configuration.getPropertyValue(
				std::string("updateCycle"),
				durTicks);
		updateCyle = static_cast<OEVDuration>(std::chrono::milliseconds(durTicks));

		{
		auto durSec = std::chrono::duration_cast<std::chrono::seconds>(errorTimeout);
		durTicks = durSec.count();
		}
		durTicks = configuration.getPropertyValue(
				std::string("errorTimeout"),
				durTicks);
		errorTimeout = static_cast<OEVDuration>(std::chrono::seconds(durTicks));

		errorMaxNumRetries = configuration.getPropertyValue(
				std::string("errorMaxNumRetries"),
				static_cast<long long>(errorMaxNumRetries));

		calibrationDataFileName = configuration.getPropertyValue(
				std::string("calibrationDataFile"),
				"");
		useCalibrationDataFile = !calibrationDataFileName.empty();

		calibrationDataUpdateFileName = configuration.getPropertyValue(
				std::string("calibrationDataUpdateFile"),
				"");
		durTicks = configuration.getPropertyValue(
				std::string("calibrationDataUpdateCycle"),0LL);
		calibrationDataWriteInterval = static_cast<OEVDuration>(std::chrono::seconds(durTicks));
		if (!calibrationDataUpdateFileName.empty()) {
			doCyclicUpdateCalibrationDataFile = durTicks > 0LL;

			saveZeroOffsetCalibrationOnce = configuration.getPropertyValue(
					std::string("saveZeroOffsetCalibrationOnce"),
					saveZeroOffsetCalibrationOnce);

		}

		loadCalibrationDataUpdateFileBeforeStatic = configuration.getPropertyValue(
				std::string("loadCalibrationDataUpdateFileBeforeStatic"),
				loadCalibrationDataUpdateFileBeforeStatic);

	} catch (std::exception const& e) {
		auto str = fmt::format(_("Error reading configuration of device instance \"{0}\": {1}"),instanceName, e.what());

		LOG4CXX_ERROR(logger, str);
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,str.c_str());
	}


	LOG4CXX_INFO (logger,fmt::format(_("Common parameters of driver instance \"{0}\":"),instanceName));
	LOG4CXX_INFO (logger,"\t portName = " << portName);
	LOG4CXX_INFO (logger,"\t updateCyle = " << (std::chrono::duration_cast<std::chrono::milliseconds>(updateCyle).count()) << "ms");
	LOG4CXX_INFO (logger,"\t errorTimeout = " << (std::chrono::duration_cast<std::chrono::seconds>(errorTimeout).count()) << "s");
	LOG4CXX_INFO (logger,"\t errorMaxNumRetries = " << errorMaxNumRetries);
	LOG4CXX_INFO (logger,"\t calibrationDataFileName = " << calibrationDataFileName);
	LOG4CXX_INFO (logger,"\t useCalibrationDataFile = " << useCalibrationDataFile);
	LOG4CXX_INFO (logger,"\t calibrationDataUpdateFileName = " << calibrationDataUpdateFileName);
	LOG4CXX_INFO (logger,"\t calibrationDataWriteInterval = " << (std::chrono::duration_cast<std::chrono::seconds>(calibrationDataWriteInterval).count()) << "s");
	LOG4CXX_INFO (logger,"\t saveZeroOffsetCalibrationOnce = " << saveZeroOffsetCalibrationOnce);
	LOG4CXX_INFO (logger,"\t doCyclicUpdateCalibrationDataFile = " << doCyclicUpdateCalibrationDataFile);
	LOG4CXX_INFO (logger,"\t loadCalibrationDataUpdateFileBeforeStatic = " << loadCalibrationDataUpdateFileBeforeStatic);


}


void DriverBase::setCalibrationUpdateNextTime(OEVClock::time_point refTime) {
	LOG4CXX_DEBUG(logger, __FUNCTION__ << ": Instance " << instanceName
			<< ": doCyclicUpdateCalibrationDataFile  = " << doCyclicUpdateCalibrationDataFile
			<< ", calibrationDataWriteInterval = "
			<< (std::chrono::duration_cast<std::chrono::milliseconds>(calibrationDataWriteInterval).count()) << "ms");
	if (doCyclicUpdateCalibrationDataFile) {
		nextCalibrationDataWriteTime = refTime + calibrationDataWriteInterval;
	} else {
		// Set the next calibration data update 10 years from now, i.e. never :D
		nextCalibrationDataWriteTime = refTime + std::chrono::hours(24*356*10);
	}

	LOG4CXX_DEBUG(logger," nextCalibrationDataWriteTime = " << timePointToString(nextCalibrationDataWriteTime));
}

#if !defined DOXYGEN
DriverBase::SensorCapabilityHelperClass DriverBase::SensorCapabilityHelperObj;
#endif

} // namespace openEV::drivers

std::ostream& operator << (std::ostream &o,openEV::drivers::DriverBase::SensorCapability ind) {
	o << openEV::drivers::DriverBase::SensorCapabilityHelperObj.getString (ind);
	return o;
}
