/*
 * MPL3115Driver.cpp
 *
 *  Created on: Dec 28, 2020
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2020  Kai Horstmann
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

#include <fstream>
#include <chrono>
#include <thread>

#include "MPL3115Driver.h"
#include "MPL3115A2.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.MPL3115");
	}
}

#endif

namespace openEV::drivers::MPL3115 {

MPL3115Driver::MPL3115Driver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,MPL3115Lib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(STATIC_PRESSURE	);

}


MPL3115Driver::~MPL3115Driver() {

}


void MPL3115Driver::driverInit(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

}

void MPL3115Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, __FUNCTION__ << " Driver" << driverName << " read configuraion");

	try {
		auto portNameConfig = configuration.searchProperty("portName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"PortName\" is a struct or a string list.");
		}

		portName = portNameConfig->getStringValue();

		ioPort = dynamic_cast<io::I2CPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not an I2C port.");
		}
	} catch (std::exception const& e) {
		LOG4CXX_ERROR(logger, "Read configuration of driver \"" << driverName
				<< "\" failed:"
				<< e.what());
		throw;
	}

	useTemperatureSensor = configuration.getPropertyValue(
    		std::string("useTemperatureSensor"),
			useTemperatureSensor);
    errorTimeout = configuration.getPropertyValue(
    		std::string("errorTimeout"),
			(long long)(errorTimeout));
    errorMaxNumRetries = configuration.getPropertyValue(
    		std::string("errorTimeout"),
			(long long)(errorMaxNumRetries));

	LOG4CXX_INFO(logger,"	portName = " << portName);
	LOG4CXX_INFO(logger,"	useTemperatureSensor = " << useTemperatureSensor);
	LOG4CXX_INFO(logger,"	errorTimeout = " << errorTimeout);
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);

}

#define SQUARE(x) ((x)*(x))

void MPL3115Driver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain) {

	// Wait for 20 seconds for 16 samples to appear, and a defined temperature value
	for (int i = 0; i < 20; i++) {
		if (numValidInitValues < NumInitValues || isnan(temperatureVal)) {
			using namespace std::chrono_literals; // used for the term "1s" below. 's' being the second literal.

			LOG4CXX_TRACE(logger,__FUNCTION__ << ": Only " << numValidInitValues <<
					" valid samples collected. Wait another second");
			std::this_thread::sleep_for(1s);
		} else {
			break;
		}
	}

	if (numValidInitValues >= NumInitValues) {
		FloatType avgPressure = 0.0f;
		double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

		for (int i = 0 ; i < NumInitValues; i++) {
			avgPressure += initValues[i];
			LOG4CXX_TRACE(logger," initValues[" << i << "] = " << initValues[i]);
		}
		avgPressure /= FloatType(NumInitValues);
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": avgPressure = " << avgPressure);

		if (!isnan(measurements.gpsMSL)) {
			initQFF(varioStatus,measurements,varioMain,avgPressure);
		}

		if (isnan(varioStatus.altMSL)) {
			if (!isnan(varioStatus.qff) && !isnan(temperatureVal) /*!isnan(measurements.tempLocalC)*/) {
			auto const currTempK = temperatureVal /*measurements.tempLocalC*/ + CtoK;
			varioStatus.altMSL  = (currTempK -(pow((avgPressure / varioStatus.qff),(1.0/BarometricFormulaExponent)) * currTempK)) / TempLapseIndiffBoundLayer;
			varioStatus.lastPressure = avgPressure;
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Initial altitude from QFF:" << varioStatus.qff
					<< ", Temp (K): " << currTempK
					<< " = " << varioStatus.altMSL);

			// 1 mbar initial uncertainty translated into 8m.
			varioStatus.getErrorCovariance_P().
							coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) = SQUARE(1.0*8.0);

			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Initial variance = "
					<< varioStatus.getErrorCovariance_P().
					coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL));
			} else {
				LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Either QFF or local temperature or both are undefined. QFF = "
						<< varioStatus.qff << ", temperature = " << temperatureVal);
			}

		} else {
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": altMSL is already defined = " << varioStatus.altMSL);
		}

		if (isnan(varioStatus.getSystemNoiseCovariance_Q().
				coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL)) ||
				(varioStatus.getSystemNoiseCovariance_Q().
				coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) > SQUARE(4.0) * baseIntervalSec)) {

			varioStatus.getSystemNoiseCovariance_Q().
							coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) = SQUARE(2.0) * baseIntervalSec;
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": System noise increment = "
					<< varioStatus.getSystemNoiseCovariance_Q().
												coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL));
		}

	} else {
		LOG4CXX_WARN(logger,__FUNCTION__ << "Could not obtain " << NumInitValues
				<< " valid measurements in a row for 20 seconds. Cannot initialize the Kalman filter state.");
	}


}

void MPL3115Driver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	// Nothing to do here

}


void MPL3115Driver::driverThreadFunction() {

	int numRetries = 0;

	if (ioPort == nullptr) {
		LOG4CXX_ERROR (logger,"No valid I/O port for driver " << getDriverName()
				<< ". The driver is not operable");
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				ioPort->open();
				numRetries = 0;
				processingMainLoop ();
				ioPort->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,"Error in main loop of driver \"" << getDriverName()
						<< "\":" << e.what());
				ioPort->close();

				sleep(errorTimeout);
			}
		}
	}
}

void MPL3115Driver::processingMainLoop() {
    std::chrono::system_clock::time_point nextStartConversion;

    using namespace std::chrono_literals;

	setupMPL3115();

	nextStartConversion = std::chrono::system_clock::now();

	while (!getStopDriverThread()) {

		startConversionMPL3155();

		std::this_thread::sleep_until(nextStartConversion + 60ms);

		readoutMPL3155();
		nextStartConversion += 100ms;
		std::this_thread::sleep_until(nextStartConversion);
	}

}

/// Like in the AVR libraries
#define _BV(x) (1<<(x))

void MPL3115Driver::setupMPL3115() {
	uint8_t ctrl1AddrVal[2];
	uint8_t ptDataCfgAddrVal[2];
	uint8_t whoAmI = ioPort->readByteAtRegAddrByte(MPL3115A2I2CAddr, MPL3115_WHO_AM_I);
	if (whoAmI != MPL3115WhoAmIValue) {
		std::ostringstream str;
		str << __FUNCTION__ << ": Who am I value is not 0x" << std::hex << uint32_t(MPL3115WhoAmIValue)
				<< ", but 0x" << std::hex << uint32_t(whoAmI) << std::dec;
		LOG4CXX_ERROR(logger,str.str());
		throw GliderVarioExceptionBase(__FILE__,__LINE__,str.str().c_str());
	}
	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Who am I value is as expected 0x" << std::hex << uint32_t(whoAmI) << std::dec);

	// First set the thing to standby mode.
	ctrl1AddrVal[1] = ioPort->readByteAtRegAddrByte(MPL3115A2I2CAddr, MPL3115_CTRL_REG1);
	ctrl1AddrVal[1] &= ~_BV(MPL3115Cfg1_SBYB);
	ctrl1AddrVal[0] = MPL3115_CTRL_REG1;
	ioPort->writeBlock(MPL3115A2I2CAddr, ctrl1AddrVal, sizeof(ctrl1AddrVal));

	// Set up with 16-times oversampling
	ctrl1AddrVal[1] &= ~(0b111 << MPL3115Cfg1_OS); // Do not forget to clear existing bits first :)
	ctrl1AddrVal[1] |= 0b100 << MPL3115Cfg1_OS;
	// ... and barometer mode (delete Altitude mode flag)
	ctrl1AddrVal[1] &= ~_BV(MPL3115Cfg1_ALT);
	ioPort->writeBlock(MPL3115A2I2CAddr, ctrl1AddrVal, sizeof(ctrl1AddrVal));

	// Setup data event configuration
	ptDataCfgAddrVal[0] = MPL3115_PT_DATA_CFG;
	ptDataCfgAddrVal[1] =
			_BV(MPL3115PTDataCfg_DREM) |
			_BV(MPL3115PTDataCfg_PDEFE) |
			_BV(MPL3115PTDataCfg_TDEFE);
	ioPort->writeBlock(MPL3115A2I2CAddr, ptDataCfgAddrVal, sizeof(ptDataCfgAddrVal));

	// Set the sensor to active mode
	ctrl1AddrVal[1] |= _BV(MPL3115Cfg1_SBYB);
	ioPort->writeBlock(MPL3115A2I2CAddr, ctrl1AddrVal, sizeof(ctrl1AddrVal));

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Set Sensor to 16-times oversampling, and polling mode. Activate it.");

}

void MPL3115Driver::startConversionMPL3155() {

	uint8_t ctrl1AddrVal[2];

	ctrl1AddrVal[0] = MPL3115_CTRL_REG1;

	ctrl1AddrVal[1] = ioPort->readByteAtRegAddrByte(MPL3115A2I2CAddr, MPL3115_CTRL_REG1);
	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Ctrl1 = 0x" << std::hex << uint32_t(ctrl1AddrVal[1]) << std::dec);

	// Delete the OST flag
	ctrl1AddrVal[1] &= ~_BV(MPL3115Cfg1_OST);
	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Reset OST. Set Ctrl1 to 0x" << std::hex << uint32_t(ctrl1AddrVal[1]) << std::dec);

	ioPort->writeBlock(MPL3115A2I2CAddr, ctrl1AddrVal, sizeof(ctrl1AddrVal));

	// 	And set the OST flag again
	ctrl1AddrVal[1] |= _BV(MPL3115Cfg1_OST);
	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Set OST. Set Ctrl1 to 0x" << std::hex << uint32_t(ctrl1AddrVal[1]) << std::dec);

	ioPort->writeBlock(MPL3115A2I2CAddr, ctrl1AddrVal, sizeof(ctrl1AddrVal));

}

void MPL3115Driver::readoutMPL3155() {

	uint8_t statusVal = 0;
	uint8_t sensorValues[5];

	while ((statusVal & _BV(MPL3115Status_PTDR)) == 0) {
		statusVal = ioPort->readByteAtRegAddrByte(MPL3115A2I2CAddr, MPL3115_STATUS);
	}

	ioPort->readBlockAtRegAddrByte(MPL3115A2I2CAddr, MPL3115_OUT_P_MSB, sensorValues, sizeof(sensorValues));

	if ((statusVal & _BV(MPL3115Status_TDR)) != 0) {
		temperatureVal = (FloatType(sensorValues[3])) + (FloatType(sensorValues[4])) / 256.0f;

		LOG4CXX_TRACE(logger,__FUNCTION__ << ": Temperature is "
				<< std::hex << uint32_t(sensorValues[3]) << ':' << uint32_t(sensorValues[4])
				<< std::dec << " =  " << temperatureVal << " DegC.");

	}

	if ((statusVal & _BV(MPL3115Status_PDR)) != 0) {
		uint32_t pressureValInt;
		pressureValInt = (((uint32_t(sensorValues[0]) << 8) | uint32_t(sensorValues[1])) << 4) | (uint32_t(sensorValues[2]) >> 4);
		pressureVal = FloatType(pressureValInt) / 400.0f; // Integer value is Pa*4. Value in mBar.

		LOG4CXX_TRACE(logger,__FUNCTION__ << ": Pressure registers "
				<< std::hex << uint32_t(sensorValues[0]) << ':' << uint32_t(sensorValues[1]) << ':' << uint32_t(sensorValues[2])
				<< " = 0x" << std::hex << pressureValInt << std::dec << " = " << pressureValInt
				);
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal << " mBar."
				);

		if (getIsKalmanUpdateRunning()) {
			GliderVarioMainPriv::LockedCurrentStatus lockedStatus(*varioMain);
			FloatType &tempLocalC = lockedStatus.getMeasurementVector()->tempLocalC;

			if (useTemperatureSensor) {
				tempLocalC = temperatureVal;
			}

			GliderVarioMeasurementUpdater::staticPressureUpd(
					pressureVal, tempLocalC, SQUARE(0.5),
					*lockedStatus.getMeasurementVector(), *lockedStatus.getCurrentStatus());

		} else {
			if (numValidInitValues < NumInitValues) {
				initValues[numValidInitValues] = pressureVal;
				numValidInitValues ++;
			}
		}

	} else {
		// Missed cycle in the initialization phase. Start from scratch.
		numValidInitValues = 0;
	}

}

void MPL3115Driver::initQFF(
		GliderVarioStatus &varioStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain,
		FloatType avgPressure) {

	GliderVarioStatus::StatusCoVarianceType &systemNoiseCov = varioStatus.getSystemNoiseCovariance_Q();
	GliderVarioStatus::StatusCoVarianceType &errorCov = varioStatus.getErrorCovariance_P();
	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	FloatType pressureFactor = calcBarometricFactor(
    		measurements.gpsMSL,
			temperatureVal
			);
	LOG4CXX_DEBUG (logger,__FUNCTION__ << " pressureFactor = " << pressureFactor);

	varioStatus.qff = avgPressure / pressureFactor;

	// Assume quite a bit lower variance of qff pressure as the initial altitude variance (9)
	errorCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			= SQUARE(1);
	systemNoiseCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) =
			SQUARE(0.05) * baseIntervalSec; // 0.1hPa/sec

	LOG4CXX_DEBUG (logger,"	QFF = " << varioStatus.qff
			<< ", initial variance = "
			<< errorCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			<< ", variance increment = "
			<< systemNoiseCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			<< " / " << baseIntervalSec << "s");



}

} /* namespace openEV */
