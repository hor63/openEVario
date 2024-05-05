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
#  include "config.h"
#endif

#include <fstream>
#include <chrono>
#include <thread>

#include <fmt/format.h>

#include "MPL3115Driver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

namespace openEV::drivers::MPL3115 {

MPL3115Driver::MPL3115Driver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: DriverBase {driverName,description,instanceName,MPL3115Lib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	logger = log4cxx::Logger::getLogger("openEV.Drivers.MPL3115");
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(STATIC_PRESSURE	);

	// Default cycle time as documented in the template parameter file
	using namespace std::chrono_literals;
	updateCyle = 100ms;

}


MPL3115Driver::~MPL3115Driver() {

}


void MPL3115Driver::driverInit(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

	ioPort = getIoPort<decltype(ioPort)>(logger);

}

void MPL3115Driver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, fmt::format (_("{0}: for driver instance \"{1}\""),
			__PRETTY_FUNCTION__, instanceName));

	LOG4CXX_INFO(logger,"	portName = " << portName);
	LOG4CXX_INFO(logger,"	i2cAddress = 0x" << std::hex <<  uint32_t(i2cAddress) << std::dec);
	LOG4CXX_INFO(logger,"	useTemperatureSensor = " << useTemperatureSensor);
	LOG4CXX_INFO(logger,"	errorTimeout = " << ((errorTimeout.count() * decltype(errorTimeout)::period::num) / decltype(errorTimeout)::period::den));
	LOG4CXX_INFO(logger,"	errorMaxNumRetries = " << errorMaxNumRetries);

}

#define SQUARE(x) ((x)*(x))

void MPL3115Driver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain) {

	// Wait for 20 seconds for 16 samples to appear, and a defined temperature value
	for (int i = 0; i < 20; i++) {
		if (numValidInitValues < NumInitValues || UnInitVal == temperatureVal) {
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

		for (int i = 0 ; i < NumInitValues; i++) {
			avgPressure += initValues[i];
			LOG4CXX_TRACE(logger," initValues[" << i << "] = " << initValues[i]);
		}
		avgPressure /= FloatType(NumInitValues);
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": avgPressure = " << avgPressure);

		if (UnInitVal != measurements.gpsMSL) {
			initQFF(varioStatus,measurements,varioMain,avgPressure);
		}

		if (UnInitVal == varioStatus.altMSL) {
			if (UnInitVal != varioStatus.qff && UnInitVal != temperatureVal) {
			auto const currTempK = temperatureVal + CtoK;
			varioStatus.altMSL  = (currTempK -(pow((avgPressure / varioStatus.qff),(1.0/BarometricFormulaExponent)) * currTempK)) / TempLapseIndiffBoundLayer;
			varioStatus.lastPressure = avgPressure;
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Initial altitude from QFF:" << varioStatus.qff
					<< ", Temp (K): " << currTempK
					<< " = " << varioStatus.altMSL);

			// 1 mbar initial uncertainty translated into around 8m.
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

	} else {
		LOG4CXX_WARN(logger, fmt::format(_(
				"{0}: Could not obtain {1} valid measurements in a row for 20 seconds. Cannot initialize the Kalman filter state."),
				__PRETTY_FUNCTION__,NumInitValues));
	}


}

void MPL3115Driver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	// Nothing to do here

}


void MPL3115Driver::driverThreadFunction() {

	int numRetries = 0;

	if (ioPort == nullptr) {
		LOG4CXX_ERROR (logger,fmt::format(_(
				"No valid I/O port for driver instance \"{0}\". The driver is not operable"),instanceName));
	} else {
		while (!getStopDriverThread() && ( errorMaxNumRetries == 0 || numRetries <= errorMaxNumRetries)) {
			try {
				ioPort->open();
				numRetries = 0;
				processingMainLoop ();
				ioPort->close();
			} catch (std::exception const& e) {
				numRetries ++;
				LOG4CXX_ERROR(logger,fmt::format(_("Error in the main loop of driver instance \"{0}\": {1}"),
						instanceName,e.what()));
				ioPort->close();

				std::this_thread::sleep_for(errorTimeout);
			}
		}
	}
}

void MPL3115Driver::processingMainLoop() {
    using namespace std::chrono_literals;

	setupMPL3115();

	auto nextStartConversion = OEVClock::now();

	while (!getStopDriverThread()) {

		startConversionMPL3155();

		std::this_thread::sleep_until(nextStartConversion + 60ms);

		readoutMPL3155();

		// In case that you miss a cycle advance to the next cycle
		auto now = OEVClock::now();
		do {
			nextStartConversion += updateCyle;
		} while (nextStartConversion < now);
		std::this_thread::sleep_until(nextStartConversion);
	}

}

/// Like in the AVR libraries
#define _BV(x) (1<<(x))

void MPL3115Driver::setupMPL3115() {
	uint8_t ctrl1AddrVal[2];
	uint8_t ptDataCfgAddrVal[2];
	uint8_t whoAmI = ioPort->readByteAtRegAddrByte(i2cAddress, MPL3115_WHO_AM_I);
	if (whoAmI == MPL3115WhoAmIValue) {
		LOG4CXX_INFO(logger,fmt::format(_(
				"{0}: WHO AM I contains expected {1:#04X}."),
				__PRETTY_FUNCTION__,static_cast<uint32_t>(whoAmI)));
	} else {
		auto str = fmt::format(_(
				"{0}: WHO AM I value is not {1:#04X}, but {2:#04X}. The device is obviously not a {3} sensor."),
				__PRETTY_FUNCTION__,static_cast<uint32_t>(MPL3115WhoAmIValue),static_cast<uint32_t>(whoAmI),
				"MPL3115");
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioExceptionBase(__FILE__,__LINE__,str.c_str());
	}
	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Who am I value is as expected 0x" << std::hex << uint32_t(whoAmI) << std::dec);

	// First set the thing to standby mode.
	ctrl1AddrVal[1] = ioPort->readByteAtRegAddrByte(i2cAddress, MPL3115_CTRL_REG1);
	ctrl1AddrVal[1] &= ~_BV(MPL3115Cfg1_SBYB);
	ctrl1AddrVal[0] = MPL3115_CTRL_REG1;
	ioPort->writeBlock(i2cAddress, ctrl1AddrVal, sizeof(ctrl1AddrVal));

	// Set up with 16-times oversampling
	ctrl1AddrVal[1] &= ~(0b111 << MPL3115Cfg1_OS); // Do not forget to clear existing bits first :)
	ctrl1AddrVal[1] |= 0b100 << MPL3115Cfg1_OS;
	// ... and barometer mode (delete Altitude mode flag)
	ctrl1AddrVal[1] &= ~_BV(MPL3115Cfg1_ALT);
	ioPort->writeBlock(i2cAddress, ctrl1AddrVal, sizeof(ctrl1AddrVal));

	// Setup data event configuration
	ptDataCfgAddrVal[0] = MPL3115_PT_DATA_CFG;
	ptDataCfgAddrVal[1] =
			_BV(MPL3115PTDataCfg_DREM) |
			_BV(MPL3115PTDataCfg_PDEFE) |
			_BV(MPL3115PTDataCfg_TDEFE);
	ioPort->writeBlock(i2cAddress, ptDataCfgAddrVal, sizeof(ptDataCfgAddrVal));

	// Set the sensor to active mode
	ctrl1AddrVal[1] |= _BV(MPL3115Cfg1_SBYB);
	ioPort->writeBlock(i2cAddress, ctrl1AddrVal, sizeof(ctrl1AddrVal));

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Set Sensor to 16-times oversampling, and polling mode. Activate it.");

}

void MPL3115Driver::startConversionMPL3155() {

	uint8_t ctrl1AddrVal[2];

	ctrl1AddrVal[0] = MPL3115_CTRL_REG1;

	ctrl1AddrVal[1] = ioPort->readByteAtRegAddrByte(i2cAddress, MPL3115_CTRL_REG1);
	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Ctrl1 = 0x" << std::hex << uint32_t(ctrl1AddrVal[1]) << std::dec);

	// Delete the OST flag
	ctrl1AddrVal[1] &= ~_BV(MPL3115Cfg1_OST);
	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Reset OST. Set Ctrl1 to 0x" << std::hex << uint32_t(ctrl1AddrVal[1]) << std::dec);

	ioPort->writeBlock(i2cAddress, ctrl1AddrVal, sizeof(ctrl1AddrVal));

	// 	And set the OST flag again
	ctrl1AddrVal[1] |= _BV(MPL3115Cfg1_OST);
	LOG4CXX_TRACE(logger,__FUNCTION__ << ": Set OST. Set Ctrl1 to 0x" << std::hex << uint32_t(ctrl1AddrVal[1]) << std::dec);

	ioPort->writeBlock(i2cAddress, ctrl1AddrVal, sizeof(ctrl1AddrVal));

}

void MPL3115Driver::readoutMPL3155() {

	uint8_t statusVal = 0;
	uint8_t sensorValues[5];

	while ((statusVal & _BV(MPL3115Status_PTDR)) == 0) {
		statusVal = ioPort->readByteAtRegAddrByte(i2cAddress, MPL3115_STATUS);
	}

	ioPort->readBlockAtRegAddrByte(i2cAddress, MPL3115_OUT_P_MSB, sensorValues, sizeof(sensorValues));

	if ((statusVal & _BV(MPL3115Status_TDR)) != 0) {
		// Temperature comes as 12 bit *signed* fixed point decimal. Oh my.
		// I cannot use simple bit manipulation because I need to take care of expanding the sign bit to
		// a signed 16 bit integer.
		int16_t tempValInt = *reinterpret_cast<int8_t*> (&sensorValues[3]);

		tempValInt *= 16; // Shift 4 bits left in a signed integer fashion. The compiler will work out that it is a shift.
		// Now merge the upper 4 bits of the next byte binary into the signed integer.
		* reinterpret_cast <uint16_t*> (&tempValInt) = (* reinterpret_cast <uint16_t*> (&tempValInt)) | (sensorValues[4] >> 4);

		temperatureVal = static_cast<FloatType>(tempValInt) / 16.0f;

		LOG4CXX_TRACE(logger,__FUNCTION__ << ": Temperature is "
				<< std::hex << uint32_t(sensorValues[3]) << ':' << uint32_t(sensorValues[4])
				<< std::dec << " =  " << temperatureVal << " DegC.");

	}

	if ((statusVal & _BV(MPL3115Status_PDR)) != 0) {
		uint32_t pressureValInt;
		pressureValInt = (((uint32_t(sensorValues[0]) << 8) | uint32_t(sensorValues[1])) << 4) | (uint32_t(sensorValues[2]) >> 4);
		pressureVal = static_cast<FloatType>(pressureValInt) / 400.0f; // Integer value is Pa*4. Value in mBar.

		// Only accept values within the operational range
		if (pressureVal >= 500.0f && pressureVal <= 1500.0f) {

			LOG4CXX_TRACE(logger,__FUNCTION__ << ": Pressure registers "
					<< std::hex << uint32_t(sensorValues[0]) << ':' << uint32_t(sensorValues[1]) << ':' << uint32_t(sensorValues[2])
					<< " = 0x" << std::hex << pressureValInt << std::dec << " = " << pressureValInt
					);
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Pressure = " << pressureVal << " mBar."
					);

			if (getIsKalmanUpdateRunning()) {
				GliderVarioMainPriv::LockedCurrentStatus lockedStatus(*varioMain);
				FloatType &tempLocalC = lockedStatus.getMeasurementVector()->tempLocalC;

				if (useTemperatureSensor && temperatureVal != UnInitVal) {
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

		} else { // if (pressureVal >= 500.0f && pressureVal <= 1500.0f) {
			// and if you are out of the operational range re-start initialization data collection.
			// More likely the sensor is screwed you you connected the static system to a pump or the oxygen system :D
			numValidInitValues = 0;
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

	GliderVarioStatus::StatusCoVarianceType &errorCov = varioStatus.getErrorCovariance_P();

	FloatType pressureFactor = calcBarometricFactor(
    		measurements.gpsMSL,
			temperatureVal
			);
	LOG4CXX_DEBUG (logger,__FUNCTION__ << " pressureFactor = " << pressureFactor);

	varioStatus.qff = avgPressure / pressureFactor;

	// Assume quite a bit lower variance of qff pressure as the initial altitude variance (9)
	errorCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) = 1.0f;

	LOG4CXX_DEBUG (logger,"	QFF = " << varioStatus.qff
			<< ", initial variance = "
			<< errorCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF));

}

} /* namespace openEV */
