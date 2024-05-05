/*
 * DifferentialPressureSensorBase.cpp
 *
 *  Created on: Jan 10, 2024
 *      Author: hor
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include "fmt/format.h"

#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

#include "DifferentialPressureSensorBase.h"

namespace openEV {
namespace drivers {

DifferentialPressureSensorBase::DifferentialPressureSensorBase (
	    char const *driverName,
		char const *description,
		char const *instanceName,
		DriverLibBase &driverLib
		)
		:DriverBase (
			    driverName,
				description,
				instanceName,
				driverLib
				) {

	setSensorCapability(DYNAMIC_PRESSURE);

	// Default cycle time as documented in the template parameter file
	using namespace std::chrono_literals;
	updateCyle = 100ms;

}

DifferentialPressureSensorBase::~DifferentialPressureSensorBase() {}

void DifferentialPressureSensorBase::driverInit(GliderVarioMainPriv &varioMain) {

	this->varioMain = &varioMain;

	ioPort = getIoPort<decltype(ioPort)>(logger);

	LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Loaded port with name " << portName
			<< ". Pointer = " << static_cast<void*>(ioPort));

}

#define SQUARE(x) ((x)*(x))

void DifferentialPressureSensorBase::initializeStatus(
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

		FloatType initialTAS = 0.0f;

		for (int i = 0 ; i < numValidInitValues; i++) {
			avgPressureOnStartup += FloatType(initValues[i]);
			LOG4CXX_TRACE(logger," initValues[" << i << "] = " << initValues[i]);
		}
		avgPressureOnStartup /= static_cast<FloatType>(numValidInitValues);
		LOG4CXX_DEBUG(logger,__FUNCTION__ << ": avgPressureOnStartup = " << avgPressureOnStartup << " mBar");

		// Use the avg pressure as offset only when the instrument is obviously not switched on during flight,
		// or during high-wind conditions on the field (> 20 km/h),
		// unless there is no calibration data of the bias available.
		// When there is no previous calibration data is available use the average obained at startup, and hope for the best.

		if (UnInitVal == pressureBias) {
			// No pre-loaded bias value from calibration data.
			// Assume initial startup in controlled environment.
			pressureBias = avgPressureOnStartup;
			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": No bias from calibration data available. pressureBias = " << pressureBias << " mBar");

			if (saveZeroOffsetCalibrationOnce) {
				updateAndWriteCalibrationData();
			}

		} else {

			// Dynamic pressure in mBar at about 20km/h on the ground at 0C. Variations at different temperatures
			// and atmospheric pressures are insignificant here.
			// I only want a threshold to differentiate between switching the device on in high-wind conditions or even in flight.
			static constexpr FloatType PressureLimit = 0.2;

			if (fabs(avgPressureOnStartup - pressureBias) < PressureLimit) {
				// Not too far off.
				// Assume the measured value is the new offset/bias of the sensor.
				LOG4CXX_DEBUG(logger,__FUNCTION__ << ": Old Pressure bias = " << pressureBias
						<< ", new pressureBias = " << avgPressureOnStartup << " mBar");
				pressureBias = avgPressureOnStartup;

				if (saveZeroOffsetCalibrationOnce) {
					updateAndWriteCalibrationData();
				}

			}
		}

		if (pressureBias != avgPressureOnStartup) {
			// There is a significant pressure on the sensor.
			// Convert it into IAS. On the ground this is approximately TAS
			// \p varioStatus.lastPressure is initialized to standard sea level pressure
			// When there is already an actual pressure value available, even better.
			FloatType currStaticPressure;
			if (UnInitVal != varioStatus.lastPressure) {
				currStaticPressure = varioStatus.lastPressure;
			} else {
				currStaticPressure = PressureStdMSL;
			}

			FloatType airDensity = currStaticPressure*100.0f / Rspec / (temperatureVal + CtoK);
			initialTAS = sqrtf(200.0f * avgPressureOnStartup / airDensity);

			LOG4CXX_DEBUG(logger,__FUNCTION__ << ": TAS @ "
					<< temperatureVal << "C, " << varioStatus.lastPressure << "mBar = "
					<< initialTAS << "m/s.");
		}

		// All data is collected. Initialize the status
		varioStatus.trueAirSpeed = initialTAS;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_TAS,varioStatus.STATUS_IND_TAS) = 9.0f;

	} else {
		LOG4CXX_WARN(logger, fmt::format(_("{0}: Could not obtain {1} valid measurements in a row for 20 seconds. Cannot initialize the Kalman filter state."),
						__FUNCTION__,NumInitValues));

	}

	if (UnInitVal == pressureBias) {
		pressureBias = 0.0f;
	}

	statusInitDone = true;

}

void DifferentialPressureSensorBase::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	// Nothing to do here

}

void DifferentialPressureSensorBase::fillCalibrationDataParameters () {

	calibrationData.pressureBias = pressureBias;

	writeConfigValue(*calibrationDataParameters,"zeroOffset",calibrationData.pressureBias);

	LOG4CXX_DEBUG (logger, __PRETTY_FUNCTION__ << " Fill calibration data for device \"" << instanceName << "\": \n"
			<< "\tpressureBias	= " << calibrationData.pressureBias << "\n");
}

void DifferentialPressureSensorBase::applyCalibrationData(){

	calibrationData.pressureBias = pressureBias;

	readOrCreateConfigValue(*calibrationDataParameters,"zeroOffset", calibrationData.pressureBias);

	pressureBias = static_cast<FloatType>(calibrationData.pressureBias);

	LOG4CXX_DEBUG (logger, __PRETTY_FUNCTION__ << " Set calibration data for device \"" << instanceName << "\": \n"
			<< "\tpressureBias		 = " << calibrationData.pressureBias << "\n");
}

} /* namespace drivers */
} /* namespace openEV */
