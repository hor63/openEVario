/*
 * BMXSensorBoardDriver.cpp
 *
 *  Created on: Feb 04, 2020
 *      Author: kai_horstmann
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

#include "Properties4CXX/Property.h"

#include "BMXSensorBoardDriver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"
#include "util/RotationMatrix.h"
#include "util/crc.h"

#include "horOvIp-I2C-Bridge/BMX160net.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.BMXSensorBoard");
	}
}

#endif

namespace openEV::drivers::BoschBMX160 {

BMXSensorBoardDriver::BMXSensorBoardDriver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,BMXSensorBoardLib::theOneAndOnly},
  calibrationDataUpdateCycle{0}
{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(ACCEL_X);
	setSensorCapability(ACCEL_Y);
	setSensorCapability(ACCEL_Z);
	setSensorCapability(GYRO_X);
	setSensorCapability(GYRO_Y);
	setSensorCapability(GYRO_Z);
	setSensorCapability(COMPASS_X);
	setSensorCapability(COMPASS_Y);
	setSensorCapability(COMPASS_Z);

	memset (&magTrimData,0,sizeof(magTrimData));

	for (int i = 0; i < SIZE_SENSOR_DATA_ARRAY; i++) {
		sensorDataArr[i].accelDataValid = false;
		sensorDataArr[i].accelX = 0.0f;
		sensorDataArr[i].accelY = 0.0f;
		sensorDataArr[i].accelZ = 0.0f;

		sensorDataArr[i].gyroDataValid = false;
		sensorDataArr[i].gyroX = 0.0f;
		sensorDataArr[i].gyroY = 0.0f;
		sensorDataArr[i].gyroZ = 0.0f;

		sensorDataArr[i].magDataValid = false;
		sensorDataArr[i].magX = 0.0f;
		sensorDataArr[i].magY = 0.0f;
		sensorDataArr[i].magZ = 0.0f;
	}
}


BMXSensorBoardDriver::~BMXSensorBoardDriver() {

	if (calibrationDataParameters) {
		delete calibrationDataParameters;
	}
}

/** \brief Little helper to reduce code size
 *
 * If the property \p parameterName does not exist in the properties set \p value will be unchanged.
 *
 * @param[in] calibrationDataParameters Properties which were read from the calibration parameter file
 * @param[in] parameterName Name of the calibration value
 * @param[in,out] value Value of the calibration value
 */
static void readOrCreateConfigValue(
		Properties4CXX::Properties* calibrationDataParameters,
		char const* parameterName,
		double& value
		) {

	try {
		Properties4CXX::Property const * prop = calibrationDataParameters->searchProperty(parameterName);
		value = prop->getDoubleValue();
	} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
		calibrationDataParameters->addProperty(new Properties4CXX::PropertyDouble(parameterName,value));
	}
	catch (std::exception const &e) {}

}

/** \brief Little helper to reduce code size
 *
 * New values are written by an existing property when it exists, and write a new one.
 *
 * @param[in,out] calibrationDataParameters Properties which hold the calibration data
 * @param[in] parameterName Name of the calibration value
 * @param[in] value New value of the calibration value
 */
static void writeConfigValue (
		Properties4CXX::Properties* calibrationDataParameters,
		char const* parameterName,
		double value
		) {
	calibrationDataParameters->deletePropery(parameterName);
	calibrationDataParameters->addProperty(new Properties4CXX::PropertyDouble(parameterName,value));
}


void BMXSensorBoardDriver::driverInit(GliderVarioMainPriv &varioMain) {

	// Read the calibration data file, and extract the initial parameters
	if (calibrationDataParameters) {
		try {
			calibrationDataParameters->readConfiguration();
		} catch (std::exception const &e) {
			LOG4CXX_ERROR(logger,"Driver " << driverName
					<< ": Error reading calibration data from file " << calibrationDataFileName
					<< ": " << e.what());
			// The file does not exist, or it has unexpected/undefined content.
			// Therefore I am initializing the calibration parameters fresh.
			delete calibrationDataParameters;
			calibrationDataParameters = new Properties4CXX::Properties(calibrationDataFileName);

		}
	}

	readOrCreateConfigValue(calibrationDataParameters,"magXFactor",calibrationData.magXFactor);
	readOrCreateConfigValue(calibrationDataParameters,"magYFactor",calibrationData.magYFactor);
	readOrCreateConfigValue(calibrationDataParameters,"magZFactor",calibrationData.magZFactor);
	readOrCreateConfigValue(calibrationDataParameters,"magXBias",calibrationData.magXBias);
	readOrCreateConfigValue(calibrationDataParameters,"magYBias",calibrationData.magYBias);
	readOrCreateConfigValue(calibrationDataParameters,"magZBias",calibrationData.magZBias);
	readOrCreateConfigValue(calibrationDataParameters,"magXVariance",calibrationData.magXVariance);
	readOrCreateConfigValue(calibrationDataParameters,"magYVariance",calibrationData.magYVariance);
	readOrCreateConfigValue(calibrationDataParameters,"magZVariance",calibrationData.magZVariance);

	readOrCreateConfigValue(calibrationDataParameters,"gyrXFactor",calibrationData.gyrXFactor);
	readOrCreateConfigValue(calibrationDataParameters,"gyrYFactor",calibrationData.gyrYFactor);
	readOrCreateConfigValue(calibrationDataParameters,"gyrZFactor",calibrationData.gyrZFactor);
	readOrCreateConfigValue(calibrationDataParameters,"gyrXBias",calibrationData.gyrXBias);
	readOrCreateConfigValue(calibrationDataParameters,"gyrYBias",calibrationData.gyrYBias);
	readOrCreateConfigValue(calibrationDataParameters,"gyrZBias",calibrationData.gyrZBias);
	readOrCreateConfigValue(calibrationDataParameters,"gyrXVariance",calibrationData.gyrXVariance);
	readOrCreateConfigValue(calibrationDataParameters,"gyrYVariance",calibrationData.gyrYVariance);
	readOrCreateConfigValue(calibrationDataParameters,"gyrZVariance",calibrationData.gyrZVariance);

	readOrCreateConfigValue(calibrationDataParameters,"accelXBias",calibrationData.accelXBias);
	readOrCreateConfigValue(calibrationDataParameters,"accelYBias",calibrationData.accelYBias);
	readOrCreateConfigValue(calibrationDataParameters,"accelZBias",calibrationData.accelZBias);
	readOrCreateConfigValue(calibrationDataParameters,"accelXFactor",calibrationData.accelXFactor);
	readOrCreateConfigValue(calibrationDataParameters,"accelYFactor",calibrationData.accelYFactor);
	readOrCreateConfigValue(calibrationDataParameters,"accelZFactor",calibrationData.accelZFactor);
	readOrCreateConfigValue(calibrationDataParameters,"accelXVariance",calibrationData.accelXVariance);
	readOrCreateConfigValue(calibrationDataParameters,"accelYVariance",calibrationData.accelYVariance);
	readOrCreateConfigValue(calibrationDataParameters,"accelZVariance",calibrationData.accelZVariance);

	readOrCreateConfigValue(calibrationDataParameters,"gravityValue",calibrationData.gravity);
	readOrCreateConfigValue(calibrationDataParameters,"gravityVariance",calibrationData.gravityVariance);

}

void BMXSensorBoardDriver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_DEBUG(logger,"Driver" << driverName << " read configuraion");


	try {
		auto portNameConfig = configuration.searchProperty("portName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"PortName\" is a struct or a string list.");
		}

		portName = portNameConfig->getStringValue();

		ioPort = dynamic_cast<io::DatagramPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not a stream port.");
		}
	} catch (std::exception const& e) {
		LOG4CXX_ERROR(logger, "Read configuration of driver \"" << driverName
				<< "\" failed:"
				<< e.what());
		throw;
	}

    errorTimeout = configuration.getPropertyValue(std::string("errorTimeout"),(long long)(10));
    errorMaxNumRetries = configuration.getPropertyValue(std::string("errorMaxNumRetries"),(long long)(0));

    try {
    	auto fileNameProp = configuration.searchProperty("calibrationDataFile");

		if (!fileNameProp->isList() && !fileNameProp->isStruct()) {
	    	calibrationDataFileName = fileNameProp->getStringValue();
	    	calibrationDataParameters = new Properties4CXX::Properties(calibrationDataFileName);
		}

		calibrationDataUpdateCycle = std::chrono::seconds(configuration.getPropertyValue("calibrationDataUpdateCycle",(long long)(0)));

    } catch (...) {
    	LOG4CXX_INFO(logger,"Driver" << driverName << ": No calibration data file specified");
    }


}

#define SQUARE(x) ((x)*(x))

void BMXSensorBoardDriver::initializeStatusAccel(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain,
		struct SensorData const &sumSensorData,
		int numAccelData
		) {

	// Assume that you are on the ground, but maybe tilted to the side
	// (remember this instrument is primarily for gliders)
	// and slightly pitched up in launch position compared to level flight
	auto avgAccelX = sumSensorData.accelX / float(numAccelData);
	auto avgAccelY = sumSensorData.accelY / float(numAccelData);
	auto avgAccelZ = sumSensorData.accelZ / float(numAccelData);
	auto absoluteAccel = sqrtf(avgAccelX*avgAccelX + avgAccelY*avgAccelY + avgAccelZ*avgAccelZ);

	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__ << " baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgAccelX = " << avgAccelX);
	LOG4CXX_DEBUG(logger,"avgAccelY = " << avgAccelY);
	LOG4CXX_DEBUG(logger,"avgAccelZ = " << avgAccelZ);
	LOG4CXX_DEBUG(logger,"absoluteAccel = " << absoluteAccel);

	// Try to assess pitch and roll angle from the accelerometer.
	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_PITCH,varioStatus.STATUS_IND_PITCH) == 0.0f) {
		varioStatus.pitchAngle = FastMath::fastASin(avgAccelX/absoluteAccel);
		LOG4CXX_DEBUG(logger,"Initial pitchAngle = " << varioStatus.pitchAngle);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_PITCH,varioStatus.STATUS_IND_PITCH) = 3.0f * 3.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_PITCH,varioStatus.STATUS_IND_PITCH) =
				SQUARE(2.0) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ROLL,varioStatus.STATUS_IND_ROLL) == 0.0f) {
		varioStatus.rollAngle = -FastMath::fastASin(avgAccelY/absoluteAccel);
		LOG4CXX_DEBUG(logger,"Initial rollAngle = " << varioStatus.rollAngle);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROLL,varioStatus.STATUS_IND_ROLL) =
				SQUARE(3.0f/ FastMath::fastCos(varioStatus.pitchAngle));
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ROLL,varioStatus.STATUS_IND_ROLL) =
				SQUARE(2.0) * baseIntervalSec;
	}

	// Gravity and accelerometers are siamese twins. So I handle gravity here too.

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_GRAVITY,varioStatus.STATUS_IND_GRAVITY) == 0.0f) {
		varioStatus.gravity = calibrationData.gravity;
		LOG4CXX_DEBUG(logger,"Initial gravity = " << varioStatus.gravity);
		/* Multiply the variance by 2.0 for two reasons:
		 * 1. The stored values may have shifted in the meantime
		 * 2. Before updating the calibration data file the variance must have improved sufficiently to be updated.
		 */
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GRAVITY,varioStatus.STATUS_IND_GRAVITY) = calibrationData.gravityVariance * 2.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_GRAVITY,varioStatus.STATUS_IND_GRAVITY) =
				SQUARE(0.05) * baseIntervalSec;
	}


	// With the fast-cycle accelerometer the accuracy of position and speed increase but I can allow for a much higher variance of the acceleration itself
	// But the accuracy of the position should vastly improve by the dead-reckoning
	if (varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_LATITUDE_OFFS,varioStatus.STATUS_IND_LATITUDE_OFFS) == 0.0) {
		// Lüneburg airport EDHG
		varioStatus.latitude(53.2483333333);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_LATITUDE_OFFS,varioStatus.STATUS_IND_LATITUDE_OFFS) = 10000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_LATITUDE_OFFS,varioStatus.STATUS_IND_LATITUDE_OFFS) =
				SQUARE(3.0) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_LONGITUDE_OFFS,varioStatus.STATUS_IND_LONGITUDE_OFFS) == 0.0f) {
		// Lüneburg airport EDHG
		varioStatus.longitude(10.4586111111);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_LONGITUDE_OFFS,varioStatus.STATUS_IND_LONGITUDE_OFFS) = 10000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_LONGITUDE_OFFS,varioStatus.STATUS_IND_LONGITUDE_OFFS) =
				SQUARE(3.0) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) == 0.0f) {
		// Lüneburg airport EDHG
		varioStatus.altMSL = 49.0f;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) = 1000.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ALT_MSL,varioStatus.STATUS_IND_ALT_MSL) =
				SQUARE(4.0) * baseIntervalSec;
	}


	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_SPEED_GROUND_N,varioStatus.STATUS_IND_SPEED_GROUND_N) == 0.0f) {
		varioStatus.groundSpeedNorth = 0.0f;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_SPEED_GROUND_N,varioStatus.STATUS_IND_SPEED_GROUND_N) = 100.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_SPEED_GROUND_N,varioStatus.STATUS_IND_SPEED_GROUND_N) =
				SQUARE(2.0) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_SPEED_GROUND_E,varioStatus.STATUS_IND_SPEED_GROUND_E) == 0.0f) {
		varioStatus.groundSpeedEast = 0.0f;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_SPEED_GROUND_E,varioStatus.STATUS_IND_SPEED_GROUND_E) = 100.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_SPEED_GROUND_E,varioStatus.STATUS_IND_SPEED_GROUND_E) =
				SQUARE(2.0) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ACC_HEADING,varioStatus.STATUS_IND_ACC_HEADING) == 0.0f) {
		varioStatus.accelHeading = 0.0f;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ACC_HEADING,varioStatus.STATUS_IND_ACC_HEADING) = 4.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ACC_HEADING,varioStatus.STATUS_IND_ACC_HEADING) =
				SQUARE(10.0) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ACC_CROSS,varioStatus.STATUS_IND_ACC_CROSS) == 0.0f) {
		varioStatus.accelCross = 0.0f;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ACC_CROSS,varioStatus.STATUS_IND_ACC_CROSS) = 1.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ACC_CROSS,varioStatus.STATUS_IND_ACC_CROSS) =
				SQUARE(10.0) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ACC_VERTICAL,varioStatus.STATUS_IND_ACC_VERTICAL) == 0.0f) {
		varioStatus.accelVertical = 0.0f;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ACC_VERTICAL,varioStatus.STATUS_IND_ACC_VERTICAL) = 4.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ACC_VERTICAL,varioStatus.STATUS_IND_ACC_VERTICAL) =
				SQUARE(20.0) * baseIntervalSec;
	}


}

void BMXSensorBoardDriver::initializeStatusGyro(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain,
		struct SensorData const &sumSensorData,
		int numGyroData
		) {
	// Assume that you are on the ground, but maybe tilted to the side
	// (remember this instrument is primarily for gliders)
	// and slightly pitched up
	auto avgGyroX = sumSensorData.gyroX / float(numGyroData);
	auto avgGyroY = sumSensorData.gyroY / float(numGyroData);
	auto avgGyroZ = sumSensorData.gyroZ / float(numGyroData);

	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__ << " baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgGyroX = " << avgGyroX);
	LOG4CXX_DEBUG(logger,"avgGyroY = " << avgGyroY);
	LOG4CXX_DEBUG(logger,"avgGyroZ = " << avgGyroZ);

	// Assume the plane is static, and the measurement is the current gyro bias.

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_GYRO_BIAS_X,varioStatus.STATUS_IND_GYRO_BIAS_X) == 0.0f) {
		varioStatus.gyroBiasX = avgGyroX;
		LOG4CXX_DEBUG(logger,"Initial gyroBiasX = " << varioStatus.gyroBiasX);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_X,varioStatus.STATUS_IND_GYRO_BIAS_X) =
				1.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_X,varioStatus.STATUS_IND_GYRO_BIAS_X) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ROTATION_X,varioStatus.STATUS_IND_ROTATION_X) == 0.0f) {
		varioStatus.rollRateX = 0;
		LOG4CXX_DEBUG(logger,"Initial rollRateX = " << varioStatus.rollRateX);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROTATION_X,varioStatus.STATUS_IND_ROTATION_X) =
				1.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ROTATION_X,varioStatus.STATUS_IND_ROTATION_X) =
				SQUARE(10) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_GYRO_BIAS_Y,varioStatus.STATUS_IND_GYRO_BIAS_Y) == 0.0f) {
		varioStatus.gyroBiasY = avgGyroY;
		LOG4CXX_DEBUG(logger,"Initial gyroBiasY = " << varioStatus.gyroBiasY);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Y,varioStatus.STATUS_IND_GYRO_BIAS_Y) =
				1.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Y,varioStatus.STATUS_IND_GYRO_BIAS_Y) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ROTATION_Y,varioStatus.STATUS_IND_ROTATION_Y) == 0.0f) {
		varioStatus.pitchRateY = 0;
		LOG4CXX_DEBUG(logger,"Initial pitchRateY = " << varioStatus.pitchRateY);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROTATION_Y,varioStatus.STATUS_IND_ROTATION_Y) =
				4.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ROTATION_Y,varioStatus.STATUS_IND_ROTATION_Y) =
				SQUARE(10) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_GYRO_BIAS_Z,varioStatus.STATUS_IND_GYRO_BIAS_Z) == 0.0f) {
		varioStatus.gyroBiasZ = avgGyroZ;
		LOG4CXX_DEBUG(logger,"Initial gyroBiasZ = " << varioStatus.gyroBiasZ);
		/* Multiply the variance by 2.0 for two reasons:
		 * 1. The stored values may have shifted in the meantime
		 * 2. Before updating the calibration data file the variance must have improved sufficiently to be updated.
		 */
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Z,varioStatus.STATUS_IND_GYRO_BIAS_Z) =
				0.5f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Z,varioStatus.STATUS_IND_GYRO_BIAS_Z) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ROTATION_Z,varioStatus.STATUS_IND_ROTATION_Z) == 0.0f) {
		varioStatus.yawRateZ = 0;
		LOG4CXX_DEBUG(logger,"Initial yawRateZ = " << varioStatus.yawRateZ);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROTATION_Z,varioStatus.STATUS_IND_ROTATION_Z) =
				4.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ROTATION_Z,varioStatus.STATUS_IND_ROTATION_Z) =
				SQUARE(10) * baseIntervalSec;
	}

}

void BMXSensorBoardDriver::initializeStatusMag(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain,
		struct SensorData const &sumSensorData,
		int numMagData
		) {

	/*
	 *
	 * Assume that you are on the ground, but maybe tilted to the side
	 * (remember this instrument is primarily for gliders)
	 * and slightly pitched up
	 *
	 * Another precondition is that initializeStatusAccel() has been called before so that roll and pitch angle are
	 * already determined, and I can calculate the yaw (i.e. the direction)
	 */
	auto avgMagX = sumSensorData.magX / float(numMagData) - calibrationData.magXBias;
	auto avgMagY = sumSensorData.magY / float(numMagData) - calibrationData.magYBias;
	auto avgMagZ = sumSensorData.magZ / float(numMagData) - calibrationData.magZBias ;

	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__ << "baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgMagX = " << avgMagX);
	LOG4CXX_DEBUG(logger,"avgMagY = " << avgMagY);
	LOG4CXX_DEBUG(logger,"avgMagZ = " << avgMagZ);

	// If the pitch angle is nearly perpendicular to the flat plane the roll angle cannot be determined with any accuracy
	// Albeit a more than unlikely scenario :D
	if (fabsf(varioStatus.pitchAngle) < 80) {
		RotationMatrix rotMatrix (0.0f,varioStatus.pitchAngle,varioStatus.rollAngle);
		Vector3DType planeMagVector (avgMagX,avgMagY,avgMagZ);
		Vector3DType worldMagVector;

		// I already determined pitch and roll angle. With these now I can move the plane coordinate system into
		// the wold coordinate system, and can determine the heading (yaw angle).
		rotMatrix.calcPlaneVectorToWorldVector(planeMagVector,worldMagVector);

		LOG4CXX_DEBUG(logger,"worldMagX = " << worldMagVector[0]);
		LOG4CXX_DEBUG(logger,"worldMagY = " << worldMagVector[1]);
		LOG4CXX_DEBUG(logger,"worldMagZ = " << worldMagVector[2]);

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) == 0.0f) {

			varioStatus.heading = FastMath::fastATan2(-worldMagVector[1],worldMagVector[0]);
			LOG4CXX_DEBUG(logger,"Initial heading = " << varioStatus.heading);
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) = 5.0f * 5.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) =
					SQUARE(5.0) * baseIntervalSec;
		}

		if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_MAGNETIC_INCLINATION,varioStatus.STATUS_IND_MAGNETIC_INCLINATION) == 0.0f) {
			varioStatus.magneticInclination = FastMath::fastATan2(
					-avgMagZ,sqrtf(avgMagX*avgMagX + avgMagY*avgMagY));
			if (varioStatus.magneticInclination > 90.0f) {
				varioStatus.magneticInclination -= 360.0f;
			}
			LOG4CXX_DEBUG(logger,"Initial magnetic inclination = " << varioStatus.magneticInclination);
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_MAGNETIC_INCLINATION,varioStatus.STATUS_IND_MAGNETIC_INCLINATION) = 5.0f * 5.0f;
			varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_MAGNETIC_INCLINATION,varioStatus.STATUS_IND_MAGNETIC_INCLINATION) =
					SQUARE(0.1) * baseIntervalSec;
		}
	}

	// Set the magnetometer bias unconditionally
	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_COMPASS_DEVIATION_X,varioStatus.STATUS_IND_COMPASS_DEVIATION_X) == 0.0f) {
		varioStatus.compassDeviationX = calibrationData.magXBias;
		LOG4CXX_DEBUG(logger,"Initial compassDeviationX = " << varioStatus.compassDeviationX);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_X,varioStatus.STATUS_IND_COMPASS_DEVIATION_X) = calibrationData.magXVariance * 4.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_X,varioStatus.STATUS_IND_COMPASS_DEVIATION_X) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_COMPASS_DEVIATION_Y,varioStatus.STATUS_IND_COMPASS_DEVIATION_Y) == 0.0f) {
		varioStatus.compassDeviationY = calibrationData.magYBias;
		LOG4CXX_DEBUG(logger,"Initial compassDeviationY = " << varioStatus.compassDeviationY);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_Y,varioStatus.STATUS_IND_COMPASS_DEVIATION_Y) = calibrationData.magYVariance * 4.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_Y,varioStatus.STATUS_IND_COMPASS_DEVIATION_Y) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_COMPASS_DEVIATION_Z,varioStatus.STATUS_IND_COMPASS_DEVIATION_Z) == 0.0f) {
		varioStatus.compassDeviationZ = calibrationData.magZBias;
		LOG4CXX_DEBUG(logger,"Initial compassDeviationZ = " << varioStatus.compassDeviationZ);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_Z,varioStatus.STATUS_IND_COMPASS_DEVIATION_Z) = calibrationData.magZVariance * 4.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_Z,varioStatus.STATUS_IND_COMPASS_DEVIATION_Z) =
				SQUARE(0.1) * baseIntervalSec;
	}


}

#undef SQUARE

void BMXSensorBoardDriver::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain) {

	// Try 20 times, and wait for a second if no data yet available
	for (int i = 0; i < 20;i++) {
		using namespace std::chrono_literals;

		struct SensorData avgSensorData {
			false,0.0f,0.0f,0.0f,
			false,0.0f,0.0f,0.0f,
			false,0.0f,0.0f,0.0f
		};
		int numAccel = 0;
		int numGyro  = 0;
		int numMag   = 0;

		// Add up the data in the ring buffer
		for (int k = 0; k < SIZE_SENSOR_DATA_ARRAY; k++) {
			struct SensorData &sensorData = sensorDataArr[k];

			if (sensorData.accelDataValid) {
				avgSensorData.accelX += sensorData.accelX;
				avgSensorData.accelY += sensorData.accelY;
				avgSensorData.accelZ += sensorData.accelZ;
				numAccel ++;
			}

			if (sensorData.gyroDataValid) {
				avgSensorData.gyroX += sensorData.gyroX;
				avgSensorData.gyroY += sensorData.gyroY;
				avgSensorData.gyroZ += sensorData.gyroZ;
				numGyro ++;
			}

			if (sensorData.magDataValid) {
				avgSensorData.magX += sensorData.magX;
				avgSensorData.magY += sensorData.magY;
				avgSensorData.magZ += sensorData.magZ;
				numMag ++;
			}
		}

		if (numAccel >= 10 && numGyro >= 10 && numMag >= 10) {
			initializeStatusAccel(varioStatus,varioMain,avgSensorData,numAccel);
			initializeStatusGyro(varioStatus,varioMain,avgSensorData,numGyro);
			initializeStatusMag(varioStatus,varioMain,avgSensorData,numMag);

			statusInitDone = true;

			break;
		}

		std::this_thread::sleep_for(1s);

	} // for (int i = 0; i < 20;i++)

	lastUpdateTime = std::chrono::system_clock::now();

}

void BMXSensorBoardDriver::updateKalmanStatus (GliderVarioStatus &varioStatus) {



}


void BMXSensorBoardDriver::driverThreadFunction() {

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

static constexpr float BMM150_OVERFLOW_OUTPUT_FLOAT = 1.0e10f;

void BMXSensorBoardDriver::processingMainLoop () {

	/** \brief Factor to divide the raw accelerometer values by to get the actual g value.
	 *
	 *  Range on the BMX 160 is set from -4g - +4g. This range is 0x8000 in raw values
	 *
	 */
	static constexpr double accFactor = 4.0 / double(0x8000);

	/** \brief Factor to divide the raw gyroscope values by to get the actual deg/s value.
	 *
	 *  Range on the BMX 160 is set from -250deg&s - +250deg/s. This range is 0x8000 in raw values, but as signed int.
	 *
	 */
	static constexpr double gyrFactor = double(0x8000) / 250.0;
	struct BMX160Data bmxData;


	while (!getStopDriverThread()) {
		auto readLen = ioPort->recv((uint8_t *)(&bmxData),sizeof(bmxData));
		LOG4CXX_DEBUG(logger,"Read " << readLen << "Bytes from the port. Expected " << sizeof(bmxData) << " bytes.");

		struct SensorData &currSensorData = sensorDataArr[currSensorDataIndex];
		currSensorData.accelDataValid = false;
		currSensorData.gyroDataValid = false;
		currSensorData.magDataValid = false;

		LOG4CXX_DEBUG(logger,"bmxData.header.unionCode    = " << int(bmxData.header.unionCode));
		LOG4CXX_DEBUG(logger,"bmxData.header.length       = " << bmxData.header.length);
		LOG4CXX_DEBUG(logger,"bmxData.header.versionMajor = " << int(bmxData.header.versionMajor));
		LOG4CXX_DEBUG(logger,"bmxData.header.versionMinor = " << int(bmxData.header.versionMinor));
		LOG4CXX_DEBUG(logger,"bmxData.header.crc          = " << std::hex << uint16_t(bmxData.header.crc) << std::dec);

		if (bmxData.header.versionMajor == BMX160_SENSORBOX_MSG_VERSION_MAJOR &&
				bmxData.header.versionMinor == BMX160_SENSORBOX_MSG_VERSION_MINOR)
		{
			switch (bmxData.header.unionCode) {
			case BMX160DATA_TRIM:
				if (bmxData.header.length == (sizeof(bmxData.header)+sizeof(bmxData.trimData))) {
					uint16_t msgCrc;
					msgCrc = bmxData.header.crc;
					bmxData.header.crc = 0U;

					if (msgCrc == 0xffff || msgCrc == crc16CCIT(PPP_INITFCS,&bmxData,bmxData.header.length)) {
						magTrimData = bmxData.trimData;
						LOG4CXX_DEBUG(logger, "TrimData = \n"
								<< "\tdig_x1   = " << int16_t(magTrimData.dig_x1) << "\n"
								<< "\tdig_y1   = " << int16_t(magTrimData.dig_y1) << "\n"
								<< "\tdig_z1   = " << magTrimData.dig_z1 << "\n"
								<< "\tdig_x2   = " << int16_t(magTrimData.dig_x2) << "\n"
								<< "\tdig_y2   = " << int16_t(magTrimData.dig_y2) << "\n"
								<< "\tdig_z2   = " << magTrimData.dig_z2 << "\n"
								<< "\tdig_z3   = " << magTrimData.dig_z3 << "\n"
								<< "\tdig_z4   = " << magTrimData.dig_z4 << "\n"
								<< "\tdig_xy1  = " << uint16_t(magTrimData.dig_xy1) << "\n"
								<< "\tdig_xy2  = " << int16_t(magTrimData.dig_xy2) << "\n"
								<< "\tdig_xyz1 = " << magTrimData.dig_xyz1);

						// Essential trim data is missing.
						// compensate function would return BMM150_OVERFLOW_OUTPUT_FLOAT = 0.0.
						// Therefore request a reset of the IMU, re-read the magnetometer trim data,
						// and re-send them to me.
						if ((magTrimData.dig_z2 == 0) || (magTrimData.dig_z1 == 0)
								|| (magTrimData.dig_xyz1 == 0) ) {
							struct BMX160RecvData sendMsg;

							sendMsg.header.unionCode = BMX160RECV_DATA_RESET_IMU;
							sendMsg.header.filler = 0;
							sendMsg.header.length = sizeof sendMsg;
							sendMsg.header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR;
							sendMsg.header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR;
							sendMsg.dummy = 0;

							sendMsg.header.crc = 0U;
							sendMsg.header.crc = 0xffff; // crc16CCIT(PPP_INITFCS,&sendMsg,sendMsg.header.length);

							ioPort->send((uint8_t*)(&sendMsg),sizeof(sendMsg));

						}
					} else {
						LOG4CXX_ERROR(logger,
								"CRC error of received magnetometer trim data message");
						// Send a request to send the trim data again
						struct BMX160RecvData sendMsg;

						sendMsg.header.unionCode = BMX160RECV_DATA_RESET_IMU;
						sendMsg.header.filler = 0;
						sendMsg.header.length = sizeof sendMsg;
						sendMsg.header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR;
						sendMsg.header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR;
						sendMsg.dummy = 0;

						sendMsg.header.crc = 0U;
						sendMsg.header.crc = crc16CCIT(PPP_INITFCS,&sendMsg,sendMsg.header.length);

						ioPort->send((uint8_t*)(&sendMsg),sizeof(sendMsg));
					}
				} else {
					LOG4CXX_ERROR(logger,
							"Alignment error. bmxData.header.length = " << bmxData.header.length
							<< " but sizeof(bmxData.header)+sizeof(bmxData.trimData) = " << (sizeof(bmxData.header)+sizeof(bmxData.trimData)));
				}
				break;
			case BMX160DATA_ACC_GYR_MAG:
				if (bmxData.header.length == (sizeof(bmxData.header)+sizeof(bmxData.accGyrMagData))) {
					uint16_t msgCrc;
					msgCrc = bmxData.header.crc;
					bmxData.header.crc = 0U;

					if (msgCrc == 0xffff || msgCrc == crc16CCIT(PPP_INITFCS,&bmxData,bmxData.header.length)) {

						// advance the index, and wrap it around if necessary
						currSensorDataIndex++;
						currSensorDataIndex &= SIZE_SENSOR_DATA_ARRAY - 1;

						// Essential trim data is missing.
						// compensate function would return BMM150_OVERFLOW_OUTPUT_FLOAT = 0.0.
						// Therefore request a reset of the IMU, re-read the magnetometer trim data,
						// and re-send them to me.
						if ((magTrimData.dig_z2 == 0) || (magTrimData.dig_z1 == 0)
								|| (magTrimData.dig_xyz1 == 0) ) {
							struct BMX160RecvData sendMsg;

							sendMsg.header.unionCode = BMX160RECV_DATA_RESET_IMU;
							sendMsg.header.filler = 0;
							sendMsg.header.length = sizeof sendMsg;
							sendMsg.header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR;
							sendMsg.header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR;
							sendMsg.dummy = 0;

							sendMsg.header.crc = 0U;
							sendMsg.header.crc = 0xffff;

							ioPort->send((uint8_t*)(&sendMsg),sizeof(sendMsg));

						} else {

							currSensorData.magX = compensate_x(bmxData.accGyrMagData.magX,bmxData.accGyrMagData.magRHall) * calibrationData.magXFactor;
							currSensorData.magY = compensate_y(bmxData.accGyrMagData.magY,bmxData.accGyrMagData.magRHall) * calibrationData.magYFactor;
							currSensorData.magZ = compensate_z(bmxData.accGyrMagData.magZ,bmxData.accGyrMagData.magRHall) * calibrationData.magZFactor;

							LOG4CXX_DEBUG(logger,"magX (uT) = " << currSensorData.magX);
							LOG4CXX_DEBUG(logger,"magY (uT) = " << currSensorData.magY);
							LOG4CXX_DEBUG(logger,"magZ (uT) = " << currSensorData.magZ);
							if (currSensorData.magX != BMM150_OVERFLOW_OUTPUT_FLOAT &&
									currSensorData.magY != -BMM150_OVERFLOW_OUTPUT_FLOAT &&
									currSensorData.magZ != -BMM150_OVERFLOW_OUTPUT_FLOAT
									) {
								currSensorData.magDataValid = true;
							}
						}

						currSensorData.gyroX = double(bmxData.accGyrMagData.gyrX)/ gyrFactor * calibrationData.gyrXFactor;
						currSensorData.gyroY = double(bmxData.accGyrMagData.gyrY)/ gyrFactor * calibrationData.gyrYFactor;
						currSensorData.gyroZ = double(bmxData.accGyrMagData.gyrZ)/ gyrFactor * calibrationData.gyrZFactor;
						currSensorData.gyroDataValid = true;

						LOG4CXX_DEBUG(logger,"gyrX (deg/s) = " << currSensorData.gyroX);
						LOG4CXX_DEBUG(logger,"gyrY (deg/s) = " << currSensorData.gyroY);
						LOG4CXX_DEBUG(logger,"gyrZ (deg/s) = " << currSensorData.gyroZ);


						currSensorData.accelX = (double)(bmxData.accGyrMagData.accX) * accFactor *
								calibrationData.accelXFactor - calibrationData.accelXBias;
						currSensorData.accelY = (double)(bmxData.accGyrMagData.accY) * accFactor *
								calibrationData.accelYFactor - calibrationData.accelYBias;
						currSensorData.accelZ = (double)(bmxData.accGyrMagData.accZ) * accFactor *
								calibrationData.accelZFactor - calibrationData.accelZBias;
						currSensorData.accelDataValid = true;

						LOG4CXX_DEBUG(logger,"accX (g) = " << currSensorData.accelX);
						LOG4CXX_DEBUG(logger,"accY (g) = " << currSensorData.accelY);
						LOG4CXX_DEBUG(logger,"accZ (g) = " << currSensorData.accelZ);
					} else { // if (crc == msgCrc)
						LOG4CXX_ERROR(logger,
								"CRC error of received acc/gyr/mag sensor data message");
					} // if (crc == msgCrc)
				} else { // if (bmxData.header.length == ...
					LOG4CXX_ERROR(logger,
							"Alignment error. bmxData.header.length = " << bmxData.header.length
							<< " but (sizeof(bmxData.header)+sizeof(bmxData.accGyrMagData)) = " << (sizeof(bmxData.header)+sizeof(bmxData.accGyrMagData)));
				} // if (bmxData.header.length == ...
				break;
			case BMX160DATA_ACC_GYR:
				if (bmxData.header.length == (sizeof(bmxData.header)+sizeof(bmxData.accGyrData))) {
					uint16_t msgCrc;
					msgCrc = bmxData.header.crc;
					bmxData.header.crc = 0U;

					if (msgCrc == 0xffff || msgCrc == crc16CCIT(PPP_INITFCS,&bmxData,bmxData.header.length)) {
						// advance the index, and wrap it around if necessary
						currSensorDataIndex++;
						currSensorDataIndex &= SIZE_SENSOR_DATA_ARRAY - 1;

						currSensorData.gyroX = double(bmxData.accGyrData.gyrX)/ gyrFactor;
						currSensorData.gyroY = double(bmxData.accGyrData.gyrY)/ gyrFactor;
						currSensorData.gyroZ = double(bmxData.accGyrData.gyrZ)/ gyrFactor;
						currSensorData.gyroDataValid = true;

						LOG4CXX_DEBUG(logger,"gyrX (deg/s) = " << currSensorData.gyroX);
						LOG4CXX_DEBUG(logger,"gyrY (deg/s) = " << currSensorData.gyroY);
						LOG4CXX_DEBUG(logger,"gyrZ (deg/s) = " << currSensorData.gyroZ);

						currSensorData.accelX = (double)(bmxData.accGyrMagData.accX) * accFactor *
								calibrationData.accelXFactor - calibrationData.accelXBias;
						currSensorData.accelY = (double)(bmxData.accGyrMagData.accY) * accFactor *
								calibrationData.accelYFactor - calibrationData.accelYBias;
						currSensorData.accelZ = (double)(bmxData.accGyrMagData.accZ) * accFactor *
								calibrationData.accelZFactor - calibrationData.accelZBias;
						currSensorData.accelDataValid = true;

						LOG4CXX_DEBUG(logger,"accX (g) = " << currSensorData.accelX);
						LOG4CXX_DEBUG(logger,"accY (g) = " << currSensorData.accelY);
						LOG4CXX_DEBUG(logger,"accZ (g) = " << currSensorData.accelZ);
					} else { // if (crc == msgCrc)
						LOG4CXX_ERROR(logger,
								"CRC error of received acc/gyr sensor data message");
					} // if (crc == msgCrc) {
				} else { // if (bmxData.header.length == ...
					LOG4CXX_ERROR(logger,
							"Alignment error. bmxData.header.length = " << bmxData.header.length
							<< " but (sizeof(bmxData.header)+sizeof(bmxData.accGyrData)) = " << (sizeof(bmxData.header)+sizeof(bmxData.accGyrData)));
				} // if (bmxData.header.length == ...
				break;
			default:
				LOG4CXX_ERROR (logger,"Unknown union code " << (int)bmxData.header.unionCode);

				break;
			} // switch (bmxData.header.unionCode)
		} // if (bmxData.header.versionMajor == ...

		if (getIsKalmanUpdateRunning()) {
			GliderVarioMainPriv::LockedCurrentStatus currStatus(*varioMain);

			if (currSensorData.accelDataValid) {
				GliderVarioMeasurementUpdater::accelUpd(
						currSensorData.accelX,0.1f,
						currSensorData.accelY,0.1f,
						currSensorData.accelZ,0.1f,
						*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
			}
			if (currSensorData.gyroDataValid) {
				GliderVarioMeasurementUpdater::gyroUpd(
						currSensorData.gyroX,0.1f,
						currSensorData.gyroY,0.1f,
						currSensorData.gyroZ,0.1f,
						*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
			}
			if (currSensorData.magDataValid) {
				GliderVarioMeasurementUpdater::compassUpd(
						currSensorData.magX,currSensorData.magY,currSensorData.magZ,
						2.0f,2.0f,2.0f,
						*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
			}

			auto lastPredictionUpdate = varioMain->getLastPredictionUpdate();
			auto timeSinceLastCalibrationWrite = lastPredictionUpdate - lastUpdateTime;
			if (!calibrationWriterRunning && (timeSinceLastCalibrationWrite >= calibrationDataUpdateCycle)) {
				calibrationWriterRunning = true;
				if (calibrationDataWriteThread.joinable()) {
					calibrationDataWriteThread.join();
				}
				lastUpdateTime = std::chrono::system_clock::now();
				calibrationDataWriteThread = std::thread(&BMXSensorBoardDriver::calibrationDataWriteFunc,this);
			}
		}
	}
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer x axis data(micro-tesla) in float.
 */
float BMXSensorBoardDriver::compensate_x(int16_t mag_data_x, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_x0;
	float process_comp_x1;
	float process_comp_x2;
	float process_comp_x3;
	float process_comp_x4;

	/* Overflow condition check */
	if ((mag_data_x != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) &&
		(data_rhall != 0) && (magTrimData.dig_xyz1 != 0)) {
			/*Processing compensation equations*/
			process_comp_x0 = (((float)magTrimData.dig_xyz1) * 16384.0f / data_rhall);
			retval = (process_comp_x0 - 16384.0f);
			process_comp_x1 = ((float)magTrimData.dig_xy2) * (retval * retval / 268435456.0f);
			process_comp_x2 = process_comp_x1 + retval * ((float)magTrimData.dig_xy1) / 16384.0f;
			process_comp_x3 = ((float)magTrimData.dig_x2) + 160.0f;
			process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
			retval = ((process_comp_x4 / 8192.0f) + (((float)magTrimData.dig_x1) * 8.0f)) / 16.0f;
	} else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer y axis data(micro-tesla) in float.
 */
float BMXSensorBoardDriver::compensate_y(int16_t mag_data_y, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_y0;
	float process_comp_y1;
	float process_comp_y2;
	float process_comp_y3;
	float process_comp_y4;

	/* Overflow condition check */
	if ((mag_data_y != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
		&& (data_rhall != 0) && (magTrimData.dig_xyz1 != 0)) {
			/*Processing compensation equations*/
			process_comp_y0 = ((float)magTrimData.dig_xyz1) * 16384.0f / data_rhall;
			retval = process_comp_y0 - 16384.0f;
			process_comp_y1 = ((float)magTrimData.dig_xy2) * (retval * retval / 268435456.0f);
			process_comp_y2 = process_comp_y1 + retval * ((float)magTrimData.dig_xy1) / 16384.0f;
			process_comp_y3 = ((float)magTrimData.dig_y2) + 160.0f;
			process_comp_y4 = mag_data_y * (((process_comp_y2) + 256.0f) * process_comp_y3);
			retval = ((process_comp_y4 / 8192.0f) + (((float)magTrimData.dig_y1) * 8.0f)) / 16.0f;
	} else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer z axis data(micro-tesla) in float.
 */
float BMXSensorBoardDriver::compensate_z(int16_t mag_data_z, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_z0;
	float process_comp_z1;
	float process_comp_z2;
	float process_comp_z3;
	float process_comp_z4;
	float process_comp_z5;

	 /* Overflow condition check */
	if ((mag_data_z != BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL) &&
		(magTrimData.dig_z2 != 0) && (magTrimData.dig_z1 != 0)
		&& (magTrimData.dig_xyz1 != 0) && (data_rhall != 0)) {
			/* Processing compensation equations */
			process_comp_z0 = ((float)mag_data_z) - ((float)magTrimData.dig_z4);
			process_comp_z1 = ((float)data_rhall) - ((float)magTrimData.dig_xyz1);
			process_comp_z2 = (((float)magTrimData.dig_z3) * process_comp_z1);
			process_comp_z3 = ((float)magTrimData.dig_z1) * ((float)data_rhall) / 32768.0f;
			process_comp_z4 = ((float)magTrimData.dig_z2) + process_comp_z3;
			process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
			retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;
	} else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
	}

	return retval;
}

void BMXSensorBoardDriver::calibrationDataWriteFunc() {


	if (!varioMain) {
		calibrationWriterRunning = false;
		return;
	}

	{
		// Lock the current status as briefly as possible.
		GliderVarioMainPriv::LockedCurrentStatus currentLockedStatus(*varioMain);
		GliderVarioStatus* currentStatus = currentLockedStatus.getCurrentStatus();
		GliderVarioStatus::StatusCoVarianceType &coVariance = currentStatus->getErrorCovariance_P();

		// If the estimated error is in a similar range or better than the current Variance update the estimated bias.
		// Allow for fluctuations of the variance otherwise necessary updates may never happen
		auto currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,
				GliderVarioStatus::STATUS_IND_GYRO_BIAS_X);
		if (currVariance <= calibrationData.gyrXVariance * 1.5f) {
			calibrationData.gyrXBias = currentStatus->gyroBiasX;
			calibrationData.gyrXVariance = currVariance;
			writeConfigValue(calibrationDataParameters,"gyrXBias",calibrationData.gyrXBias);
			writeConfigValue(calibrationDataParameters,"gyrXVariance",currVariance);
		}
		currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,
				GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y);
		if (currVariance <= calibrationData.gyrYVariance * 1.5f) {
			calibrationData.gyrYBias = currentStatus->gyroBiasY;
			calibrationData.gyrYVariance = currVariance;
			writeConfigValue(calibrationDataParameters,"gyrYBias",calibrationData.gyrYBias);
			writeConfigValue(calibrationDataParameters,"gyrYVariance",currVariance);
		}
		currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,
				GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z);
		if (currVariance <= calibrationData.gyrZVariance * 1.5f) {
			calibrationData.gyrZBias = currentStatus->gyroBiasZ;
			calibrationData.gyrZVariance = currVariance;
			writeConfigValue(calibrationDataParameters,"gyrZBias",calibrationData.gyrZBias);
			writeConfigValue(calibrationDataParameters,"gyrZVariance",currVariance);
		}

		currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X,
				GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X);
		if (currVariance <= calibrationData.magXVariance * 1.5f) {
			calibrationData.magXBias = currentStatus->compassDeviationX;
			calibrationData.magXVariance = currVariance;
			writeConfigValue(calibrationDataParameters,"magXBias",calibrationData.magXBias);
			writeConfigValue(calibrationDataParameters,"magXVariance",currVariance);
		}
		currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y,
				GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y);
		if (currVariance <= calibrationData.magYVariance * 1.5f) {
			calibrationData.magYBias = currentStatus->compassDeviationY;
			calibrationData.magYVariance = currVariance;
			writeConfigValue(calibrationDataParameters,"magYBias",calibrationData.magYBias);
			writeConfigValue(calibrationDataParameters,"magYVariance",currVariance);
		}
		currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z,
				GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z);
		if (currVariance <= calibrationData.magZVariance * 1.5f) {
			calibrationData.magZBias = currentStatus->compassDeviationZ;
			calibrationData.magZVariance = currVariance;
			writeConfigValue(calibrationDataParameters,"magZBias",calibrationData.magZBias);
			writeConfigValue(calibrationDataParameters,"magZVariance",currVariance);
		}

		currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_GRAVITY,
				GliderVarioStatus::STATUS_IND_GRAVITY);
		if (currVariance <= calibrationData.gravityVariance * 1.5f) {
			calibrationData.gravity = currentStatus->gravity;
			calibrationData.gravityVariance = currVariance;
			writeConfigValue(calibrationDataParameters,"gravityValue",calibrationData.gravity);
			writeConfigValue(calibrationDataParameters,"gravityVariance",currVariance);
		}
	}

	try {
		std::ofstream of(calibrationDataFileName,of.out | of.trunc);
		if (of.good()) {
			calibrationDataParameters->writeOut(of);
		}
	} catch (std::exception const &e) {
		LOG4CXX_ERROR(logger,"Error in " << __PRETTY_FUNCTION__
				<< ". Cannot write calibration data. Error = " << e.what());
	}
	catch (...) {}

	calibrationWriterRunning = false;

}

} // namespace openEV
