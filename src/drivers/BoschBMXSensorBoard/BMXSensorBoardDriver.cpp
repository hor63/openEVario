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
#include <thread>

#include "BMXSensorBoardDriver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"
#include "util/RotationMatrix.h"

#include "horOvIp-I2C-Bridge/BMX160net.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.BMXSensorBoard");
	}
}

#endif

namespace openEV {

/** CRC-16 CCIT taken from the PPP implementation of lwip
 *
 * \see http://git.savannah.nongnu.org/cgit/lwip.git/tree/src/netif/ppp/pppos.c line 97ff
 */
static const uint16_t fcstab[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
#define PPP_FCS(fcs, c) (((fcs) >> 8) ^ fcstab[((fcs) ^ (c)) & 0xff])
static inline uint16_t crcBlock(uint16_t crc, const void* const block,uint16_t len) {
	uint16_t rc = crc;
	uint16_t i;

	for (i=0; i < len; i++){
		rc = PPP_FCS(rc,((const char* const)block)[i]);
	}

	return rc;
}
/*
 * Values for FCS calculations.
 */
#define PPP_INITFCS     0xffffU  /* Initial FCS value */
#define PPP_GOODFCS     0xf0b8U  /* Good final FCS value */
// End of lwip section. Thank you.


BMXSensorBoardDriver::BMXSensorBoardDriver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: GliderVarioDriverBase {driverName,description,instanceName,BMXSensorBoardLib::theOneAndOnly}
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


}


void BMXSensorBoardDriver::driverInit() {

	/// todo fill me

}

void BMXSensorBoardDriver::readConfiguration (Properties4CXX::Properties const &configuration) {


	try {
		auto portNameConfig = configuration.searchProperty("PortName");

		if (portNameConfig->isList() || portNameConfig->isStruct()) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"Configuration variable \"PortName\" is a struct or a string list.");
		}

		portName = portNameConfig->getStringValue();

		ioPort = dynamic_cast<io::StreamPort*> (io::PortBase::getPortByName(portName));
		if (ioPort == nullptr) {
			throw GliderVarioFatalConfigException(__FILE__,__LINE__,"I/O Port is not a stream port.");
		}
	} catch (std::exception const& e) {
		LOG4CXX_ERROR(logger, "Read configuration of driver \"" << driverName
				<< "\" failed:"
				<< e.what());
	}

    errorTimeout = (long long)(configuration.getPropertyValue(std::string("errorTimeout"),(long long)(10)));
    errorMaxNumRetries = (long long)(configuration.getPropertyValue(std::string("errorTimeout"),(long long)(0)));


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
	// and slightly pitched up
	auto avgAccelX = sumSensorData.accelX / float(numAccelData);
	auto avgAccelY = sumSensorData.accelY / float(numAccelData);
	auto avgAccelZ = sumSensorData.accelZ / float(numAccelData);

	double baseIntervalSec = std::chrono::duration_cast<std::chrono::duration<double>>(varioMain.getProgramOptions().idlePredictionCycle).count() ;

	LOG4CXX_DEBUG(logger,"baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgAccelX = " << avgAccelX);
	LOG4CXX_DEBUG(logger,"avgAccelY = " << avgAccelY);
	LOG4CXX_DEBUG(logger,"avgAccelZ = " << avgAccelZ);

	// Try to assess pitch and roll angle from the accelerometer.
	// first the pitch angle:
	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_PITCH,varioStatus.STATUS_IND_PITCH) == 0.0f) {
		varioStatus.pitchAngle = FastMath::fastASin(avgAccelX);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_PITCH,varioStatus.STATUS_IND_PITCH) = 3.0f * 3.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_PITCH,varioStatus.STATUS_IND_PITCH) =
				SQUARE(4.0) * baseIntervalSec;
	}

	// If the plane is tilted nearly perpendicular up I cannot calculate the roll from the accelerometer measurement any more.
	if (fabsf(varioStatus.pitchAngle) < 80.0f &&
			varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_ROLL,varioStatus.STATUS_IND_ROLL) == 0.0f) {
		varioStatus.rollAngle = FastMath::fastASin(-avgAccelY / FastMath::fastCos(varioStatus.pitchAngle));
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROLL,varioStatus.STATUS_IND_ROLL) =
				(3.0f/ FastMath::fastCos(varioStatus.pitchAngle)) * (3.0f/ FastMath::fastCos(varioStatus.pitchAngle));
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_ROLL,varioStatus.STATUS_IND_ROLL) =
				SQUARE(4.0) * baseIntervalSec;
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

	double baseIntervalSec = std::chrono::duration_cast<std::chrono::duration<double>>(varioMain.getProgramOptions().idlePredictionCycle).count() ;

	LOG4CXX_DEBUG(logger,"baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgGyroX = " << avgGyroX);
	LOG4CXX_DEBUG(logger,"avgGyroY = " << avgGyroY);
	LOG4CXX_DEBUG(logger,"avgGyroZ = " << avgGyroZ);

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_GYRO_BIAS_X,varioStatus.STATUS_IND_GYRO_BIAS_X) == 0.0f) {
		varioStatus.gyroBiasX = avgGyroX;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_X,varioStatus.STATUS_IND_GYRO_BIAS_X) = 1.0f * 1.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_X,varioStatus.STATUS_IND_GYRO_BIAS_X) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_GYRO_BIAS_Y,varioStatus.STATUS_IND_GYRO_BIAS_Y) == 0.0f) {
		varioStatus.gyroBiasY = avgGyroY;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Y,varioStatus.STATUS_IND_GYRO_BIAS_Y) = 1.0f * 1.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Y,varioStatus.STATUS_IND_GYRO_BIAS_Y) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_GYRO_BIAS_Z,varioStatus.STATUS_IND_GYRO_BIAS_Z) == 0.0f) {
		varioStatus.gyroBiasZ = avgGyroZ;
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Z,varioStatus.STATUS_IND_GYRO_BIAS_Z) = 1.0f * 1.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Z,varioStatus.STATUS_IND_GYRO_BIAS_Z) =
				SQUARE(0.1) * baseIntervalSec;
	}


}

void BMXSensorBoardDriver::initializeStatusMag(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain,
		struct SensorData const &sumSensorData,
		int numMagData
		) {

	// Assume that you are on the ground, but maybe tilted to the side
	// (remember this instrument is primarily for gliders)
	// and slightly pitched up
	auto avgMagX = sumSensorData.magX / float(numMagData);
	auto avgMagY = sumSensorData.magY / float(numMagData);
	auto avgMagZ = sumSensorData.magZ / float(numMagData);

	double baseIntervalSec = std::chrono::duration_cast<std::chrono::duration<double>>(varioMain.getProgramOptions().idlePredictionCycle).count() ;

	LOG4CXX_DEBUG(logger,"baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgMagX = " << avgMagX);
	LOG4CXX_DEBUG(logger,"avgMagY = " << avgMagY);
	LOG4CXX_DEBUG(logger,"avgMagZ = " << avgMagZ);

	// If the pitch angle is nearly perpendicular to the flat plane the roll angle cannot be determined with any accuracy
	if (fabsf(varioStatus.pitchAngle) < 80 &&
			varioStatus.getErrorCovariance_P().coeff(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) != 0.0f) {
		RotationMatrix rotMatrix (0.0f,varioStatus.pitchAngle,varioStatus.rollAngle);
		Vector3DType planeMagVector (avgMagX,avgMagY,avgMagZ);
		Vector3DType worldMagVector;

		// I already determined pitch and roll angle. With these now I can move the plane coordinate system into
		// the wold coordinate system, and can determine the heading (yaw angle).
		rotMatrix.calcPlaneVectorToWorldVector(planeMagVector,worldMagVector);

		LOG4CXX_DEBUG(logger,"worldMagX = " << worldMagVector[0]);
		LOG4CXX_DEBUG(logger,"worldMagY = " << worldMagVector[1]);
		LOG4CXX_DEBUG(logger,"worldMagZ = " << worldMagVector[2]);

		varioStatus.heading = FastMath::fastATan2(worldMagVector[0],worldMagVector[1]);
		varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) = 10.0f * 10.0f;
		varioStatus.getSystemNoiseCovariance_Q().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) =
				SQUARE(4.0) * baseIntervalSec;

		LOG4CXX_DEBUG(logger,"Heading = " << varioStatus.heading);

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

	if (!statusInitDone) {
		// Full initialization seemed impossible
	}
}

void BMXSensorBoardDriver::updateKalmanStatus (GliderVarioStatus &varioStatus) {

	/// todo fill me

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
	static constexpr double accFactor = double(0x8000) / 4.0;

	/** \brief Factor to divide the raw gyroscope values by to get the actual deg/s value.
	 *
	 *  Range on the BMX 160 is set from -250deg&s - +250deg/s. This range is 0x8000 in raw values, but as signed int.
	 *
	 */
	static constexpr double gyrFactor = double(0x8000) / 250.0;
	struct BMX160Data bmxData;


	while (!getStopDriverThread()) {
		auto readLen = ioPort->readExactLen((uint8_t *)(&bmxData.header),sizeof(bmxData.header));
		LOG4CXX_DEBUG(logger,"Read " << readLen << "Bytes from the port. Expected " << sizeof(bmxData.header) << " bytes.");

		// Answer the sensor board right away to eliminate the delayed acknowledge. Put the ack piggyback on the answer
		struct BMX160RecvData sendMsg;

		sendMsg.header.unionCode = BMX160RECV_DATA_NONE;
		sendMsg.header.filler = 0;
		sendMsg.header.length = sizeof sendMsg;
		sendMsg.header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR;
		sendMsg.header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR;
		sendMsg.dummy = 0;

		sendMsg.header.crc = 0U;
		sendMsg.header.crc = crcBlock(PPP_INITFCS,&sendMsg,sendMsg.header.length);

		ioPort->writeExactLen((uint8_t*)(&sendMsg),sizeof(sendMsg));

		if (readLen == sizeof(bmxData.header)) {

			struct SensorData &currSensorData = sensorDataArr[currSensorDataIndex];
			currSensorData.accelDataValid = false;
			currSensorData.gyroDataValid = false;
			currSensorData.magDataValid = false;

			LOG4CXX_DEBUG(logger,"bmxData.header.unionCode    = " << bmxData.header.unionCode);
			LOG4CXX_DEBUG(logger,"bmxData.header.length       = " << bmxData.header.length);
			LOG4CXX_DEBUG(logger,"bmxData.header.versionMajor = " << bmxData.header.versionMajor);
			LOG4CXX_DEBUG(logger,"bmxData.header.versionMinor = " << bmxData.header.versionMinor);
			LOG4CXX_DEBUG(logger,"bmxData.header.crc          = " << std::hex << bmxData.header.crc << std::dec);

			if (bmxData.header.versionMajor == BMX160_SENSORBOX_MSG_VERSION_MAJOR &&
					bmxData.header.versionMinor == BMX160_SENSORBOX_MSG_VERSION_MINOR)
			{
				switch (bmxData.header.unionCode) {
				case BMX160DATA_TRIM:
					if (bmxData.header.length == (sizeof(bmxData.header)+sizeof(bmxData.trimData))) {
						if (ioPort->readExactLen((uint8_t *)(&bmxData.trimData),sizeof(bmxData.trimData)) == sizeof(bmxData.trimData)) {
							uint16_t msgCrc, crc;
							msgCrc = bmxData.header.crc;
							bmxData.header.crc = 0U;
							crc = crcBlock(PPP_INITFCS,&bmxData,bmxData.header.length);
							if (crc == msgCrc) {
								magTrimData = bmxData.trimData;
								LOG4CXX_DEBUG(logger, "TrimData = \n"
										<< "\tdig_x1   = " << magTrimData.dig_x1 << "\n"
										<< "\tdig_y1   = " << magTrimData.dig_y1 << "\n"
										<< "\tdig_z1   = " << magTrimData.dig_z1 << "\n"
										<< "\tdig_x2   = " << magTrimData.dig_x2 << "\n"
										<< "\tdig_y2   = " << magTrimData.dig_y2 << "\n"
										<< "\tdig_z2   = " << magTrimData.dig_z2 << "\n"
										<< "\tdig_z3   = " << magTrimData.dig_z3 << "\n"
										<< "\tdig_z4   = " << magTrimData.dig_z4 << "\n"
										<< "\tdig_xy1  = " << magTrimData.dig_xy1 << "\n"
										<< "\tdig_xy2  = " << magTrimData.dig_xy2 << "\n"
										<< "\tdig_xyz1 = " << magTrimData.dig_xyz1);

								// Essential trim data is missing.
								// compensate function would return BMM150_OVERFLOW_OUTPUT_FLOAT = 0.0.
								// Therefore request a reset of the IMU, re-read the magnetometer trim data,
								// and re-send them to me.
								if ((magTrimData.dig_z2 == 0) || (magTrimData.dig_z1 == 0)
										|| (magTrimData.dig_xyz1 == 0) ) {
									sendMsg.header.unionCode = BMX160RECV_DATA_RESET_IMU;
									ioPort->writeExactLen((uint8_t*)(&sendMsg),sizeof(sendMsg));

								}
							} else {
								LOG4CXX_ERROR(logger,
										"CRC error of received magnetometer trim data message");
								// Send a request to send the trim data again
								sendMsg.header.unionCode = BMX160RECV_DATA_RESET_IMU;
								ioPort->writeExactLen((uint8_t*)(&sendMsg),sizeof(sendMsg));
							}
						}
					} else {
						LOG4CXX_ERROR(logger,
								"Alignment error. bmxData.header.length = " << bmxData.header.length
								<< " but sizeof(bmxData.header)+sizeof(bmxData.trimData) = " << (sizeof(bmxData.header)+sizeof(bmxData.trimData)));
					}
					break;
				case BMX160DATA_ACC_GYR_MAG:
					if (bmxData.header.length == (sizeof(bmxData.header)+sizeof(bmxData.accGyrMagData))) {
						if (ioPort->readExactLen((uint8_t*)(&bmxData.accGyrMagData),sizeof(bmxData.accGyrMagData)) == sizeof(bmxData.accGyrMagData)) {
							uint16_t msgCrc, crc;
							msgCrc = bmxData.header.crc;
							bmxData.header.crc = 0U;
							crc = crcBlock(PPP_INITFCS,&bmxData,bmxData.header.length);
							if (crc == msgCrc) {
								GliderVarioMainPriv::LockedCurrentStatus currStatus(*varioMain);

								// advance the index, and wrap it around if necessary
								currSensorDataIndex++;
								currSensorDataIndex &= SIZE_SENSOR_DATA_ARRAY - 1;

								/*
								 * The Y and Z axis of the BMX160 are rotated 180deg around the X axis to my model,
								 * i.e. the Y axis points left, and the Z axis point upward on the BMX160
								 * Therefore all Y and Z values are negated to fit my model.
								 * This is also true for all Y and Z rotations.
								 */

								currSensorData.magX = compensate_x(bmxData.accGyrMagData.magX,bmxData.accGyrMagData.magRHall);
								currSensorData.magY = -compensate_y(bmxData.accGyrMagData.magY,bmxData.accGyrMagData.magRHall);
								currSensorData.magZ = -compensate_z(bmxData.accGyrMagData.magZ,bmxData.accGyrMagData.magRHall);

								LOG4CXX_DEBUG(logger,"magX (uT) = " << currSensorData.magX);
								LOG4CXX_DEBUG(logger,"magY (uT) = " << currSensorData.magY);
								LOG4CXX_DEBUG(logger,"magZ (uT) = " << currSensorData.magZ);
								if (currSensorData.magX != BMM150_OVERFLOW_OUTPUT_FLOAT &&
										currSensorData.magY != -BMM150_OVERFLOW_OUTPUT_FLOAT &&
										currSensorData.magZ != -BMM150_OVERFLOW_OUTPUT_FLOAT
										) {
									currSensorData.magDataValid = true;
								}

								currSensorData.gyroX = double(bmxData.accGyrMagData.gyrX)/ gyrFactor;
								currSensorData.gyroY = -double(bmxData.accGyrMagData.gyrY)/ gyrFactor;
								currSensorData.gyroZ = -double(bmxData.accGyrMagData.gyrZ)/ gyrFactor;
								currSensorData.gyroDataValid = true;

								LOG4CXX_DEBUG(logger,"gyrX (deg/s) = " << currSensorData.gyroX);
								LOG4CXX_DEBUG(logger,"gyrY (deg/s) = " << currSensorData.gyroY);
								LOG4CXX_DEBUG(logger,"gyrZ (deg/s) = " << currSensorData.gyroZ);

								currSensorData.accelX = (double)(bmxData.accGyrMagData.accX)/ accFactor;
								currSensorData.accelY = -(double)(bmxData.accGyrMagData.accY)/ accFactor;
								currSensorData.accelZ = -(double)(bmxData.accGyrMagData.accZ)/ accFactor;
								currSensorData.accelDataValid = true;

								LOG4CXX_DEBUG(logger,"accX (deg/s) = " << currSensorData.accelX);
								LOG4CXX_DEBUG(logger,"accY (deg/s) = " << currSensorData.accelY);
								LOG4CXX_DEBUG(logger,"accZ (deg/s) = " << currSensorData.accelZ);
							} else { // if (crc == msgCrc)
								LOG4CXX_ERROR(logger,
										"CRC error of received acc/gyr/mag sensor data message");
							} // if (crc == msgCrc)
						} // if (ioPort->readExactLen( ...
					} else { // if (bmxData.header.length == ...
						LOG4CXX_ERROR(logger,
								"Alignment error. bmxData.header.length = " << bmxData.header.length
								<< " but (sizeof(bmxData.header)+sizeof(bmxData.accGyrMagData)) = " << (sizeof(bmxData.header)+sizeof(bmxData.accGyrMagData)));
					} // if (bmxData.header.length == ...
					break;
				case BMX160DATA_ACC_GYR:
					if (bmxData.header.length == (sizeof(bmxData.header)+sizeof(bmxData.accGyrData))) {
						if (ioPort->readExactLen((uint8_t*)(&bmxData.accGyrData),sizeof(bmxData.accGyrData)) == sizeof(bmxData.accGyrData)) {
							uint16_t msgCrc, crc;
							msgCrc = bmxData.header.crc;
							bmxData.header.crc = 0U;
							crc = crcBlock(PPP_INITFCS,&bmxData,bmxData.header.length);
							if (crc == msgCrc) {
								// advance the index, and wrap it around if necessary
								currSensorDataIndex++;
								currSensorDataIndex &= SIZE_SENSOR_DATA_ARRAY - 1;

								/*
								 * The Y and Z axis of the BMX160 are rotated 180deg around the X axis to my model,
								 * i.e. the Y axis points left, and the Z axis point upward on the BMX160
								 * Therefore all Y and Z values are negated to fit my model.
								 * This is also true for all Y and Z rotations.
								 */

								currSensorData.gyroX = double(bmxData.accGyrData.gyrX)/ gyrFactor;
								currSensorData.gyroY = -double(bmxData.accGyrData.gyrY)/ gyrFactor;
								currSensorData.gyroZ = -double(bmxData.accGyrData.gyrZ)/ gyrFactor;
								currSensorData.gyroDataValid = true;

								LOG4CXX_DEBUG(logger,"gyrX (deg/s) = " << currSensorData.gyroX);
								LOG4CXX_DEBUG(logger,"gyrY (deg/s) = " << currSensorData.gyroY);
								LOG4CXX_DEBUG(logger,"gyrZ (deg/s) = " << currSensorData.gyroZ);

								currSensorData.accelX = (double)(bmxData.accGyrData.accX)/ accFactor;
								currSensorData.accelY = -(double)(bmxData.accGyrData.accY)/ accFactor;
								currSensorData.accelZ = -(double)(bmxData.accGyrData.accZ)/ accFactor;
								currSensorData.accelDataValid = true;

								LOG4CXX_DEBUG(logger,"accX (g) = " << currSensorData.accelX);
								LOG4CXX_DEBUG(logger,"accY (g) = " << currSensorData.accelY);
								LOG4CXX_DEBUG(logger,"accZ (g) = " << currSensorData.accelZ);
							} else { // if (crc == msgCrc)
								LOG4CXX_ERROR(logger,
										"CRC error of received acc/gyr sensor data message");
							} // if (crc == msgCrc) {
						} // if (ioPort->readExactLen( ...
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
					GliderVarioMeasurementUpdater::accelUpd(currSensorData.accelX * currStatus->gravity,
							currSensorData.accelY * currStatus->gravity,
							currSensorData.accelZ * currStatus->gravity,
							0.01f,0.01f,0.01f,*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
				}
				if (currSensorData.gyroDataValid) {
					GliderVarioMeasurementUpdater::gyroUpd(currSensorData.gyroX,currSensorData.gyroY,currSensorData.gyroZ,
							0.01f,0.01f,0.01f,*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
				}
				if (currSensorData.magDataValid) {
					GliderVarioMeasurementUpdater::compassUpd(currSensorData.magX,currSensorData.magY,currSensorData.magZ,
							4.0f,4.0f,4.0f,*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
				}
			}
		} // if (readLen == sizeof(bmxData.header))
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


} // namespace openEV
