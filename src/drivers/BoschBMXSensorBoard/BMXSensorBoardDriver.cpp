/*
 * BMXSensorBoardDriver.cpp
 *
 *  Created on: Feb 04, 2020
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

#include <fstream>
#include <chrono>

#include "fmt/format.h"

#include "Properties4CXX/Property.h"

#include "BMXSensorBoardDriver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"
#include "util/RotationMatrix.h"
#include "util/crc.h"

#include "horOvIp-I2C-Bridge/BMX160net.h"

namespace openEV::drivers::BoschBMX160 {


BMXSensorBoardDriver::BMXSensorBoardDriver(
	    char const *driverName,
		char const *description,
		char const *instanceName
		)
: IMUBase {driverName,description,instanceName,BMXSensorBoardLib::theOneAndOnly}
{

#if defined HAVE_LOG4CXX_H
	logger = log4cxx::Logger::getLogger("openEV.Drivers.BMXSensorBoard");
#endif /* HAVE_LOG4CXX_H */

	setSensorCapability(ACCEL_3D);
	setSensorCapability(GYRO_3D);
	setSensorCapability(MAGNETOMETER_3D);

	memset (&magTrimData,0,sizeof(magTrimData));

}


BMXSensorBoardDriver::~BMXSensorBoardDriver() {

}

void BMXSensorBoardDriver::readConfiguration (Properties4CXX::Properties const &configuration) {

	LOG4CXX_INFO(logger, fmt::format (_("{0}: for driver instance \"{1}\""),
			__PRETTY_FUNCTION__, instanceName));

}

void BMXSensorBoardDriver::driverInit(GliderVarioMainPriv &varioMain) {
	ioPort = getIoPort<decltype(ioPort)>(logger);

	IMUBase::driverInit(varioMain);
}

void BMXSensorBoardDriver::driverThreadFunction() {

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
				LOG4CXX_ERROR(logger,fmt::format(_("Error in the main loop of driver instance \"{0}\": "),instanceName,e.what()));
				ioPort->close();

				std::this_thread::sleep_for(errorTimeout);
			}
		}
	}
}

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

		SensorData &currSensorData = sensorDataArr[currSensorDataIndex];
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
								_("CRC error of received magnetometer trim data message"));
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
								_("CRC error of received acc/gyr/mag sensor data message"));
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
								_("CRC error of received acc/gyr sensor data message"));
					} // if (crc == msgCrc) {
				} else { // if (bmxData.header.length == ...
					LOG4CXX_ERROR(logger,
							"Alignment error. bmxData.header.length = " << bmxData.header.length
							<< " but (sizeof(bmxData.header)+sizeof(bmxData.accGyrData)) = " << (sizeof(bmxData.header)+sizeof(bmxData.accGyrData)));
				} // if (bmxData.header.length == ...
				break;
			default:
				LOG4CXX_ERROR (logger,__PRETTY_FUNCTION__ << ": Unknown union code " << (int)bmxData.header.unionCode);

				break;
			} // switch (bmxData.header.unionCode)
		} // if (bmxData.header.versionMajor == ...

		updateKalman(currSensorData);
	}
}

/** \brief This internal API is used to obtain the compensated
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

/** \brief This internal API is used to obtain the compensated
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

/** \brief This internal API is used to obtain the compensated
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
