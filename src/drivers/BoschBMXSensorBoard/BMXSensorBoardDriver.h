/*
 * BMXSensorBoardDriver.h
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

#ifndef BMXSENSORBOARDDRIVER_H_
#define BMXSENSORBOARDDRIVER_H_

#include <fstream>
#include <string>
#include <map>

#include "OEVCommon.h"

// Shared structures and constants with the sensor board MCU firmware.
#include "horOvIp-I2C-Bridge/BMX160net.h"

#include "drivers/GliderVarioDriverBase.h"
#include "BMXSensorBoardLib.h"
#include "util/io/StreamPort.h"

namespace openEV {

/** \brief Driver for Bosch BMX160 IMU which is mounted on the hovImuBoard sensor board
 *
 * This driver communicates with the BMX SensorBoard to obtain accelerometer, gyroscope, and magnetometer.
 *
 * The hardware of sensor board is available as KiCad schematics and BOM in my GitHub repository [hovImuBoard](https://github.com/hor63/hovImuBoard)
 * The firmware for the AVR micro controller on the board is in my GitHub repository [horOvIp-I2C-Bridge](https://github.com/hor63/horOvIp-I2C-Bridge)
 *
 * Communications with the sensor board is via TCP/IP. The board connects to the computer via serial line using SLIP.
 *
 */
class BMXSensorBoardDriver  : public GliderVarioDriverBase {
public:

	BMXSensorBoardDriver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~BMXSensorBoardDriver();

    /** \brief Initialize the driver
     *
     * \see GliderVarioDriverBase::driverInit()
     */
    virtual void driverInit() override;


    /** \brief Read the configuration
     *
     * \see GliderVarioDriverBase::readConfiguration()
     */
    virtual void readConfiguration (Properties4CXX::Properties const &configuration) override;

    /** \brief Initialize the Kalman filter status from initial sensor measurements
     *
     * \see GliderVarioDriverBase::initializeStatus()
     */
    virtual void initializeStatus(
    		GliderVarioStatus &varioStatus,
			GliderVarioMainPriv &varioMain) override;

    /** \brief Callback to update the Kalman filter status based on received data.
     *
     * \see GliderVarioDriverBase::updateKalmanStatus()
     */
    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) override;

protected:


    /** \brief The main worker thread of this driver
     *
     * \see GliderVarioDriverBase::driverThreadFunction()
     *
     */
    virtual void driverThreadFunction() override;

    /** \brief The main loop of the driver after the port was opened
     *
     * Read data from the sensor, process them, and update the Kalman filter.
     */
    virtual void processingMainLoop ();

private:

    /** \brief Name of the communications port.
     *
     * I/O ports are defined in the IOPorts section of the configuration
     */

    std::string portName;

    /** \brief Timeout in seconds between recovery attempts when an error in the main loop occurs.
     *
     * Configuration parameter is "errorTimeout" in the driver section.
     */
    int32_t errorTimeout = 10;

    /** \brief Maximum number of retries upon consecutive errors in the main loop.
     *
     * A value <= 0 means that the number of retries is unlimited.
     *
     * When the maximum number of retries is exceeded the main loop terminates and the driver ceases to operate
     *
     * Configuration parameter is "errorMaxNumRetries" in the driver section.
     *
     */
    int32_t errorMaxNumRetries = 0;

    /// \brief The I/O port. Typically this is a TCP port.
    io::StreamPort *ioPort = nullptr;

    /// \brief BMX160 magnetometer trim data structure
    struct bmm150_trim_registers magTrimData;

    /// \brief Is the status initialization done
    bool statusInitDone = false;

    /// \brief Size of the array \ref sensorDataArr
    static constexpr int SIZE_SENSOR_DATA_ARRAY = 16;

    /** \brief Structure holding one set of sensor data
     *
     */
    struct SensorData {
		bool accelDataValid; ///< \brief Are \p accelX, \p accelY, and \p accelZ valid?
		float accelX; ///< \brief Acceleration along the X axis in g. X is forward.
		float accelY; ///< \brief Acceleration along the Y axis in g. Y is to the right.
		float accelZ; ///< \brief Acceleration along the Z axis in g. Z is *downward*! Gravitation measures as -1g!

		bool gyroDataValid; ///< \brief Are \p gyroX, \p gyroY, and \p gyroZ valid?
		float gyroX; ///< \brief Roll rate around the X axis in deg/sec. Positive value is rolling right.
		float gyroY; ///< \brief Pitch rate around the Y axis in deg/sec. Positive value is pitching up.
		float gyroZ; ///< \brief Yaw rate around the Z axis in deg/sec. Positive value is yawing/turning right.

		bool magDataValid; ///< \brief Are \p magX, \p magY, and \p magZ valid?
		float magX; ///< \brief Magnetic field strength along the X axis in uT.
		float magY; ///< \brief Magnetic field strength along the Y axis in uT.
		float magZ; ///< \brief Magnetic field strength along the Z axis in uT.
    } sensorDataArr [SIZE_SENSOR_DATA_ARRAY];

    /** \brief Index of current sensor data into sensorDataArr
     *
     * The array is filled in ring buffer fashion. For status initialization the average is used.
     * For continuous updates only the current record is used.
     */
    int currSensorDataIndex = 0;

    /**
     * @brief This internal API is used to obtain the compensated
     * magnetometer x axis data(micro-tesla) in float.
     */
    float compensate_x(int16_t mag_data_x, uint16_t data_rhall);

    /**
     * @brief This internal API is used to obtain the compensated
     * magnetometer y axis data(micro-tesla) in float.
     */
    float compensate_y(int16_t mag_data_y, uint16_t data_rhall);

    /*!
     * @brief This internal API is used to obtain the compensated
     * magnetometer z axis data(micro-tesla) in float.
     */
    float compensate_z(int16_t mag_data_z, uint16_t data_rhall);

    /** \brief Initialize the Kalman status from the accelerometer measurements
     *
     * Actually I am *not* initializing the acceleration values of the Kalman status.
     * Reason is that the acceleration of the model is not exacly the accelerometer measurements of the IMU when the body
     * is not perfectly flat, i.e. neither pitch nor roll applies. Particularly the roll angle is anything but 0 in a glider on the ground.
     *
     * Instead I am initializing the roll and pitch angle assuming that the plane is stationary during the initialization which usually happens
     * when the plane is sitting on the ground. But even in level flight that should work to a certain degree.
     *
     * @param varioStatus The Kalman status to be initialized
     * @param varioMain Vario main object
     * @param sumSensorData Sensor data summed up over \p numAccelData times.
     * @param numAccelData Number of accelerometer measurements summed up in \p numSensorData
     */
    void initializeStatusAccel(
    		GliderVarioStatus &varioStatus,
    		GliderVarioMainPriv &varioMain,
    		struct SensorData const &sumSensorData,
    		int numAccelData
    		);

    /** \brief Initialize the Kalman status with the gyroscope measurements
     *
     * Initialize the gyroscope bias which is in body coordinates anyway. So no issues with non-flat attitudes whatsoever.
     *
     * @param varioStatus The Kalman status to be initialized
     * @param varioMain Vario main object
     * @param sumSensorData Sensor data summed up over \p numGyroData times.
     * @param numGyroData Number of gyroscope measurements summed up in \p numSensorData
     */
    void initializeStatusGyro(
    		GliderVarioStatus &varioStatus,
    		GliderVarioMainPriv &varioMain,
    		struct SensorData const &avgSensorData,
    		int numGyroData
    		);

    /** \brief Initialize the Kalman status with the magnetometer measurements
     *
     * Rotate the measured vector into the world plane, and calculate the yaw angle (direction) from the horizontal component
     *
     * This function requires that \ref initializeStatusAccel was called before to determine the roll and pitch angle.
     *
     * @param varioStatus The Kalman status to be initialized
     * @param varioMain Vario main object
     * @param sumSensorData Sensor data summed up over \p numMagData times.
     * @param numMagData Number of magnetometer measurements summed up in \p numSensorData
     */
    void initializeStatusMag(
    		GliderVarioStatus &varioStatus,
    		GliderVarioMainPriv &varioMain,
    		struct SensorData const &sumSensorData,
    		int numMagData
    		);

};

} /* namespace openEV */
#endif /* BMXSENSORBOARDDRIVER_H_ */

