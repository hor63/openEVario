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
#include <thread>
#include <chrono>

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
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
     * \see GliderVarioDriverBase::driverInit()
     */
    virtual void driverInit(GliderVarioMainPriv &varioMain) override;


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

    /// Name of the calibration data parameter file
    std::string calibrationDataFileName;

    /// Loaded and parsed calibration data
    Properties4CXX::Properties *calibrationDataParameters = nullptr;

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
    };

    /** \brief Array of sensor data
     *
     * Ring buffer of sensor data.
     * For continuous operation only the recent record which is indicated by \ref currSensorDataIndex is being used.
     * During initialization I use the whole array to get a recent average to prime the Kalman status.
     *
     */
    SensorData sensorDataArr [SIZE_SENSOR_DATA_ARRAY];

    /** \brief Index of current sensor data into sensorDataArr
     *
     * The array \ref sensorDataArr is filled in ring buffer fashion. For status initialization the average is calculated.
     * For continuous updates only the current record is used.
     */
    int currSensorDataIndex = 0;

    /** \brief
     *
     * Calibration data for the BMX160 sensor box
     * Except for the accelerometer these are bias and standard Variance values only.
     *
     */
    struct SensorCalibrationData {

    	/**
    	 * Magnetometer bias can be measured for the magnetometer
    	 * measuring the magnetic field in an arbitrary direction as val1,
    	 * then turn the box 180 degrees that the measured axis points opposite and measure again as val2.
    	 * The bias is now = (val1 + val2) / 2
    	 *
    	 */
    	double magXBias = 0.0;
    	double magYBias = 0.0;
    	double magZBias = 0.0;

    	/// Standard Variance of the magnetometer measurements
    	double magXVariance = 50.0;
    	double magYVariance = 50.0;
    	double magZVariance = 50.0;

    	/// Gyro bias is the easiest: Let the box rest and measure the gyro values. These are the bias.
    	double gyrXBias = 0.0;
    	double gyrYBias = 0.0;
    	double gyrZBias = 0.0;

    	/// Standard Variance of the gyro measurements
    	/// Just leave the sensor box sitting still and measure a series of values and calculate the standard devition, and square it
    	double gyrXVariance = 5.0;
    	double gyrYVariance = 5.0;
    	double gyrZVariance = 5.0;

    	/**
    	 * Accel bias is measured mostly the same way as magnetometer bias.
    	 * However the measured axis should point straight up for val1,
    	 * and straight down for val2. The formula is the same.
    	 */
    	double accelXBias = 0.0;
    	double accelYBias = 0.0;
    	double accelZBias = 0.0;

    	/**
    	 *
    	 * For the accelerometer factor you can get the same measurements val1 and val2 at the same time
    	 * when measuring the bias.
    	 * I actually have a calibrated value for the Accelerometer, i.e. the gravity.
    	 * Val1 is the positive gravitation, and val 2 is negative gravity.
    	 * The formula is: 2 / (val1 - val2)
    	 * Please note that the expected value should always be 1.0g
    	 */
    	double accelXFactor = 1.0;
    	double accelYFactor = 1.0;
    	double accelZFactor = 1.0;

    	/// Standard Variance of the accelerometer measurements
    	double accelXVariance = 0.1;
    	double accelYVariance = 0.1;
    	double accelZVariance = 0.1;


    	/// Value of the local gravity
    	double gravity = GRAVITY;
    	double gravityVariance = 0.1;

    } calibrationData;

    /** \brief Thread object for the calibration data writer thread
     *
     * Writing out the calibration data is a fairly time consuming I/O operation.
     * Therefore it is implemented as a one-shot thread which is re-started every
     * \ref calibrationDataUpdateCycle seconds.
     */
    std::thread calibrationDataWriteThread;

    /// \brief Cycle time of calibration data updates when the Kalman filter is running.
    std::chrono::system_clock::duration calibrationDataUpdateCycle;
    /// \brief Time of the last calibration data update, or the initial load
    std::chrono::system_clock::time_point lastUpdateTime;
    /** \brief Indicator if the previous calibration write run is still active or finished.
     *
     * Indicator if the thread code actually ran to the end.
     * This prevents blocking the driver thread when it joins the last thread run.
     */
    volatile bool calibrationWriterRunning = false;

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

    /** \brief Thread function of \ref calibrationDataWriteThread
     *
     * Analyze the current status.
     *   - When the variance of the gyro bias is smaller than the one of the calibration data update the gyro calibration data.
     *   - When the variance of the magnetic bias (incl. Variance) is smaller than the calibration data update the mag bias data.
     *   - Accelerometer calibration data are not being touched. I presume they are stable.
     *   There is also no accelerometer bias and factor in the model. The Gravity parameter in the model actually applies only to the
     *   Z axis.
     *
     * If any calibration data was updated write out the updated configuration back into the configuration parameter file.
     *
     * *Note*: This function runs in an own thread!
     *
     */
    void calibrationDataWriteFunc();
};

} /* namespace openEV */
#endif /* BMXSENSORBOARDDRIVER_H_ */

