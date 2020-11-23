/*
 * NmeaGPSDriver.h
 *
 *  Created on: Aug 13, 2020
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

#ifndef NMEAGPSDRIVER_H_
#define NMEAGPSDRIVER_H_

#include <string>
#include <map>

#include "OEVCommon.h"

#include "drivers/GliderVarioDriverBase.h"
#include "NmeaGPSLib.h"
#include "NMEASet.h"

#include "util/io/StreamPort.h"

namespace openEV::drivers::NMEA0813 {

/** \brief Driver for Bosch BMX160 IMU which is mounted on the hovImuBoard sensor board
 *
 * This driver communicates with the BMX SensorBoard to obtain accelerometer, gyroscope, and magnetometer
 */
class NmeaGPSDriver  : public GliderVarioDriverBase {
public:

	NmeaGPSDriver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~NmeaGPSDriver();

    /** \brief Initialize the driver
     *
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

    /// \see \ref varioMain
    GliderVarioMainPriv *getVarioMain() {return varioMain;}

    /// \see \ref CEP
    float getCEP() const {return CEP;}

    /// \see \ref altStdDev
    float getAltStdDev() const {return altStdDev;}

    /// \see \ref maxStdDeviationPositionInitialization
    float getMaxStdDeviationPositionInitialization() {return maxStdDeviationPositionInitialization;}

    /// \see \ref maxStdDeviationAltitudeInitialization
    float getMaxStdDeviationAltitudeInitialization() { return maxStdDeviationAltitudeInitialization;}

    /// \see \ref maxStdDeviationPositionUpdate
	float getMaxStdDeviationPositionUpdate() {return maxStdDeviationPositionUpdate;}

	/// \see \ref maxStdDeviationAltitudeUpdate
	float getMaxStdDeviationAltitudeUpdate() {return maxStdDeviationAltitudeUpdate;}


protected:


    /** \brief The main worker thread of this driver
     *
     * \see GliderVarioDriverBase::driverThreadFunction()
     *
     */
    void driverThreadFunction() override;

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

    /// \brief The I/O port.
    ///
    /// Typically this is a serial port, either real RS-232, or Serial via USB or Bluetooth SPP. \n
    /// TCP will work either.
    io::StreamPort *ioPort = nullptr;

    /** \brief Circular Error Probable: Radius of 50% probability of the correct position in meters
     *
     * CEP is essentially the standard deviation of the position under most favorable conditions
     * (good antenna, un-obstructed view to the entire sky, big number, and equal distributed and traceable satellites.
     *
     * \see (Wikipedia: Circular error probable)[https://en.wikipedia.org/wiki/Circular_error_probable]
     */
    float CEP = 5.0f;

    /** \brief Standard deviation of the altitude
     *
     * The standard deviation of the altitude usually is a lot worse than the horizontal CEP. \n
     * I am assuming it being 2 times the CEP.
     *
     */
    float altStdDev = 7.5f;

    /** \brief Maximum standard deviation of latitude and longitude in meter for
     * initialization of the position of the Kalman filter
	 *
     * When the standard deviation of either longitude or latitude exceeds the max. std. dev.
     * the measurements are not used to initialize the Kalman filter with the position.
     * Of course the measurements are also not used to update the Kalman filter. They will be ignored.
     *
     * Default is 15 m
     */
    float maxStdDeviationPositionInitialization = 15.0f;


    /** \brief Max standard deviation of altitude MSL in meter for initialization
     * of the altitude of the Kalman filter
     *
     * When the standard deviation of the altitude exceeds the max. std. dev. the measurement
     * is not used to initialize the Kalman filter
     * with the altitude. Of course the measurement is also not used to update the Kalman filter.
     * It will be ignored.
     *
     * Default is 20 m
     */
    float maxStdDeviationAltitudeInitialization = 20.0f;

    /** \brief Max standard deviation of latitude and longitude in meter for the continuous update of the position of the Kalman filter
     *
     * When the standard deviation of either longitude or latitude exceeds the max. std. dev.
     * the measurements are not used to update the Kalman filter with the position. They will be ignored.
     *
     * Default is 15 m
     */
	float maxStdDeviationPositionUpdate = 15.0f;

	/** \brief Max standard deviation of altitude MSL in meter for initialization of the altitude of the Kalman filter
	 *
	 * When the standard deviation of the altitude exceeds the max. std. dev. the measurement is not used to initialize the Kalman filter
	 * with the altitude. Of course the measurement is also not used to update the Kalman filter. It will be ignored.
	 *
	 * Default is 20 m
	 */
	float maxStdDeviationAltitudeUpdate = 20.0f;


    NMEASet nmeaSet;

	/// Pointer to the main vario object which also hosts the Kalman filter.
    GliderVarioMainPriv *varioMain = nullptr;

};

} /* namespace openEV */
#endif /* NMEAGPSDRIVER_H_ */

