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

#include "drivers/GliderVarioDriverBase.h"
#include "BMXSensorBoardLib.h"

namespace openEV {

/** \brief Driver for Bosch BMX160 IMU which is mounted on the hovImuBoard sensor board
 *
 * This driver communicates with the BMX SensorBoard to obtain accelerometer, gyroscope, and magnetometer
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

    /** \brief Start data acquisition
     *
     * \see GliderVarioDriverBase::start()
     */
    void start(GliderVarioMainPriv &varioMain) override;

    /** \brief Suspend the driver temporarily
     *
     * \see GliderVarioDriverBase::suspend()
     */
    virtual void suspend() override;

    /** \brief Resume data acquisition when it was suspended before by suspend()
     *
     * \see GliderVarioDriverBase::resume()
     */
    virtual void resume() override;

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
    void driverThreadFunction() override;

};

} /* namespace openEV */
#endif /* BMXSENSORBOARDDRIVER_H_ */

