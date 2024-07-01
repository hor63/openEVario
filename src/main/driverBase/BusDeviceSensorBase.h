/*
 * BusDeviceSensorBase.h
 *
 *  Created on: Jun 26, 2024
 *      Author: hor
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

#ifndef MAIN_DRIVERBASE_BUSDEVICESENSORBASE_H_
#define MAIN_DRIVERBASE_BUSDEVICESENSORBASE_H_

#include "drivers/DriverBase.h"

namespace openEV {
namespace drivers {

class OEV_MAIN_PUBLIC BusDeviceSensorBase: public virtual DriverBase {
public:
	BusDeviceSensorBase();

	virtual ~BusDeviceSensorBase();

protected:

	/// Used by most derived sensor classes to calculate the sleep time between cycles.
	OEVClock::time_point nextStartConversion = OEVClock::now();

	/// Number of attempts to read from the sensor with the same \ref lastErrno.
	/// Is being reset to 0 when an processing cycle succeeds.
	int32_t numRetries = 0;

	/// Set to \p errno when an I/O operation fails.
	/// Is being reset to 0 when an processing cycle succeeds.
	int lastErrno = 0;

    /** \brief The main worker thread of the sensor driver
     *
     *	Implementation of the function for shared ports, particularly I2C buses.
     *
     * \see \ref DriverBase::driverThreadFunction()
     */
	virtual void driverThreadFunction() override;

    /** \brief The inner main loop of the driver after the port was opened
     *
     * Generic method.
     * Calls \ref setupSensor() once at the begining.
     * Then calls \ref processOneMeasurementCycle() in a loop while \ref getStopDriverThread() returns \p false.
     *
     * This method handles I/O errors caused by the connected bus device
     * and cancels the driver instance when the number of repetitive error is exceeded.
     *
     */
    virtual void processingMainLoop ();

    /** \brief Process one measurement cycle of the sensor.
     *
     * The method runs one measurement cycle of the sensor and updates the Kalman filter.
     * When an I/O error occurs (or any other internal error) the method throws an appropriate exception.
     * The caller takes care of default error handling.
     *
     * \see \ref processingMainLoop()
     */
    virtual void processOneMeasurementCycle() = 0;

    /** \brief Setup and prepare the sensor for operations
     *
     * Called before cyclic acquisition of measurements from the sensor starts.
     * Can be used by sensor classes to configure and activate the sensor.
     *
     * The default implementation here does nothing.
     *
     */
    virtual void setupSensor();

};

} /* namespace drivers */
} /* namespace openEV */

#endif /* MAIN_DRIVERBASE_BUSDEVICESENSORBASE_H_ */
