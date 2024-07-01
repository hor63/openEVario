/*
 * AMS5915Driver.h
 *
 *  Created on: Apr 21 2021
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2021  Kai Horstmann
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

#ifndef AMS5915DRIVER_H_
#define AMS5915DRIVER_H_

#include <fstream>
#include <string>
#include <map>

#include "AMS5915Defs.h"
#include "main/driverBase/DifferentialPressureSensorBase.h"
#include "main/driverBase/BusDeviceSensorBase.h"
#include "AMS5915Lib.h"

namespace openEV::drivers::AMS5915 {

/** \brief Driver for TE Connectivity Measurement Specialists (MEAS) 4515DO pressure sensors.
 *
 * This driver focuses and assumes sole use of the sensor as differential sensor for measuring dynamic (impact) pressure
 * as a measure of air speed.
 *
 * The driver first reads the factory calibration values from the ROM in the sensor.
 * In the startup phase it tries to determine the 0-offset by measuring pressure a couple times, assuming the plane is standing still,
 * and no significant wind is blowing into it. \n
 * If the wind is stronger or the device is even switched on mid-flight it falls back to a previously stored 0-offset value.
 * For this purpose a measured 0-offset is compared with the previous value.
 * If the new 0-offset differs less 0.2mBar (~20km/h at MSL pressure) from the previous 0-offset the new value is taken as new 0-offset,
 * and also immediately stored back to the calibration data file. Else the previous 0-offset is taken.\n
 * Reading and storing of 0-offsets is optional and enabled by configuring the calibration data file name.
 *
 * You should use either a 50mBar or a 35mBar sensor. The 50mBar sensor is good up to 300km/h, but the 35mBar is still for 250km/h.
 * Anyway use a differential sensor with two ports, one for static pressure the other one for total pressure.
 *
 * \see [AMS5915 online at Analog Microelectronics](https://www.analog-micro.com/en/products/pressure-sensors/board-mount-pressure-sensors/ams5915/)
 * \see [AMS5915 Datasheet](https://www.analog-micro.com/products/pressure-sensors/board-mount-pressure-sensors/ams5915/ams5915-datasheet.pdf)
 *
 */
class AMS5915Driver  : public DifferentialPressureSensorBase, public BusDeviceSensorBase {
public:

	AMS5915Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~AMS5915Driver();

    /** \brief Read the configuration
     *
     * \see \ref DriverBase::readConfiguration()
     */
    virtual void readConfiguration (Properties4CXX::Properties const &configuration) override;

protected:

    /** \brief Fill calibration data parameter list; driver specific
     *
     *	\see DriverBase::fillCalibrationDataParameters()
     */
    virtual void fillCalibrationDataParameters () override;

    /** \brief Driver specific function to apply calibration data to the driver instance
     *
     * \see DriverBase::applyCalibrationData()
     */
    virtual void applyCalibrationData() override;

    /** \brief Process one measurement cycle of the sensor.
     *
     * \see \ref BusDeviceSensorBase::processOneMeasurementCycle()
     */
    virtual void processOneMeasurementCycle () override;

    /** \brief Read out the sensor data
     *
     * Before reading out the data wait for the conversion to complete.
     */
    virtual void readoutAMS5915();

    /** \brief Convert pressure sensor reading to pressure in mBar
     *
     * Calculation is simple: Interpolate the sensor reading between \ref pMin at a register value \ref AMS5915PressureRangeMinCount
     * and \ref pMax at a register value \ref AMS5915PressureRangeMaxCount
     *
     * @param registerVal Binary value from register index \ref AMS5915_PRESSURE_HIGH and \ref AMS5915_PRESSURE_LOW.
     * @return Converted value in mBar
     */
    FloatType convertRegisterPressureToMBar (uint16_t registerVal) const;

};

} /* namespace openEV */
#endif /* AMS5915DRIVER_H_ */

