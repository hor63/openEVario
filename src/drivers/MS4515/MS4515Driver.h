/*
 * MS4515Driver.h
 *
 *  Created on: Jan 17 2021
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

#ifndef MS4515DRIVER_H_
#define MS4515DRIVER_H_

#include <fstream>
#include <string>
#include <map>

#include "CommonDefs.h"
#include "MS4515DO.h"
#include "main/driverBase/DifferentialPressureSensorBase.h"
#include "main/driverBase/BusDeviceSensorBase.h"
#include "MS4515Lib.h"
#include "util/io/I2CPort.h"

namespace openEV::drivers::MS4515 {

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
 * If the new 0-offser differs less 0.2mBar (~20km/h at MSL pressure) from the previous 0-offset the new value is taken as new 0-offset,
 * and also immediately stored back to the calibration data file. Else the previous 0-offset is taken.\n
 * Reading and storing of 0-offsets is optional and enabled by configuring the calibration data file name.
 *
 * You should use either a 20inH2O(~50mBar) or a 10inH2O sensor. The 20inH2O sensor is good up to 300km/h, the 10inH2O sensor still up to 210km/h.
 * Anyway use a differential sensor with two ports, one for static pressure the other one for total pressure.
 *
 * \see [MS4515DO online at TE Connectivity](https://www.te.com/usa-en/product-CAT-BLPS0001.html)
 * \see [MS4515DO Datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS4515DO%7FB10%7Fpdf%7FEnglish%7FENG_DS_MS4515DO_B10.pdf%7FCAT-BLPS0001)
 * \see [Interface to MS Connectivity digital pressure modules](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Specification+Or+Standard%7FInterfacing_to_DigitalPressure_Modules%7FA3%7Fpdf%7FEnglish%7FENG_SS_Interfacing_to_DigitalPressure_Modules_A3.pdf%7FCAT-BLPS0001)
 * \see [Configuration, POR, and Power consumption](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Specification+Or+Standard%7FConfiguration_POR_and_Power_Consumption%7FA3%7Fpdf%7FEnglish%7FENG_SS_Configuration_POR_and_Power_Consumption_A3.pdf%7FCAT-BLPS0001)
 *
 */
class MS4515Driver  : public DifferentialPressureSensorBase, public BusDeviceSensorBase {
public:

	enum SensorType {
		SENSOR_TYPE_UNDEFINED, ///< Unknown sensor type.
		SENSOR_TYPE_A, ///< Sensor type A defines pMin as 10%, and pMax as 90% of the physical measurement range of 0x3fff.
		SENSOR_TYPE_B, ///< Sensor type B defines pMin as 5%, and pMax as 95% of the physical measurement range of 0x3fff.
	};

	MS4515Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~MS4515Driver();

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
    virtual void readoutMS4515();

    /** \brief Convert pressure sensor reading to pressure in mBar for a Type A sensor
     *
     *
     * Look up "Interfacing To MEAS Digital Pressure Modules" in the
     * \ref openEV::drivers::MS4515 namespace description how to retrieve
     * the binary register values from the register bank of the sensor.
     *
     * The formula is: \n
     * (output - 16383*loFact) * (pMax-pMin) / (16383*hiFact) + pMin \n
     * \p loFact is 0.05 for B-type, and 0.1 for A-type sensors. \n
     * \p hiFact is 0.9 for B-type, and 0.8 for A-type sensors.
     *
     * Constant factors in the formula up there are pre-calculated in \ref readConfiguration():
     * - \ref f1 = 16383*loFact
     * - \ref f2 = (pMax-pMin)/(16383*hiFact)
     *
     * Thus the formula becomes \n
     * (output - f1) * f2 + pMin
     *
     * @param registerVal Binary value from registers 0 and 1 of the sensor.
     * @return Converted value in mBar
     */
    FloatType convertRegisterPressureToMBar (FloatType registerVal) const;

private:

    /** \brief Type of sensor (A or B)
     *
     * \see SensorType for definitions of A and B types.
     *
     */
    SensorType sensorType = SENSOR_TYPE_UNDEFINED;

    /** \brief Helper for convertRegisterPressureToMBar()
     *
     * \see \ref convertRegisterPressureToMBar() what is it for and how it is calculated.
     */
    FloatType f1 = UnInitVal;

    /** \brief Helper for convertRegisterPressureToMBar()
     *
     * \see \ref convertRegisterPressureToMBar() what is it for and how it is calculated.
     */
    FloatType f2 = UnInitVal;

};

} /* namespace openEV */
#endif /* MS4515DRIVER_H_ */

