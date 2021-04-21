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

#include "OEVCommon.h"

#include "AMS5915Defs.h"
#include "drivers/GliderVarioDriverBase.h"
#include "AMS5915Lib.h"
#include "util/io/I2CPort.h"

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
 * If the new 0-offser differs less 0.2mBar (~20km/h at MSL pressure) from the previous 0-offset the new value is taken as new 0-offset,
 * and also immediately stored back to the calibration data file. Else the previous 0-offset is taken.\n
 * Reading and storing of 0-offsets is optional and enabled by configuring the calibration data file name.
 *
 * You should use either a 20inH2O(~50mBar) or a 10inH2O sensor. The 20inH2O sensor is good up to 300km/h, the 10inH2O sensor still up to 210km/h.
 * Anyway use a differential sensor with two ports, one for static pressure the other one for total pressure.
 *
 * \see [AMS5915 online at TE Connectivity](https://www.te.com/usa-en/product-CAT-BLPS0001.html)
 * \see [AMS5915 Datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FAMS5915%7FB10%7Fpdf%7FEnglish%7FENG_DS_AMS5915_B10.pdf%7FCAT-BLPS0001)
 * \see [Interface to MS Connectivity digital pressure modules](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Specification+Or+Standard%7FInterfacing_to_DigitalPressure_Modules%7FA3%7Fpdf%7FEnglish%7FENG_SS_Interfacing_to_DigitalPressure_Modules_A3.pdf%7FCAT-BLPS0001)
 * \see [Configuration, POR, and Power consumption](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Specification+Or+Standard%7FConfiguration_POR_and_Power_Consumption%7FA3%7Fpdf%7FEnglish%7FENG_SS_Configuration_POR_and_Power_Consumption_A3.pdf%7FCAT-BLPS0001)
 *
 */
class AMS5915Driver  : public GliderVarioDriverBase {
public:

	enum SensorType {
		SENSOR_TYPE_UNDEFINED, ///< Unknown sensor type.
		SENSOR_TYPE_A, ///< Sensor type A defines pMin as 10%, and pMax as 90% of the physical measurement range of 0x3fff.
		SENSOR_TYPE_B, ///< Sensor type B defines pMin as 5%, and pMax as 95% of the physical measurement range of 0x3fff.
	};

    /** \brief Dynamic part of the expected error of the sensor as factor of the measurement range.
     *
     * The dynamic part of the expected error is calculated as 1% of the &lt;measured value&gt;/&lt;measurement range&gt;.
     * The dynamic error and the static error calculated by \ref pressureErrorStaticFactor are added together to
     * calculate the variance for a measured value.
     * The combination of a small static error, and a dynamic component comes from the relative accuracy at low values
     * due to automatic offset detection at startup, and the fact that the dynamic pressure is square to the air speed.
     */
    static constexpr FloatType pressureErrorDynFactor = 0.01;

    /** \brief Static part of the expected error of the sensor as factor of the measurement range.
     *
     * The static part of the expected error is calculated as 0.1% of the &lt;measurement range&gt;.
     *
     * \see \ref pressureErrorDynFactor
     */
    static constexpr FloatType pressureErrorStaticFactor = 0.001;

    static constexpr char const * const pressureBiasCalibrationName = "pressureBias";

	AMS5915Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~AMS5915Driver();

    /** \brief Initialize the driver
     *
     * \see GliderVarioDriverBase::driverInit()
     */
    virtual void driverInit(GliderVarioMainPriv &varioMain) override;


    /** \brief Read the configuration
     *
     * \see \ref GliderVarioDriverBase::readConfiguration()
     */
    virtual void readConfiguration (Properties4CXX::Properties const &configuration) override;

    /** \brief Initialize the Kalman filter status from initial sensor measurements
     *
     * \see GliderVarioDriverBase::initializeStatus()
     */
    virtual void initializeStatus(
    		GliderVarioStatus &varioStatus,
			GliderVarioMeasurementVector &measurements,
			GliderVarioMainPriv &varioMain) override;

    /** \brief Callback to update the Kalman filter status based on received data.
     *
     * \see GliderVarioDriverBase::updateKalmanStatus()
     */
    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) override;

    /** \brief Convert pressure sensor reading to pressure in mBar for a Type A sensor
     *
     * Look up "Interfacing To MEAS Digital Pressure Modules" in the
     * \ref openEV::drivers::AMS5915 namespace description how to retrieve
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


protected:


    /** \brief The main worker thread of this driver
     *
     * \see GliderVarioDriverBase::driverThreadFunction()
     *
     */
    virtual void driverThreadFunction() override;

    /** \brief The inner main loop of the driver after the port was opened
     *
     * Read data from the sensor, process them, and update the Kalman filter.
     */
    virtual void processingMainLoop ();

    /** \brief Read out the sensor data
     *
     * Before reading out the data wait for the conversion to complete.
     */
    virtual void readoutAMS5915();

private:

    /** \brief Name of the communications port.
     *
     * I/O ports are defined in the IOPorts section of the configuration
     */
    std::string portName;

    uint8_t i2cAddress = AMS5915I2CAddr;

    /**
     * Use the builtin temperature sensor. The current temperature is used for calculating altitude from pressure and vice versa,
	 * by means of the Barometric formula.
	 * Using the temperature sensor of the pressure sensor is not advised, and should only be used as a back-stop
	 * When an accurate external temperature sensor is not available.
	 * Reason is that the temperature in the cockpit is usually quite a bit higher than outside due to the greenhouse
	 * effect of the canopy.
	 * Optional. Default false.
     */
    bool useTemperatureSensor = false;

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
    /// This must be an I2C port.
    io::I2CPort *ioPort = nullptr;

	/// Pointer to the main vario object which also hosts the Kalman filter.
    GliderVarioMainPriv *varioMain = nullptr;

    bool kalmanInitDone = false;
    static constexpr int NumInitValues = 0x10;
    FloatType initValues[NumInitValues];
    int numValidInitValues = 0;

    /** \brief Type of sensor (A or B)
     *
     * \see SensorType for definitions of A and B types.
     *
     */
    SensorType sensorType = SENSOR_TYPE_UNDEFINED;

    /// Estimated bias of the sensor in mBar/hPa
    FloatType pressureBias = NAN;

    /// Latest temperature value in C
    FloatType temperatureVal = NAN;

    /** \brief Minimum pressure of the defined range in mBar.
     *
     * The configuration values can be defined in in inH20 (See my rant in \ref InchH20toMBar)
     * because the sensors are defined this way, and you can simply transcribe from the sensor type.
     */
    FloatType pMin = NAN;

    /** \brief Maximum pressure of the defined range in mBar
     *
     * \see \ref pMin
     */
    FloatType pMax = NAN;

    /** \brief Pressure range of the sensor in mBar.
     *
     * Range is \ref pMax - \ref pMin.
     */
    FloatType pressureRange = NAN;

    /** \brief Static error component of measurements
     *
     * This value is calculated in readConfiguration() because it only depends on the range.
     * \see pressureErrorDynFactor
     */
    FloatType pressureErrorStatic = NAN;

    /** \brief Helper for convertRegisterPressureToMBar()
     *
     * \see \ref convertRegisterPressureToMBar() what is it for and how it is calculated.
     */
    FloatType f1 = NAN;

    /** \brief Helper for convertRegisterPressureToMBar()
     *
     * \see \ref convertRegisterPressureToMBar() what is it for and how it is calculated.
     */
    FloatType f2 = NAN;

    /// Name of the calibration data parameter file
    std::string calibrationDataFileName;

    /// Loaded and parsed calibration data
    Properties4CXX::Properties *calibrationDataParameters = nullptr;

};

} /* namespace openEV */
#endif /* AMS5915DRIVER_H_ */

