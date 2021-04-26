/*
 * TE_MEAS_AbsPressureDriver.h
 *
 *  Created on: Dec 28, 2020
 *      Author: hor
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

#ifndef ABSPRESSURETE_MEAS_AbsPressureDRIVER_H_
#define ABSPRESSURETE_MEAS_AbsPressureDRIVER_H_

#include <fstream>
#include <string>
#include <map>

#include "OEVCommon.h"

#include "TE-MEAS-AbsPressureDefs.h"
#include "drivers/GliderVarioDriverBase.h"
#include "TE-MEAS-AbsPressureLib.h"
#include "util/io/I2CPort.h"

namespace openEV::drivers::TE_MEAS_AbsPressure {

	static char const * (DriverNames []) =
			{
					"TE-MEAS-MS5607",
					"TE-MEAS-MS5611",
					"TE-MEAS-MS5637",
					"TE-MEAS-MS5803",
					"TE-MEAS-MS5805",
					"TE-MEAS-MS5837",
					"TE-MEAS-MS5839",
					"TE-MEAS-MS5840"
			};
	static constexpr int DriverNamesNum = sizeof(DriverNames) / sizeof(DriverNames[0]);


/** \brief Driver for the MS56xx and MS58xx series of absolute pressure sensors from TE Connectivity MEAS.
 *
 * This class implements the variant of the larger sensors with 8 pins.
 * The other variant which exposes only 4 pins is implemented by a derived class.
 *
 * This driver communicates with the sensor via I2C.
 *
 * \see [TE MEAS pressure sensor list](https://www.te.com/global-en/plp/meas/ZEvrl8.html?q=&n=135117%20540192%20540193%20540195%20540174%20540175%20540176%20664985%20664984&d=685834%20685838&type=products&samples=N&inStoreWithoutPL=false&instock=N)
 */
class TE_MEAS_AbsPressureDriver  : public GliderVarioDriverBase {
public:

	TE_MEAS_AbsPressureDriver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~TE_MEAS_AbsPressureDriver();

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
			GliderVarioMeasurementVector &measurements,
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

    /** \brief The inner main loop of the driver after the port was opened
     *
     * Read data from the sensor, process them, and update the Kalman filter.
     */
    virtual void processingMainLoop ();

    /** \brief Initialize the sensor
     *
     */
    virtual void setupTE_MEAS_AbsPressure();

    /** \brief Start a pressure conversion cycle on the sensor
     *
     */
    virtual void startPressureConversion();

    /** \brief Read out the sensor pressure data
     *
     * Before reading out the data wait for the pressure conversion to complete.
     */
    virtual void readoutPressure();

    /** \brief Start a temperature conversion cycle on the sensor
     *
     */
    virtual void startTemperatureConversion();

    /** \brief Read out the sensor tempereature data
     *
     * Before reading out the data wait for the conversion to complete.
     */
    virtual void readoutTemperature();

    /** \brief Read the coeffcients from the PROM of the sensor
     *
     * This variant implements the MS5611 or MS5803 style with the CRC in the 7th byte
     * The other method with CRC in byte #0 is implemented by an overloaded class.
     */
    virtual void ReadoutCoefficients();

    /** \brief Initialize the QFF based on the latest GPS altitude, and averaged pressure used for status initialization
     *
     * @param[in,out] varioStatus Status and Co-variance of the Kalman filter to be initialized.
	 * @param[in,out] measurements Current measurement vector.
	 * Used for cross-referencing measurements of other drivers during the initialization phase
	 * @param[in,out] varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
     * @param avgPressure Averaged pressure measurements used for status initialization
     */
    void initQFF(
    		GliderVarioStatus &varioStatus,
    		GliderVarioMeasurementVector &measurements,
    		GliderVarioMainPriv &varioMain,
			FloatType avgPressure);

private:

    /** \brief Name of the communications port.
     *
     * I/O ports are defined in the IOPorts section of the configuration
     */
    std::string portName;

    uint8_t i2cAddress = TE_MEAS_AbsPressureI2CAddr;

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
    /// Typically this is a serial port, either real RS-232, or Serial via USB or Bluetooth SPP. \n
    /// TCP will work either.
    io::I2CPort *ioPort = nullptr;

	/// \brief Pointer to the main vario object which also hosts the Kalman filter.
    GliderVarioMainPriv *varioMain = nullptr;

    /// \brief The content of the sensor PROM
    ///
    /// The array fits both variants of the PROM layout.
    uint16_t promArray[PROM_REG_COUNT];

    bool kalmanInitDone = false;
    static constexpr int NumInitValues = 0x10;
    FloatType initValues[NumInitValues];
    int numValidInitValues = 0;

    FloatType pressureVal = NAN;

    FloatType temperatureVal = NAN;

};

} /* namespace openEV */
#endif /* ABSPRESSURETE_MEAS_AbsPressureDRIVER_H_ */

