/*
 * MPL3115Driver.h
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

#ifndef ABSPRESSUREMPL3115DRIVER_H_
#define ABSPRESSUREMPL3115DRIVER_H_

#include <fstream>
#include <string>
#include <map>

#include "CommonDefs.h"
#include "MPL3115A2.h"
#include "drivers/DriverBase.h"
#include "MPL3115Lib.h"
#include "util/io/I2CPort.h"

namespace openEV::drivers::MPL3115 {

/** \brief Driver for the MPL3115A2 absolute pressure sensor from NXP
 *
 * This driver communicates with the sensor via I2C.
 *
 * \see [MPL3115A2 web site](https://www.nxp.com/products/sensors/pressure-sensors/barometric-pressure-15-to-150-kpa/20-to-110-kpa-absolute-digital-pressure-sensor:MPL3115A2)
 * \see [MPL3115A2 data sheet](https://www.nxp.com/docs/en/data-sheet/MPL3150A2.pdf)
 */
class MPL3115Driver  : public DriverBase {
public:

	MPL3115Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			);
	virtual ~MPL3115Driver();

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
    virtual void setupMPL3115();

    /** \brief Start a conversion cycle on the sensor
     *
     */
    virtual void startConversionMPL3155();

    /** \brief Read out the sensor data
     *
     * Before reading out the data wait for the conversion to complete.
     */
    virtual void readoutMPL3155();

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

    uint8_t i2cAddress = MPL3115A2I2CAddr;

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

    /// \brief The I/O port.
    ///
    /// Typically this is a serial port, either real RS-232, or Serial via USB or Bluetooth SPP. \n
    /// TCP will work either.
    io::I2CPort *ioPort = nullptr;

	/// Pointer to the main vario object which also hosts the Kalman filter.
    GliderVarioMainPriv *varioMain = nullptr;

    bool kalmanInitDone = false;
    static constexpr int NumInitValues = 0x10;
    FloatType initValues[NumInitValues];
    int numValidInitValues = 0;

    /// \brief Absolute pressure in hP.
    FloatType pressureVal = UnInitVal;

    /// \brief Temperature in degrees C.
    FloatType temperatureVal = UnInitVal;

};

} /* namespace openEV */
#endif /* ABSPRESSUREMPL3115DRIVER_H_ */

