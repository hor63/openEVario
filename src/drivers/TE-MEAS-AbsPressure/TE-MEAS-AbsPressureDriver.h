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

#include "CommonDefs.h"
#include "util/GliderVarioExceptionBase.h"

#include "TE-MEAS-AbsPressureDefs.h"
#include "drivers/DriverBase.h"
#include "main/driverBase/BusDeviceSensorBase.h"
#include "TE-MEAS-AbsPressureLib.h"
#include "util/io/I2CPort.h"

namespace openEV::drivers::TE_MEAS_AbsPressure {

	enum DriverTypes {
		DRIVER_TYPE_MS5607 = 0,
		DRIVER_TYPE_MS5611,
		DRIVER_TYPE_MS5637,
		DRIVER_TYPE_MS5803,
		DRIVER_TYPE_MS5805,
		DRIVER_TYPE_MS5837,
		DRIVER_TYPE_MS5839,
		DRIVER_TYPE_MS5840
	};
	static char const * DriverNames [] =
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

class TE_MEAS_AbsPressureCRCErrorException: public GliderVarioExceptionBase {

public:
	TE_MEAS_AbsPressureCRCErrorException(
				char const *source,
				int line,
				char const *description)
	: GliderVarioExceptionBase(source,line,description)
	{}

	};

/** \brief This variable defines unconnected test mode for this driver.
 *
 * Instead of using an I2C port at all the driver only uses test/example data from the data sheets
 * to verify the correct implementation of the temperature and pressure conversion.
 *
 */
#define TE_MEAS_ABS_PRESSURE_TEST_MODE 0

/** \brief Driver for the MS56xx and MS58xx series of absolute pressure sensors from TE Connectivity MEAS.
 *
 * This class implements the variant of the larger sensors with 8 pins.
 * The other variant which exposes only 4 pins is implemented by a derived class.
 *
 * This driver communicates with the sensor via I2C.
 *
 * \see [TE MEAS pressure sensor list](https://www.te.com/global-en/plp/meas/ZEvrl8.html?q=&n=135117%20540192%20540193%20540195%20540174%20540175%20540176%20664985%20664984&d=685834%20685838&type=products&samples=N&inStoreWithoutPL=false&instock=N)
 */
class TE_MEAS_AbsPressureDriver : public BusDeviceSensorBase {
public:

	TE_MEAS_AbsPressureDriver();
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

protected:

    uint8_t i2cAddress = TE_MEAS_AbsPressureI2CAddr;

    /**
     * \brief Use the builtin temperature sensor. The current temperature is used for calculating altitude from pressure and vice versa,
	 * by means of the Barometric formula.
	 *
	 * Using the temperature sensor of the pressure sensor is not advised, and should only be used as a back-stop
	 * When an accurate external temperature sensor is not available.
	 * Reason is that the temperature in the cockpit is usually quite a bit higher than outside due to the greenhouse
	 * effect of the canopy.
     */
    bool useTemperatureSensor = false;

    /** \brief Check the CRC checksum of the PROM containing the conversion coefficients
     *
     * When true a failed CRC check will throw an exception during sensor startup and render the sensor unusable
     * When false a CRC check failure will only print a logger warning.
     */
    bool checkCRC = true;

    /// \brief The I/O port.
    ///
    /// This must be an I2C port.
    io::I2CPort *ioPort = nullptr;

	/// \brief Pointer to the main vario object which also hosts the Kalman filter.
    GliderVarioMainPriv *varioMain = nullptr;

    /// \brief The content of the sensor PROM
    ///
    /// The array fits both variants of the PROM layout.
    uint16_t promArray[PROM_REG_COUNT];

    /** \brief The applicable length of the PROM array is 8 words for the 8-pin sensors, but only 7 words for the 4-pin sensors.
     *
     * The default is an 8-pin sensor.
     * The value can be overwritten by the constructor of the derived 4-pin sensor class.
     */
    uint8_t lenPromArray = PROM_REG_COUNT;

    bool kalmanInitDone = false;
    static constexpr int NumInitValues = 0x10;
    FloatType initValues[NumInitValues];
    int numValidInitValues = 0;

    FloatType pressureVal = UnInitVal;

    /** \brief Temperature in 1/100C
     *
     * Used for temperature compensation of pressure calculation
     *
     */
    int32_t tempCentiC = 2000;

    /** \brief Difference from the reference temperature
     *
     * Used for temperature compensation for pressure calculation.
     * Unit is direct A/D units
     */
    int32_t deltaTemp = 0;

    /// \brief Temperature in C
    FloatType temperatureVal = 20.0f;


#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    /** \brief The main worker thread of this driver
     *
     * \see GliderVarioDriverBase::driverThreadFunction()
     *
     */
    virtual void driverThreadFunction() override;
#endif // #if TE_MEAS_ABS_PRESSURE_TEST_MODE

    /** \brief Process one measurement cycle of the sensor.
     *
     * \see \ref BusDeviceSensorBase::processOneMeasurementCycle()
     */
    virtual void processOneMeasurementCycle () override;

    /** \brief Read the coeffcients from the PROM of the sensor
     *
     * The method is suitable for both variants of the PROM layout because
     * \ref lenPromArray determines the length of the PROM array, and can be overwritten
     * by a derived class.
     */
    virtual void setupSensor() override;

    /** \brief Verify the CRC checksum stored in the PROM with the calculated one.
     *
     * Since there are at least two different algorithms for the calculation the
     * method is pure virtual, and must be implemented by a sub-class.
     *
     * The code is copied verbatim from
     * TE CONNECTIVITY [ Application Note AN520](https://www.te.com.cn/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Specification+Or+Standard%7FAN520_C-code_example_for_MS56xx%7FA%7Fpdf%7FEnglish%7FENG_SS_AN520_C-code_example_for_MS56xx_A.pdf%7FCAT-BLPS0003)
     * except renaming variables, and using uint16_t instead of unsigned int and the likes (AVR code!!!)
     *
     * When the CRC check fails it throws a \ref TE_MEAS_AbsPressureCRCErrorException exception
     * when \ref checkCRC is set true in the configuration.
     *
     * This method suits the 8-pin sensors with the CRC in the 7th word.
     * The the 4-pin sensors this method must be overridden.
     *
     * \throws TE_MEAS_AbsPressureCRCErrorException
     */
	virtual void verifyCRC() = 0;

    /** \brief Start a pressure conversion cycle on the sensor
     *
     */
    virtual void startPressureConversion();

    /** \brief Read out the sensor pressure data
     *
     * The conversion from raw A/D values to the physical value occurs in \ref convertPressure()
     *
     * Before reading out the data wait for the pressure conversion to complete.
     */
    virtual void readoutPressure();

    /** \brief Convert the 3-byte raw value into the pressure in hPa
     *
     * Pure virtual due to slightly different calculations for each sensor type.
     *
     */
    virtual void convertPressure(uint8_t const rawValue[]) = 0;

    /** \brief Start a temperature conversion cycle on the sensor
     *
     */
    virtual void startTemperatureConversion();

    /** \brief Read out the sensor temperature data
     *
     * The conversion from raw A/D values to the physical value occurs in \ref convertTemperature()
     *
     * Before reading out the data wait for the conversion to complete.
     */
    virtual void readoutTemperature();

    /** \brief Convert the 3-byte raw value into the temperature in C
     *
     * The base implementation does not implementation second order compensation.
     * The function calculates \ref tempCentiC, \ref deltaTemp, and \ref temperatureVal.
     *
     */
    virtual void convertTemperature(uint8_t const rawValue[]);

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

    virtual io::PortBase* getIoPortPtr() override;

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    virtual void testFillPromData() = 0;
    virtual void testGetTemperatureVal(uint8_t data[]) = 0;
    virtual void testGetPressureVal(uint8_t data[]) = 0;
#endif

}; // class TE_MEAS_AbsPressureDriver

/** \brief Intermediate class which implements the CRC check applicable to the 8-pin sensors.
 *
 * The 8-pin sensors have a 8-byte PROM area. The checksum is in the byte #7.
 *
 */
class EightPinDriver : public TE_MEAS_AbsPressureDriver{
public:

	EightPinDriver()
	: TE_MEAS_AbsPressureDriver ()
	{}

	virtual ~EightPinDriver();

	virtual void verifyCRC() override;

}; // class EightPinDriver

/** \brief Intermediate class which implements the CRC check applicable to the 4-pin sensors.
 *
 * The 4-pin sensors have a 7-byte PROM area. The checksum is in the byte #0.
 *
 */
class FourPinDriver : public TE_MEAS_AbsPressureDriver{
public:

	FourPinDriver()
	: TE_MEAS_AbsPressureDriver ()
	{
		// The PROM array of all the 4-pin sensors is only 7 words long.
		// the 8th word is un-used, and will later be set 0.
		lenPromArray = 7;
	}

	virtual ~FourPinDriver();

	virtual void verifyCRC() override;

}; // class SevenPinDriver

class MS5803Driver : public EightPinDriver{
public:

	MS5803Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			)
	: DriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly},
	  EightPinDriver ()
	{}

	virtual ~MS5803Driver();

protected:

	/** \brief Perform temperature conversion with second order temperature compensation
	 *
	 * @param rawValue
	 *
	 * \see TE_MEAS_AbsPressureDriver::convertTemperature()
	 */
    virtual void convertTemperature(uint8_t const rawValue[]) override;

    virtual void convertPressure(uint8_t const rawValue[]) override;

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    virtual void testFillPromData();
    virtual void testGetTemperatureVal(uint8_t data[]);
    virtual void testGetPressureVal(uint8_t data[]);
#endif
};

class MS5607Driver : public EightPinDriver{
public:

	MS5607Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			)
	: DriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly},
	  EightPinDriver ()
	{}

	virtual ~MS5607Driver();

protected:

	/** \brief Perform temperature conversion with second order temperature compensation
	 *
	 * @param rawValue
	 *
	 * \see TE_MEAS_AbsPressureDriver::convertTemperature()
	 */
    virtual void convertTemperature(uint8_t const rawValue[]) override;

    virtual void convertPressure(uint8_t const rawValue[]) override;

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    virtual void testFillPromData();
    virtual void testGetTemperatureVal(uint8_t data[]);
    virtual void testGetPressureVal(uint8_t data[]);
#endif
};

class MS5611Driver : public EightPinDriver{
public:

	MS5611Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			)
	: DriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly},
	  EightPinDriver ()
	{}

	virtual ~MS5611Driver();

protected:

	/** \brief Perform temperature conversion with second order temperature compensation
	 *
	 * @param rawValue
	 *
	 * \see TE_MEAS_AbsPressureDriver::convertTemperature()
	 */
    virtual void convertTemperature(uint8_t const rawValue[]) override;

    virtual void convertPressure(uint8_t const rawValue[]) override;

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    virtual void testFillPromData();
    virtual void testGetTemperatureVal(uint8_t data[]);
    virtual void testGetPressureVal(uint8_t data[]);
#endif
};

class MS5637Driver : public FourPinDriver{
public:

	MS5637Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			)
	: DriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly},
	  FourPinDriver ()
	{}

	virtual ~MS5637Driver();

protected:

	/** \brief Perform temperature conversion with second order temperature compensation
	 *
	 * @param rawValue
	 *
	 * \see TE_MEAS_AbsPressureDriver::convertTemperature()
	 */
    virtual void convertTemperature(uint8_t const rawValue[]) override;

    virtual void convertPressure(uint8_t const rawValue[]) override;

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    virtual void testFillPromData();
    virtual void testGetTemperatureVal(uint8_t data[]);
    virtual void testGetPressureVal(uint8_t data[]);
#endif
};

class MS5805Driver : public FourPinDriver{
public:

	MS5805Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			)
	: DriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly},
	  FourPinDriver ()
	{}

	virtual ~MS5805Driver();

protected:

	/** \brief Perform temperature conversion with second order temperature compensation
	 *
	 * @param rawValue
	 *
	 * \see TE_MEAS_AbsPressureDriver::convertTemperature()
	 */
    virtual void convertTemperature(uint8_t const rawValue[]) override;

    virtual void convertPressure(uint8_t const rawValue[]) override;

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    virtual void testFillPromData();
    virtual void testGetTemperatureVal(uint8_t data[]);
    virtual void testGetPressureVal(uint8_t data[]);
#endif
};

class MS5837Driver : public FourPinDriver{
public:

	MS5837Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			)
	: DriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly},
	  FourPinDriver ()
	{}

	virtual ~MS5837Driver();

protected:

	/** \brief Perform temperature conversion with second order temperature compensation
	 *
	 * @param rawValue
	 *
	 * \see TE_MEAS_AbsPressureDriver::convertTemperature()
	 */
    virtual void convertTemperature(uint8_t const rawValue[]) override;

    virtual void convertPressure(uint8_t const rawValue[]) override;

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    virtual void testFillPromData();
    virtual void testGetTemperatureVal(uint8_t data[]);
    virtual void testGetPressureVal(uint8_t data[]);
#endif
};

class MS5839_MS5840Driver : public FourPinDriver{
public:

	MS5839_MS5840Driver(
    	    char const *driverName,
			char const *description,
			char const *instanceName
			)
	: DriverBase {driverName,description,instanceName,TE_MEAS_AbsPressureLib::theOneAndOnly},
	  FourPinDriver ()
	{}

	virtual ~MS5839_MS5840Driver();

protected:

	/** \brief Perform temperature conversion with second order temperature compensation
	 *
	 * @param rawValue
	 *
	 * \see TE_MEAS_AbsPressureDriver::convertTemperature()
	 */
    virtual void convertTemperature(uint8_t const rawValue[]) override;

    virtual void convertPressure(uint8_t const rawValue[]) override;

#if TE_MEAS_ABS_PRESSURE_TEST_MODE
    virtual void testFillPromData();
    virtual void testGetTemperatureVal(uint8_t data[]);
    virtual void testGetPressureVal(uint8_t data[]);
#endif
};

} /* namespace openEV::drivers::TE_MEAS_AbsPressure */
#endif /* ABSPRESSURETE_MEAS_AbsPressureDRIVER_H_ */

