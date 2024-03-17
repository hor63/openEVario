/*
 * DifferentialPressureSensorBase.h
 *
 *  Created on: Jan 10, 2024
 *      Author: hor
 */

#ifndef MAIN_DRIVERBASE_DIFFERENTIALPRESSURESENSORBASE_H_
#define MAIN_DRIVERBASE_DIFFERENTIALPRESSURESENSORBASE_H_

#include "CommonDefs.h"
#include "util/io/I2CPort.h"
#include "drivers/DriverBase.h"

namespace openEV {
namespace drivers {

class OEV_MAIN_PUBLIC DifferentialPressureSensorBase: public DriverBase {
public:

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
     * The static part of the expected error is calculated as 0.02% of the &lt;measurement range&gt;.
     *
     * \see \ref pressureErrorDynFactor
     */
    // static constexpr FloatType pressureErrorStaticFactor = 0.001;
    static constexpr FloatType pressureErrorStaticFactor = 0.0002;

    static constexpr char const * const pressureBiasCalibrationName = "pressureBias";

	DifferentialPressureSensorBase (
		    char const *driverName,
			char const *description,
			char const *instanceName,
			DriverLibBase &driverLib
			);
	virtual ~DifferentialPressureSensorBase();

    /** \brief Initialize the driver
     *
     * \see GliderVarioDriverBase::driverInit()
     */
    virtual void driverInit(GliderVarioMainPriv &varioMain) override;

    /** \brief Callback to update the Kalman filter status based on received data.
     *
     * \see GliderVarioDriverBase::updateKalmanStatus()
     */
    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) override;

    /** \brief Initialize the Kalman filter status from initial sensor measurements
     *
     * \see GliderVarioDriverBase::initializeStatus()
     */
    virtual void initializeStatus(
    		GliderVarioStatus &varioStatus,
			GliderVarioMeasurementVector &measurements,
			GliderVarioMainPriv &varioMain) override;

protected:
	/// \brief *Must* be set to a sensible value by the derived class
    uint8_t i2cAddress = 0x7F;

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
    /// This must be an I2C port.
    io::I2CPort *ioPort = nullptr;

	/// Pointer to the main vario object which also hosts the Kalman filter.
    GliderVarioMainPriv *varioMain = nullptr;

    bool kalmanInitDone = false;
    static constexpr int NumInitValues = 0x10;
    FloatType initValues[NumInitValues];
    int numValidInitValues = 0;
    FloatType avgPressureOnStartup = 0.0f;

    /// \brief Status initialization, and dynamic determination of zero-bias are done.
    bool statusInitDone = false;

    /// Estimated bias of the sensor in mBar/hPa
    FloatType pressureBias = UnInitVal;

    /// Latest temperature value in C
    FloatType temperatureVal = UnInitVal;

    /** \brief Calibration data for the IMU
     *
     */
    struct SensorCalibrationData {
    	double pressureBias = 0.0; //< zero-offset of the sensor in hPa
    } calibrationData;

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

private:
};

} /* namespace drivers */
} /* namespace openEV */

#endif /* MAIN_DRIVERBASE_DIFFERENTIALPRESSURESENSORBASE_H_ */
