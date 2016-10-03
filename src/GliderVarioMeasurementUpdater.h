/*
 * GliderVarioMeasurementMatrix.h
 *
 *  Created on: Feb 14, 2016
 *      Author: hor
 */

#ifndef GLIDERVARIOMEASUREMENTUPDATER_H_
#define GLIDERVARIOMEASUREMENTUPDATER_H_

#include "GliderVarioStatus.h"
#include "GliderVarioMeasurementVector.h"

namespace openEV {

/***
 * Measurement matrix h of a Kalman filter. 
 * 
 */
class GliderVarioMeasurementUpdater {
public:
	GliderVarioMeasurementUpdater();
	virtual ~GliderVarioMeasurementUpdater();

	/**
	 * update the status vector with a new measurement of the latitude
	 * @param measurementVector
	 * @param varioStatus
	 */
	void gpsLatitudeUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the longitude
	 * @param measurementVector
	 * @param varioStatus
	 */
	void gpsLongitudeUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the gps altitude
	 * @param measurementVector
	 * @param varioStatus
	 */
	void gpsAltitudeUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the gps heading over ground
	 * @param measurementVector
	 * @param varioStatus
	 */
	void gpsHeadingUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the gps speed over ground
	 * @param measurementVector
	 * @param varioStatus
	 */
	void gpsSpeedUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);


	/**
	 * update the status vector with a new measurement of the acceleration along the plane X axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void accelXUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the acceleration along the plane Y axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void accelYUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the acceleration along the plane Z axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void accelZUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the turn rate around the plane X axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void gyroXUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the turn rate around the plane Y axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void gyroYUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the turn rate around the plane Z axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void gyroZUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the magnetometer along the plane X axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void compassXUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the magnetometer along the plane Y axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void compassYUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the magnetometer along the plane Z axis
	 * @param measurementVector
	 * @param varioStatus
	 */
	void compassZUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the pressure altitude
	 * @param measurementVector
	 * @param varioStatus
	 */
	void pressureAltUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * update the status vector with a new measurement of the true air speed
	 * @param measurementVector
	 * @param varioStatus
	 */
	void trueAirSpeedUpd (
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);
	

protected:

    /**
     * Calculates the status update for one measurement value according to
     * <a href="http://www.artechhouse.com/static/sample/groves-005_ch03.pdf" >Groves - Kalman Filter-Based Estimation</a>, page 107, 3.2.7  Sequential Measurement Update
     *
     * @param[in] measuredValue the measured value as it was measured by a sensor. This value may be heavily pre-processed, e.g. the IAS or TAS from the
     * dynamic pressure, and static pressure, or altitude from the absolute pressure sensor
     * @param[in] calculatedValue the theoretical measurement value as it is calculated from the current extrapolated system status.
     * @param]in] the variance of the measure. I assume the measurement variances are independent, i.e. the measurement covariance matrix is diagonal. So for one
     * measurement I can simply pass the variance of the current measurement.
     * @param[in] measRowT The transposed measurement matrix row. This row is calculated or approximated as the Jacobian partial derivates each time.
     * The matrix row is not used to calculate the theoretical measured value. This is already done by the calling function by directly using the not-linear
     * measurement function and passing the result as calculatedValue.
     * The transposed Jacobian is used here to calculate the Kalman gain for this measurement, and applying it to the updated status and covariance.
     * The vector is passed in the transposed form because in the calculation it is used transposed twice, and only once in the original form. So I have to
     * transpose only once (back to the original form).
     * @param[in,out] varioStatus The status before and after. The status and covariance are directly updated from the difference of the actually measured, and the
     * theoretical value.
     */
    static void calcSingleMeasureUpdate (
			FloatType measuredValue,
			FloatType calculatedValue,
			FloatType measurementVariance,
			GliderVarioStatus::StatusVectorType const &measRowT,
    		GliderVarioStatus &varioStatus
			);
};

} /* namespace openEV */

#endif /* GLIDERVARIOMEASUREMENTUPDATER_H_ */
