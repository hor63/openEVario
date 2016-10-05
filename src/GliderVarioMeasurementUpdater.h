/*
 * GliderVarioMeasurementUpdater.h
 *
 *  Created on: Feb 14, 2016
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016  Kai Horstmann
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

#ifndef GLIDERVARIOMEASUREMENTUPDATER_H_
#define GLIDERVARIOMEASUREMENTUPDATER_H_

#include "GliderVarioStatus.h"
#include "GliderVarioMeasurementVector.h"

namespace openEV {

/***
 * \brief Functional class which performs sequential status updates. This class is stateless.
 * Functional class which performs sequential status updates. This class is stateless.
 * The idea is taken from
 * <a href="http://www.artechhouse.com/static/sample/groves-005_ch03.pdf" >Groves - Kalman Filter-Based Estimation</a>, page 107, 3.2.7  Sequential Measurement Update
 * The functionality is split into two parts:
 * - A specific function for each measurement which performs necessary conversions into the model units of measurement, and prepares the
 * Jacobian of the measure matrix row of the concerned measurement.
 * - #calcSingleMeasureUpdate which performs the actual status update based on the prepared values and Jacobian measure matrix row. This function is
 * applicable to all measurements.
 * 
 */
class GliderVarioMeasurementUpdater {
public:

	/**
	 * \brief Update the status vector with a new measurement of the latitude
	 * Update the status vector with a new measurement of the latitude
	 * @param[in] measuredLatitude Latitude in degrees North from the GPS receiver
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void GPSLatitudeUpd (
			FloatType measuredLatitude,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the longitude
	 * Update the status vector with a new measurement of the longitude
	 * @param[in] measuredLongitude Longitude in degrees East
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void GPSLongitudeUpd (
			FloatType measuredLongitude,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the GPS altitude
	 * Update the status vector with a new measurement of the GPS altitude
	 * @param[in] measuredAltitudeMSL GPS altitude MSL (above geoid)
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void GPSAltitudeUpd (
			FloatType measuredAltitudeMSL,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the GPS heading over ground
	 * Update the status vector with a new measurement of the GPS heading over ground
	 * @param[in] measuredCourseOverGround True course over ground in deg
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void GPSHeadingUpd (
			FloatType measuredCourseOverGround,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the GPS speed over ground
	 * Update the status vector with a new measurement of the GPS speed over ground
	 * @param[in] measuredSpeedOverGround Speed over ground in kt
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void GPSSpeedUpd (
			FloatType measuredSpeedOverGround,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);


	/**
	 * \brief Update the status vector with a new measurement of the acceleration along the plane roll (X) axis
	 * Update the status vector with a new measurement of the acceleration along the plane roll (X) axis
	 * @param[in] measuredAccelX Acceleration along the body X axis. Forward is positive.
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void accelXUpd (
			FloatType measuredAccelX,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the acceleration along the plane pitch (Y) axis
	 * Update the status vector with a new measurement of the acceleration along the plane pitch (Y) axis
	 * @param[in] measuredAccelY Acceleration along the body Y axis. Right is positive.
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void accelYUpd (
			FloatType measuredAccelY,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the acceleration along the plane yaw (Z) axis
	 * Update the status vector with a new measurement of the acceleration along the plane yaw (Z) axis
	 * @param[in] measuredAccelZ Acceleration along the body Z axis. Downward is positive.
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void accelZUpd (
			FloatType measuredAccelZ,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the roll rate around the plane X axis
	 * Update the status vector with a new measurement of the roll rate around the plane X axis
	 * @param[in] measuredRollRateX Rotation rate around the body X axis in degrees per second. Right hand roll is positive.
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void gyroXUpd (
			FloatType measuredRollRateX,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the turn rate around the plane Y axis
	 * Update the status vector with a new measurement of the turn rate around the plane Y axis
	 * @param[in] measuredPitchRateY Rotation rate around the body Y axis in degrees per second. Upward pitch is positive.
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void gyroYUpd (
			FloatType measuredPitchRateY,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the turn (yaw) rate around the plane Z axis
	 * Update the status vector with a new measurement of the turn (yaw) rate around the plane Z axis
	 * @param[in] measuredYawRateZ Rotation rate around the body Z axis in degrees per second. Right (clockwise) turn is positive.
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void gyroZUpd (
			FloatType measuredYawRateZ,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the magnetometer readings along all 3 axis.
	 * Update the status vector with a new measurement of the magnetometer readings along all 3 axis.
	 * The unit of the measurements is irrelevant but must be equal for all three axis. Internally they are normalized because the measurements
	 * are used for attitude estimations only.
	 * @param[in] measuredMagFlowX Magnetic flow along the X axis of the body
	 * @param[in] measuredMagFlowY Magnetic flow along the Y axis of the body
	 * @param[in] measuredMagFlowZ Magnetic flow along the Z axis of the body
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void compassUpd (
			FloatType measuredMagFlowX,
			FloatType measuredMagFlowY,
			FloatType measuredMagFlowZ,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the pressure altitude
	 * Update the status vector with a new measurement of the pressure altitude
	 * Converting static pressure to altitude uses formulas from
	 * <a href="https://en.wikipedia.org/wiki/Barometric_formula" >Wikipedia: Barometric formula</a>. Use formula 1 which includes a temperature gradient
	 * In contrast to the standard atmosphere I am using a gradient of 1K/100m. The boundary layer which we glider pilots populate the air is usually
	 * thoroughly mixed by the thermals, and the gradient approaches an dry adiabatic indifferent gradient, i.e 1K/100m.
	 * I am ignoring water vapor in all calculations.
	 * @param[in] measuredStaticPressure Static pressure in Pascal
	 * @param[in] measuredTemperature Ambient temperature in Degrees Celsius at the current altitude
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void staticPressureUpd (
			FloatType measuredStaticPressure,
			FloatType measuredTemperature,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);

	/**
	 * \brief Update the status vector with a new measurement of the true air speed.
	 * Update the status vector with a new measurement of the true air speed.
	 * Converting dynamic pressure to IAS and finally TAS I am using formulas from
	 * <a href="https://en.wikipedia.org/wiki/Dynamic_pressure" >Wikipedia: Dynamic pressure</a>. The required air density comes from
	 * <a href="https://en.wikipedia.org/wiki/Density_of_air" >Wikipedia: Density of air</a>
	 * I am ignoring water vapor in all calculations.
	 * @param[in] measuredDynamicPressure Dynamic (pitot) pressure in Pascal (difference between static and total pressure).
	 * @param[in] measuredTemperature Ambient temperature in Degrees Celsius at the current altitude
	 * @param[in,out] measurementVector The applicable column is updated for information purposes.
	 * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
	 */
	static void dynamicPressureUpd (
			FloatType measuredDynamicPressure,
			FloatType measuredTemperature,
			GliderVarioMeasurementVector const &measurementVector,
    		GliderVarioStatus &varioStatus
			);
	

protected:

    /**
     * Calculates the status update for one measurement value
     * This is the generic part which deals with calculating the Kalman gain, and updates the state estimate and the status covariance.
     * This routine requires some preparation by the specific update routines which apply the (usually non-linear) measurement function
     * to the latest state estimate to calculate the expected measurement value as well as the Jacobian of the measurement matrix.
     *
     * @param[in] measuredValue The measured value as it was measured by a sensor. This value may be heavily pre-processed, e.g. the IAS or TAS from the
     * dynamic pressure, and static pressure, or altitude from the absolute pressure sensor
     * @param[in] calculatedValue The theoretical measurement value as it is calculated from the current extrapolated system status.
     * @param[in] measurementVariance The variance of the measure. I assume the measurement variances are independent, i.e. the measurement covariance matrix is diagonal. So for one
     * measurement I can simply pass the variance of the current measurement.
     * @param[in] measRowT The transposed measurement matrix row. This row is calculated or approximated as the Jacobian partial derivates each time.
     * The matrix row is not used to calculate the theoretical measured value. This is already done by the calling function by directly using the not-linear
     * measurement function and passing the result as calculatedValue.
     * The transposed Jacobian is used here to calculate the Kalman gain for this measurement, and applying it to the updated status and covariance.
     * The vector is passed in the transposed form because in the calculation it is used transposed twice, and only once in the original form. So I have to
     * transpose only once (back to the original form).
     * @param[in,out] varioStatus The system status and covariance before and after the measurement update. The status and covariance are directly updated from the difference of the actually measured, and the
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
