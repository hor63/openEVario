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
#include "util/RotationMatrix.h"

namespace openEV {

/***
 * \brief Functional class which performs sequential status updates. This class is stateless.
 *
 * The idea is taken from
 * [Groves - Kalman Filter-Based Estimation]http://www.artechhouse.com/static/sample/groves-005_ch03.pdf),
 * page 107, 3.2.7  Sequential Measurement Update (link is defunct, buy
 * [Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, Second Edition from Artechhouse](https://us.artechhouse.com/Principles-of-GNSS-Inertial-and-Multisensor-Integrated-Navigation-Systems-Second-Edition-P2046.aspx)
 * \emoji :smirk: ) \n
 * The functionality is split into two parts:
 * - A specific function for each measurement which performs necessary conversions into the model units of measurement, and prepares the
 * Jacobian of the measure matrix row of the concerned measurement.
 * - #calcSingleMeasureUpdate which performs the actual status update based on the prepared values and Jacobian measure matrix row. This function is
 * applicable to all measurements.
 * 
 */
class OEV_PUBLIC GliderVarioMeasurementUpdater {
public:

    /**
     * \brief Update the status vector with a new measurement of the latitude
     *
     * In contrast to most functions here \p measuredLatitude is double not FloatType (i.e. float)
     * because the latitude scale is from 90deg to 1mm, requiring about 10 decimals, much larger than float
     * Internally the Kalman filter splits the latitude in long whole seconds base and
     * an offset in m which is actually part of the Kalman filter.
     *
     * Note that \p latitudeVariance is in meter&sup2;, not degrees because GPS receivers report the
     * uncertainty in m, and internally the filter uses m for the latitude offset
     *
     * @param[in] measuredLatitude Latitude in degrees North from the GPS receiver
     * @param[in] latitudeVariance Variance of the latitude in meter&sup2;
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void GPSLatitudeUpd (
            double measuredLatitude,
            FloatType latitudeVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );

    /**
     * \brief Update the status vector with a new measurement of the longitude
     *
     * In contrast to most functions here \p measuredLongitude is double not FloatType (i.e. float)
     * because the latitude scale is from 180deg to 1mm, requiring about 11 decimals, much larger than float
     * Internally the Kalman filter splits the latitude in long whole seconds base and
     * an offset in m which is actually part of the Kalman filter.
     *
     * Note that \p longitudeVariance is in meter&sup2;, not degrees because GPS receivers report the
     * uncertainty in m, and internally the filter uses m for the longitude offset
     *
     * @param[in] measuredLongitude Longitude in degrees East
     * @param[in] longitudeVariance variance of the longitude measurement in meter&sup2;
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void GPSLongitudeUpd (
            double measuredLongitude,
            FloatType longitudeVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );

    /**
     * \brief Update the status vector with a new measurement of the GPS altitude
     * Update the status vector with a new measurement of the GPS altitude
     * @param[in] measuredAltitudeMSL GPS altitude MSL (above geoid)
     * @param[in] altitudeVariance Variance of the GPS altitude MSL
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void GPSAltitudeUpd (
            FloatType measuredAltitudeMSL,
            FloatType altitudeVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );

    /**
     * \brief Update the status vector with a new measurement of the GPS heading over ground
     * Update the status vector with a new measurement of the GPS heading over ground
     * @param[in] measuredCourseOverGround True course over ground in degrees
     * @param[in] courseOverGroundVariance Variance of the measured course over ground
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void GPSHeadingUpd (
            FloatType measuredCourseOverGround,
            FloatType courseOverGroundVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );

    /**
     * \brief Update the status vector with a new measurement of the GPS speed over ground
     * Update the status vector with a new measurement of the GPS speed over ground
     * @param[in] measuredSpeedOverGround Speed over ground in kt
     * @param[in] speedOverGroundVariance Variance of the measured speed over ground
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void GPSSpeedUpd (
            FloatType measuredSpeedOverGround,
            FloatType speedOverGroundVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );


    /**
     * \brief Update the status vector with a new measurement of the acceleration at all three axis
     * Update the status vector with a new measurement of the acceleration at all three axis.
     * All three measurements are moved into this one routine because the model coordinate system is a world coordinate system
     * where x points horizontal in heading direction, z points straight down, and y is horizontal, and perpendicular to heading.
     * @param[in] measuredAccelX Acceleration along the body X axis. Forward is positive.
     * @param[in] accelXVariance Variance of the measured acceleration
     * @param[in] measuredAccelY Acceleration along the body Y axis. Right is positive.
     * @param[in] accelYVariance Variance of the measured acceleration
     * @param[in] measuredAccelZ Acceleration along the body Z axis. Downward is positive.
     * @param[in] accelZVariance Variance of the measured acceleration
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void accelUpd (
            FloatType measuredAccelX,
            FloatType accelXVariance,
            FloatType measuredAccelY,
            FloatType accelYVariance,
            FloatType measuredAccelZ,
            FloatType accelZVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );


    /**
     * \brief Update the status vector with a new measurement of the roll rates of the plane along the 3 plane axis
     * Update the status vector with a new measurement of the roll rates of the plane along the 3 plane axis
     * @param[in] measuredRollRateX Rotation rate around the body X axis in degrees per second. Right hand roll is positive.
     * @param[in] rollRateXVariance Variance of the measured roll rate
     * @param[in] measuredPitchRateY Rotation rate around the body Y axis in degrees per second. Pitch up is positive.
     * @param[in] pitchRateYVariance Variance of the measured pitch rate
     * @param[in] measuredYawRateZ Rotation rate around the body Z axis in degrees per second. Rightward yaw is positive.
     * @param[in] yawRateZVariance Variance of the measured yaw rate
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void gyroUpd (
            FloatType measuredRollRateX,
            FloatType rollRateXVariance,
            FloatType measuredPitchRateY,
            FloatType pitchRateYVariance,
            FloatType measuredYawRateZ,
            FloatType yawRateZVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );


    /**
     * \brief Update the status vector with a new measurement of the magnetometer readings along all 3 axis.
     *
     * Update the status vector with a new measurement of the magnetometer readings along all 3 axis.
     *
     * The magnetic measurements are only used to correct the attitude. Except from the local body deviation vector the absolute strength is not relevant
     * Therefore the factor to the unit vector of the attitude is the absolute length of the measurement vector minus the body deviation.
     *
     * @param[in] measuredMagFlowX Magnetic flow along the X axis of the body in uTesla
     * @param[in] measuredMagFlowY Magnetic flow along the Y axis of the body in uTesla
     * @param[in] measuredMagFlowZ Magnetic flow along the Z axis of the body in uTesla
     * @param[in] magFlowXVariance Variance of the magnetic flow X measurement
     * @param[in] magFlowYVariance Variance of the magnetic flow Y measurement
     * @param[in] magFlowZVariance Variance of the magnetic flow Z measurement
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void compassUpd (
            FloatType measuredMagFlowX,
            FloatType measuredMagFlowY,
            FloatType measuredMagFlowZ,
            FloatType magFlowXVariance,
            FloatType magFlowYVariance,
            FloatType magFlowZVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );

    /**
     * \brief Update the status vector with a new measurement of the pressure altitude
     * Update the status vector with a new measurement of the pressure altitude
     *
     * Converting static pressure to altitude uses formulas from
     * [Wikipedia: Barometric formula](https://en.wikipedia.org/wiki/Barometric_formula). Use formula 1 which includes a temperature gradient
     * In contrast to the standard atmosphere I am using a gradient of 1K/100m. In the boundary layer which we glider pilots populate during thermal flight the air is usually
     * thoroughly mixed by the thermals, and the gradient approaches an dry adiabatic indifferent gradient, i.e 1K/100m.
     * *Beware*, this assumption is true for thermal flights only where the boundary layer is pretty thoroughly mixed, and practically indifferent.
     * This is not the case for wave flight where the atmosphere usually has a stable gradient,
     * usually somewhere around -0.5 - 0.6C/100m.
     *
     * I am ignoring water vapor in all calculations.
     * @param[in] measuredStaticPressure Static pressure in hPascal
     * @param[in] measuredTemperature Ambient temperature in Degrees Celsius at the current altitude
     * @param[in] staticPressureVariance Variance of the pressure measurement
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void staticPressureUpd (
            FloatType measuredStaticPressure,
            FloatType measuredTemperature,
            FloatType staticPressureVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );

    /**
     * \brief Update the status vector with a new measurement of the true air speed.
     * Update the status vector with a new measurement of the true air speed.
     * Converting dynamic pressure to IAS and finally TAS I am using formulas from
     * [Wikipedia: Dynamic pressure](https://en.wikipedia.org/wiki/Dynamic_pressure). The required air density comes from
     * [Wikipedia: Density of air](https://en.wikipedia.org/wiki/Density_of_air)
     * I am ignoring water vapor in all calculations.
     * @param[in] measuredDynamicPressure Dynamic (pitot) pressure in hPa(mBar) (difference between static and total pressure).
     * @param[in] measuredTemperature Ambient temperature in Degrees Celsius at the current altitude
     * @param[in] dynamicPressureVariance Variance of the dynamic pressure
     * @param[in,out] measurementVector The applicable column is updated for information purposes.
     * @param[in,out] varioStatus In: status before the measurement update. Out: Status and covariance update with the specific measurement . The update is in-place
     */
    static void dynamicPressureUpd (
            FloatType measuredDynamicPressure,
            FloatType measuredTemperature,
            FloatType dynamicPressureVariance,
            GliderVarioMeasurementVector &measurementVector,
            GliderVarioStatus &varioStatus
    );

    /** \brief Calculate the altitude MSL from the local pressure, and temperature
     *
     * Calculate the altitude MSL from the local pressure, the local temperature, and the pressure at sea level QFF.
     *
     * Use the
     * [Barometric formula](https://en.wikipedia.org/wiki/Barometric_formula).
     * Use formula 1 which includes a temperature gradient. \n
     * Reverse it and resolve it by the altitude.
     * Use the fixed temperature gradient \ref altToPressureStdTemp for the indifferent boundary layer.
     *
     * I am assuming a dry atmosphere, and I am ignoring the effects of water vapor.
     *
     * \p QFF can be measured before take-off, or the (similar) QNH from nearby airports be used.
     * With the absolute GNSS altitude the system can initialize the QFF itself when the GNSS driver diver
     * obtains a valid 3-D fix.
     *
     * @param pressure The local pressure in hPa, i.e. mBar
     * @param QFF The pressure on sea level in hPa, i.e. mBar.
     * @param temperatureAlt Temperature in Celsius.
     * @return Altitude in meter
     */
    static FloatType calcAltFromPressure (
    		double pressure,
			double QFF,
			double temperatureAlt
    		);

    // This stuff is used only for unit tests.
    // These variables contain copies of the local variables used for the measurement updates.
    // These variables are not declared and used in production code.
    static FloatType calculatedValueTst1;
    static Eigen::SparseMatrix<FloatType> measRowTTst1;

    static FloatType calculatedValueTst2;
    static Eigen::SparseMatrix<FloatType> measRowTTst2;

    static FloatType calculatedValueTst3;
    static Eigen::SparseMatrix<FloatType> measRowTTst3;

    /// Activate the unit test mode, i.e. copy the calculation results into calculatedValueTst1 and measRowTTst1 and so forth.
    static void setUnitTestMode(bool unitTestMode );
    /// Get the unit test mode, i.e. if the calculation results are copied into calculatedValueTst1 and measRowTTst1 and so forth.
    static bool getUnitTestMode();

protected:

    /** \brief Calculate the inverse of a 3x3 matrix
     *
     * The algorithm has been taken from
     * [How to Calculate the Inverse Matrix for 2×2 and 3×3 Matrices](https://aaronschlegel.me/calculate-matrix-inverse-2x2-3x3.html)
     * by [Aaron Schlegel](https://aaronschlegel.me/author/aaron-schlegel.html)
     *
     * @param[out] inverse The inverted matrix
     * @param[in] org The original 3x3 matrix
     * @return \p true when successful, \p false when the determinate of the matrix is 0, and the inverse cannot be calculated.
     * @see [Matrix Inverse](https://mathworld.wolfram.com/MatrixInverse.html) by Wolfram MathWorld
     */
    OEV_LOCAL static bool calcInverse3D (
    		Eigen::SparseMatrix <FloatType> &inverse,
			Eigen::SparseMatrix <FloatType> const &org);


    /** \brief Calculate the status update for one measurement value
     *
     * This is the generic part which deals with calculating the Kalman gain, and updates the state estimate and the status covariance.
     * This routine requires some preparation by the specific update routines which apply the (usually non-linear) measurement function
     * to the latest state estimate to calculate the expected measurement value as well as the Jacobian of the measurement matrix.
     *
     * @param[in] measuredValue The measured value as it was measured by a sensor. This value may be heavily pre-processed, e.g. the IAS or TAS from the
     * dynamic pressure, and static pressure, or altitude from the absolute pressure sensor
     * @param[in] calculatedValue The theoretical measurement value as it is calculated from the current extrapolated system status.
     * @param[in] measurementVariance_R The variance of the measure. I assume the measurement variances are independent, i.e. the measurement covariance matrix is diagonal. So for one
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
    OEV_LOCAL static void calcSingleMeasureUpdate (
            FloatType measuredValue,
            FloatType calculatedValue,
            FloatType measurementVariance_R,
            Eigen::SparseMatrix<FloatType> const &measRowT,
            GliderVarioStatus &varioStatus
    );

    /** \brief Calculate the status update for three measurement values at once
     *
     * This is the generic part which deals with calculating the Kalman gain, and updates the state estimate and the status covariance.
     * This routine requires some preparation by the specific update routines which apply the (usually non-linear) measurement function
     * to the latest state estimate to calculate the expected measurement value as well as the Jacobian of the measurement matrix.
     *
     * This function is used to correct the status simultaneouslz with threee measurements which are typical for 3-axis sensors.
     *
     * @param[in] measuredValue The measured values as they were measured by a sensor. These values may be heavily pre-processed, e.g. the IAS or TAS from the
     *   dynamic pressure, and static pressure, or altitude from the absolute pressure sensor
     * @param[in] calculatedValue The theoretical measurement values as they calculated from the current extrapolated system status.
     * @param[in] measurementVariance_R The variance of the measure. I assume the measurement variances are independent, i.e. the measurement covariance matrix is diagonal.
     *   This assumption is particularly important for the simplifications of the calculation of the inverse
     * @param[in] measRowT The transposed measurement matrix. It is calculated or approximated as the Jacobian partial derivates each time.
     *   The matrix row is not used to calculate the theoretical measured value. This is already done by the calling function by directly using the not-linear
     *   measurement function and passing the result as \p calculatedValue.
     *   The transposed Jacobian is used here to calculate the Kalman gain for this measurement, and applying it to the updated status and covariance.
     *   The matrix is passed in the transposed form because in the calculation it is used transposed twice, and only once in the original form. So I have to
     *   transpose only once (back to the original form).
     * @param[in,out] varioStatus The system status and covariance before and after the measurement update. The status and covariance are directly updated from the difference of the actually measured, and the
     * theoretical value.
     */
    OEV_LOCAL static void calc3DMeasureUpdate (
            Vector3DType const &measuredValue,
			Vector3DType const &calculatedValue,
			RotationMatrix3DType const &measurementVariance_R,
            Eigen::SparseMatrix<FloatType> const &measRowT,
            GliderVarioStatus &varioStatus
    );

    /// Indicator if calculation results are to be copied into global variables for comparison by unit tests.
    OEV_LOCAL static bool unitTestMode;
};

} /* namespace openEV */

#endif /* GLIDERVARIOMEASUREMENTUPDATER_H_ */
