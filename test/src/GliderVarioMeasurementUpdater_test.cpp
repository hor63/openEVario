/*
 * GliderVarioMeasurementMatrix_test.cpp
 *
 *  Created on: Feb 14, 2016
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016,2017  Kai Horstmann
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

#include "gtest/gtest.h"
#include "GliderVarioMeasurementUpdater.h"
#include "GliderVarioTransitionMatrix.h"
#include "FastMath.h"
#include "RotationMatrix.h"

using namespace openEV;

class MeasurementUpdaterTest :public ::testing::Test {
public:


    GliderVarioTransitionMatrix transMatrix;
    GliderVarioStatus st1,st2;
    GliderVarioMeasurementVector measVect;

    MeasurementUpdaterTest () {
        GliderVarioStatus::StatusCoVarianceType &errCov   = st1.getErrorCovariance_P();
        GliderVarioStatus::StatusCoVarianceType &noiseCov = st1.getSystemNoiseCovariance_Q();

        // Initialize the status and covariances


        // The noise covariance is set under the assumption that I run the status propagation in 100ms intervals,
        // i.e. all values per second are divided by 10, and values per minute divided by 600.
        // Mid of Lueneburg airport EDHG
        st1.latitude = 53.2483824 * 3600.0;
        // An arc minute =1.852km=1nm
        errCov.coeffRef(st1.STATUS_IND_LATITUDE,st1.STATUS_IND_LATITUDE) = 60.0f * 60.0f;
        // 1 arc sec per minute, i.e. about 100 meters
        noiseCov.coeffRef(st1.STATUS_IND_LATITUDE,st1.STATUS_IND_LATITUDE) = (1.0* 1.0) / 600.0;

        // Mid of Lueneburg airport EDHG
        st1.longitude = 10.458796 * 3600.0;
        // An arc minute at the latitude ~1km
        errCov.coeffRef(st1.STATUS_IND_LONGITUDE,st1.STATUS_IND_LONGITUDE) = 60.0f * 60.0f;
        // 1 arc sec per minute, i.e. about 60m meters
        noiseCov.coeffRef(st1.STATUS_IND_LONGITUDE,st1.STATUS_IND_LONGITUDE) = (1.0 * 1.0) / 600.0;

        st1.altMSL = 500.0;
        errCov.coeffRef(st1.STATUS_IND_ALT_MSL,st1.STATUS_IND_ALT_MSL) = 100.0 * 100.0;
        // 20m per minute
        noiseCov.coeffRef(st1.STATUS_IND_ALT_MSL,st1.STATUS_IND_ALT_MSL) = (20.0 * 20.0) / 600.0;

        // fly 120 deg course
        st1.heading = 120.0;
        errCov.coeffRef(st1.STATUS_IND_HEADING,st1.STATUS_IND_HEADING) = 60.0 * 60.0;
        // 30 deg per minute
        noiseCov.coeffRef(st1.STATUS_IND_HEADING,st1.STATUS_IND_HEADING) = (20.0 * 20.0) / 600.0;

        st1.pitchAngle = 0.0;
        errCov.coeffRef(st1.STATUS_IND_PITCH,st1.STATUS_IND_PITCH) = 10.0 * 10.0;
        // Pitch does not change so uncontrolled. The consequences would be quite noticeable otherwise :)
        noiseCov.coeffRef(st1.STATUS_IND_PITCH,st1.STATUS_IND_PITCH) = (5.0 * 5.0) / 600;

        st1.rollAngle = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ROLL,st1.STATUS_IND_ROLL) = 10.0 * 10.0;
        // Same with the roll angle
        noiseCov.coeffRef(st1.STATUS_IND_ROLL,st1.STATUS_IND_ROLL) = (10.0 * 10.0) / 600;

        // 30 m/s TAS at no wind
        st1.groundSpeedNorth = 30.0 * FastMath::fastCos(st1.heading);
        errCov.coeffRef(st1.STATUS_IND_SPEED_GROUND_N,st1.STATUS_IND_SPEED_GROUND_N) = 20.0 * 20.0;
        noiseCov.coeffRef(st1.STATUS_IND_SPEED_GROUND_N,st1.STATUS_IND_SPEED_GROUND_N) = (20.0 * 20.0) / 600.0;

        // 30 m/s TAS at no wind
        st1.groundSpeedEast = 30.0 * FastMath::fastSin(st1.heading);
        errCov.coeffRef(st1.STATUS_IND_SPEED_GROUND_E,st1.STATUS_IND_SPEED_GROUND_E) = 20.0 * 20.0;
        noiseCov.coeffRef(st1.STATUS_IND_SPEED_GROUND_E,st1.STATUS_IND_SPEED_GROUND_E) = (20.0 * 20.0) / 600.0;

        st1.trueAirSpeed = 30.0;
        errCov.coeffRef(st1.STATUS_IND_TAS,st1.STATUS_IND_TAS) = 20.0 * 20.0;
        noiseCov.coeffRef(st1.STATUS_IND_TAS,st1.STATUS_IND_TAS) = (20.0 * 20.0) / 600.0;

        st1.rateOfSink = 0.0;
        errCov.coeffRef(st1.STATUS_IND_RATE_OF_SINK,st1.STATUS_IND_RATE_OF_SINK) = 2.0 * 2.0;
        noiseCov.coeffRef(st1.STATUS_IND_RATE_OF_SINK,st1.STATUS_IND_RATE_OF_SINK) = (2.0 * 2.0) / 600.0;

        st1.verticalSpeed = -1.2;
        errCov.coeffRef(st1.STATUS_IND_VERTICAL_SPEED,st1.STATUS_IND_VERTICAL_SPEED) = 2.0 * 2.0;
        noiseCov.coeffRef(st1.STATUS_IND_VERTICAL_SPEED,st1.STATUS_IND_VERTICAL_SPEED) = (2.0 * 2.0) / 600.0;

        st1.thermalSpeed = -1.2;
        errCov.coeffRef(st1.STATUS_IND_THERMAL_SPEED,st1.STATUS_IND_THERMAL_SPEED) = 2.0 * 2.0;
        noiseCov.coeffRef(st1.STATUS_IND_THERMAL_SPEED,st1.STATUS_IND_THERMAL_SPEED) = (2.0 * 2.0) / 600.0;

        st1.accelHeading = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ACC_HEADING,st1.STATUS_IND_ACC_HEADING) = 1.0;
        noiseCov.coeffRef(st1.STATUS_IND_ACC_HEADING,st1.STATUS_IND_ACC_HEADING) = 1.0 / 600.0;

        st1.accelCross = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ACC_CROSS,st1.STATUS_IND_ACC_CROSS) = 1.0;
        noiseCov.coeffRef(st1.STATUS_IND_ACC_CROSS,st1.STATUS_IND_ACC_CROSS) = 1.0 / 600;

        st1.accelVertical = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ACC_VERTICAL,st1.STATUS_IND_ACC_VERTICAL) = 1.0;
        noiseCov.coeffRef(st1.STATUS_IND_ACC_VERTICAL,st1.STATUS_IND_ACC_VERTICAL) = 1.0 / 600.0;

        st1.rollRateX = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ROTATION_X,st1.STATUS_IND_ROTATION_X) = 20.0 * 20.0;
        noiseCov.coeffRef(st1.STATUS_IND_ROTATION_X,st1.STATUS_IND_ROTATION_X) = 20.0 * 20.0 / 600.0;

        st1.pitchRateY = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ROTATION_Y,st1.STATUS_IND_ROTATION_Y) = 10.0 * 10;
        noiseCov.coeffRef(st1.STATUS_IND_ROTATION_Y,st1.STATUS_IND_ROTATION_Y) = 10.0 * 10.0 / 600.0;

        // turning 1deg/s clock wise
        st1.yawRateZ = 1.0;
        errCov.coeffRef(st1.STATUS_IND_ROTATION_Z,st1.STATUS_IND_ROTATION_Z) = 20.0 * 20.0;
        noiseCov.coeffRef(st1.STATUS_IND_ROTATION_Z,st1.STATUS_IND_ROTATION_Z) = 20 * 20 / 600.0;

        st1.gyroBiasX = 0.0;
        errCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_X,st1.STATUS_IND_GYRO_BIAS_X) = 10.0 * 10.0;
        // Gyro bias should drift rather slow
        noiseCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_X,st1.STATUS_IND_GYRO_BIAS_X) = 1.0 / 600.0f;

        st1.gyroBiasY = 0.0;
        errCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_Y,st1.STATUS_IND_GYRO_BIAS_Y) = 10.0 * 10.0;
        // Gyro bias should drift rather slow
        noiseCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_Y,st1.STATUS_IND_GYRO_BIAS_Y) = 1.0 / 600.0f;

        st1.gyroBiasZ = 0.0;
        errCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_Z,st1.STATUS_IND_GYRO_BIAS_Z) = 10.0 * 10.0;
        // Gyro bias should drift rather slow
        noiseCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_Z,st1.STATUS_IND_GYRO_BIAS_Z) = 1.0 / 600.0f;

        st1.magneticDeclination = -2.0;
        errCov.coeffRef(st1.STATUS_IND_MAGNETIC_DECLINATION,st1.STATUS_IND_MAGNETIC_DECLINATION) = 4.0;
        noiseCov.coeffRef(st1.STATUS_IND_MAGNETIC_DECLINATION,st1.STATUS_IND_MAGNETIC_DECLINATION) = 0.2*0.2 / 600.0;

        st1.magneticInclination = MAG_INCLINATION;
        errCov.coeffRef(st1.STATUS_IND_MAGNETIC_INCLINATION,st1.STATUS_IND_MAGNETIC_INCLINATION) = 4.0f;
        noiseCov.coeffRef(st1.STATUS_IND_MAGNETIC_INCLINATION,st1.STATUS_IND_MAGNETIC_INCLINATION) = 0.2*0.2 / 600.0;

        st1.compassDeviationX = 0.0;
        // Assume Micro-Tesla, and a max. strength in the magnitute od=f the Earth magnet field
        errCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_X,st1.STATUS_IND_COMPASS_DEVIATION_X) = 50.0*50.0;
        // The deviation should fluctuate sloooowly.
        noiseCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_X,st1.STATUS_IND_COMPASS_DEVIATION_X) = 0.5*0.5 / 600.0f;

        st1.compassDeviationY = 0.0;
        // Assume Micro-Tesla, and a max. strength in the magnitute od=f the Earth magnet field
        errCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_Y,st1.STATUS_IND_COMPASS_DEVIATION_Y) = 50.0*50.0;
        // The deviation should fluctuate sloooowly.
        noiseCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_Y,st1.STATUS_IND_COMPASS_DEVIATION_Y) = 0.5*0.5 / 600.0f;

        st1.compassDeviationZ = 0.0;
        // Assume Micro-Tesla, and a max. strength in the magnitute od=f the Earth magnet field
        errCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_Z,st1.STATUS_IND_COMPASS_DEVIATION_Z) = 50.0*50.0;
        // The deviation should fluctuate sloooowly.
        noiseCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_Z,st1.STATUS_IND_COMPASS_DEVIATION_Z) = 0.5*0.5 / 600.0f;

        st1.windSpeedNorth = 0.0;
        errCov.coeffRef(st1.STATUS_IND_WIND_SPEED_N,st1.STATUS_IND_WIND_SPEED_N) = 20.0*20.0;
        // I am not aiming to capture the slightest gusts, but be responsive, 1 m/s per 10sec
        noiseCov.coeffRef(st1.STATUS_IND_WIND_SPEED_N,st1.STATUS_IND_WIND_SPEED_N) = 2.0*2.0 / 100.0;

        st1.windSpeedEast = 0.0;
        errCov.coeffRef(st1.STATUS_IND_WIND_SPEED_E,st1.STATUS_IND_WIND_SPEED_E) = 20.0*20.0;
        // I am not aiming to capture the slightest gusts, but be responsive, 1 m/s per 10sec
        noiseCov.coeffRef(st1.STATUS_IND_WIND_SPEED_E,st1.STATUS_IND_WIND_SPEED_E) = 2.0*2.0 / 100.0;

        st1.qff = 101000.0;
        errCov.coeffRef(st1.STATUS_IND_QFF,st1.STATUS_IND_QFF) = 40.0*40.0;
        // the pressure changes sloooowly
        noiseCov.coeffRef(st1.STATUS_IND_QFF,st1.STATUS_IND_QFF) = 0.1*0.1 / 600.0f;

        st1.lastPressure = st1.qff - st1.altMSL / (8.0/100.0);
        // Variance remains constant. This is only a fudge variable which is not going into any other
        // status propagation calculation.
        errCov.coeffRef(st1.STATUS_IND_LAST_PRESSURE,st1.STATUS_IND_LAST_PRESSURE) = 1.0f;
        // Therefore There is no noise. Otherwise the variance woud increase indefinitely.
        noiseCov.coeffRef(st1.STATUS_IND_LAST_PRESSURE,st1.STATUS_IND_LAST_PRESSURE) = 0.0f;


        // Now run this initial model for 10 sec. at 0.1 sec. interval.
        for (int i = 0; i<50; i++) {
            transMatrix.updateStatus(st1,st2,0.1f);
            transMatrix.updateStatus(st2,st1,0.1f);
        }

    }
};

TEST_F(MeasurementUpdaterTest, Latitude) {

    // Test the result for a given combination of input values
    // and a number of time differences

    FloatType measLat = st1.latitude/FloatType(3600.0) + FloatType(5/3600.0); // Increase by 5 arc seconds.
                                                                                //Remember measurement is in degrees, status in arc seconds
    FloatType expectResult = st1.latitude;

    GliderVarioMeasurementUpdater::GPSLatitudeUpd(measLat,15.0*15.0/3600.0/3600.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_LATITUDE:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),1.0f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

TEST_F(MeasurementUpdaterTest, Longitude) {

    // Test the result for a given combination of input values
    // and a number of time differences

    FloatType measLon = st1.longitude/FloatType(3600.0) + FloatType(5/3600.0); // Increase by 5 arc seconds.
                                                                                //Remember measurement is in degrees, status in arc seconds
    FloatType expectResult = st1.longitude;

    GliderVarioMeasurementUpdater::GPSLongitudeUpd(measLon,15.0*15.0/3600.0/3600.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_LONGITUDE:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),1.0f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

TEST_F(MeasurementUpdaterTest, GPSAltitude) {

    // Test the result for a given combination of input values
    // and a number of time differences

    FloatType measAlt = st1.altMSL + 10; // Increase by 10 m.

    FloatType expectResult = st1.altMSL;

    GliderVarioMeasurementUpdater::GPSAltitudeUpd(measAlt,25.0*25.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ALT_MSL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),1.0f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

TEST_F(MeasurementUpdaterTest, GPSHeading) {

    // Test the result for a given combination of input values
    // and a number of time differences

    FloatType expectResult = FastMath::fastATan2(st1.groundSpeedEast,st1.groundSpeedNorth);

    FloatType measHeadingGrnd = expectResult + 10.0; // Increase by 10 degrees.

    // approximate derivatives
    FloatType delta = (fabs(st1.groundSpeedEast) + fabs(st1.groundSpeedNorth)) / 100.0;
    FloatType expectDiffNorth = (FastMath::fastATan2(st1.groundSpeedEast        ,st1.groundSpeedNorth + delta) - expectResult ) / delta;
    FloatType expectDiffEast  = (FastMath::fastATan2(st1.groundSpeedEast + delta,st1.groundSpeedNorth        ) - expectResult ) / delta;

    GliderVarioMeasurementUpdater::GPSHeadingUpd(measHeadingGrnd,10.0*10.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_SPEED_GROUND_N:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),expectDiffNorth);
            break;

        case GliderVarioStatus::STATUS_IND_SPEED_GROUND_E:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),expectDiffEast);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

TEST_F(MeasurementUpdaterTest, GPSSpeed) {

    // Test the result for a given combination of input values
    // and a number of time differences

    FloatType expectResult = sqrtf(st1.groundSpeedEast * st1.groundSpeedEast + st1.groundSpeedNorth * st1.groundSpeedNorth);

    FloatType measSpeedGrnd = expectResult + 10.0; // Increase by 10 m/s.

    // approximate derivatives
    FloatType delta = expectResult / 100.0;
    FloatType temp1 =  st1.groundSpeedNorth + delta;
    FloatType expectDiffNorth = (sqrtf(st1.groundSpeedEast * st1.groundSpeedEast + temp1 * temp1) - expectResult ) / delta;
    temp1 =  st1.groundSpeedEast + delta;
    FloatType expectDiffEast = (sqrtf(temp1 * temp1 + st1.groundSpeedNorth * st1.groundSpeedNorth) - expectResult ) / delta;

    GliderVarioMeasurementUpdater::GPSSpeedUpd(measSpeedGrnd,5.0*5.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_SPEED_GROUND_N:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),expectDiffNorth);
            break;

        case GliderVarioStatus::STATUS_IND_SPEED_GROUND_E:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),expectDiffEast);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

