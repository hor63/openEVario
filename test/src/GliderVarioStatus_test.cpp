/*
 * GliderVarioStatus_test.cpp
 *
 *  Created on: Dec 8, 2015
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

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "gtest/gtest.h"
#include "kalman/GliderVarioStatus.h"


using namespace openEV;

class GliderVarioStatusTest :public ::testing::Test {
public:


    GliderVarioStatus statusVector;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(GliderVarioStatusTest, InitTest) {

    // verify that all elements of the status vector are initialized to 0 except selected components
    for (int i = 0; i < statusVector.STATUS_NUM_ROWS; i++){
        switch (i) {
        case GliderVarioStatus::STATUS_IND_GRAVITY:
        EXPECT_EQ (statusVector.getStatusVector_x()(i),::GRAVITY) << "Status vector (STATUS_IND_GRAVITY) is not ::GRAVITY = " << ::GRAVITY;
        break;
        case GliderVarioStatus::STATUS_IND_LAST_PRESSURE:
        EXPECT_EQ (statusVector.getStatusVector_x()(i),::pressureStdMSL) << "Status vector (STATUS_IND_LAST_PRESSURE) is not ::pressureStdMSL = " << ::pressureStdMSL;
        break;
        default:
            EXPECT_EQ (statusVector.getStatusVector_x()(i),0.0f) << "Status vector ("<<i<<") is not 0.0";
        }
    }

}

TEST_F(GliderVarioStatusTest, AccessorTest) {
    int i;
    // Set all components to different values
    for (i = 0; i < statusVector.STATUS_NUM_ROWS; i++){
        statusVector.getStatusVector_x()(i) = FloatType(i);
    }

    // count the accessor tests. At the end I want to test ALL accessors.
    i=0;

    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_GRAVITY), statusVector.gravity);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_LATITUDE_OFFS), statusVector.latitudeOffsC);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_LONGITUDE_OFFS), statusVector.longitudeOffsC);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_ALT_MSL), statusVector.altMSL);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_HEADING), statusVector.heading);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_PITCH), statusVector.pitchAngle);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_ROLL), statusVector.rollAngle);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_SPEED_GROUND_N), statusVector.groundSpeedNorth);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_SPEED_GROUND_E), statusVector.groundSpeedEast);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_TAS), statusVector.trueAirSpeed);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_RATE_OF_SINK), statusVector.rateOfSink);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_VERTICAL_SPEED), statusVector.verticalSpeed);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_THERMAL_SPEED), statusVector.thermalSpeed);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_ACC_HEADING), statusVector.accelHeading);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_ACC_CROSS), statusVector.accelCross);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_ACC_VERTICAL), statusVector.accelVertical);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_ROTATION_X), statusVector.rollRateX);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_ROTATION_Y), statusVector.pitchRateY);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_ROTATION_Z), statusVector.yawRateZ);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_GYRO_BIAS_X), statusVector.gyroBiasX);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_GYRO_BIAS_Y), statusVector.gyroBiasY);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_GYRO_BIAS_Z), statusVector.gyroBiasZ);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_MAGNETIC_DECLINATION), statusVector.magneticDeclination);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_MAGNETIC_INCLINATION), statusVector.magneticInclination);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_COMPASS_DEVIATION_X), statusVector.compassDeviationX);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_COMPASS_DEVIATION_Y), statusVector.compassDeviationY);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_COMPASS_DEVIATION_Z), statusVector.compassDeviationZ);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_WIND_SPEED_N), statusVector.windSpeedNorth);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_WIND_SPEED_E), statusVector.windSpeedEast);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_QFF), statusVector.qff);
    i++;
    EXPECT_EQ (statusVector.getStatusVector_x()( statusVector.STATUS_IND_LAST_PRESSURE), statusVector.lastPressure);
    i++;

    EXPECT_EQ (statusVector.STATUS_NUM_ROWS, i) << "Not all accessors have been tested.";


}
