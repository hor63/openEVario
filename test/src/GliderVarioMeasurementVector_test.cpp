/*
 * GliderVarioMeasurementVector_test.cpp
 *
 *  Created on: Jan 31, 2016
 *      Author: hor
 *
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
#include "kalman/GliderVarioMeasurementVector.h"

using namespace openEV;

class GliderVarioMeasurementVectorTest :public ::testing::Test {
public:


    openEV::GliderVarioMeasurementVector measVector;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(GliderVarioMeasurementVectorTest, InitTest) {

    // verify that all elements of the status vector are initialized to 0 except selected components
    for (int i = 0; i < measVector.MEASURE_NUM_ROWS; i++){
        EXPECT_EQ (measVector.getMeasureVector()(i),0.0f) << "Measurement vector ("<<i<<") is not 0.0";
    }

}

TEST_F(GliderVarioMeasurementVectorTest, AccessorTest) {
    int i;
    // Set all components to different values
    for (i = 0; i < measVector.MEASURE_NUM_ROWS; i++){
        measVector.getMeasureVector()(i) = FloatType(i);
    }

    // count the accessor tests. At the end I want to test ALL accessors.
    i=0;

    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_GPS_LAT), measVector.gpsLatitude);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_GPS_LON), measVector.gpsLongitude);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_GPS_ALT_MSL), measVector.gpsMSL);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_GPS_HEADING), measVector.gpsHeading);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_GPS_SPEED), measVector.gpsSpeed);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_ACC_X), measVector.accelX);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_ACC_Y), measVector.accelY);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_ACC_Z), measVector.accelZ);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_GYRO_RATE_X), measVector.gyroRateX);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_GYRO_RATE_Y), measVector.gyroRateY);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_GYRO_RATE_Z), measVector.gyroRateZ);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_MAG_X), measVector.magX);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_MAG_Y), measVector.magY);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_MAG_Z), measVector.magZ);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_STATIC_PRESSURE), measVector.staticPressure);
    i++;
    EXPECT_EQ (measVector.getMeasureVector()( measVector.MEASURE_IND_DYNAMIC_PRESSURE), measVector.dynamicPressure);
    i++;

    EXPECT_EQ (measVector.MEASURE_NUM_ROWS, i) << "Not all accessors have been tested.";

}
