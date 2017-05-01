/*
 * FastMath_test.cpp
 *
 *  Created on: Dec 23, 2015
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

#include "gtest/gtest.h"

#include <math.h>
#include <iostream>
#include "FastMath.h"

namespace openEV {

TEST(FastMath, Conversion_degToRad) {

    EXPECT_EQ(M_PI, 180.0 * FastMath::degToRad);
    EXPECT_EQ(2.0 * M_PI, 360.0 * FastMath::degToRad);

}

TEST(FastMath, Conversion_radToDeg) {

    EXPECT_EQ(180.0, M_PI * FastMath::radToDeg);
    EXPECT_EQ(360.0, 2 * M_PI * FastMath::radToDeg);

}

TEST(FastMath, fastSin) {

    for (double i = -36000.0; i <= 36000.0 ; i++) {
        EXPECT_LT(sin(i/10.0*FastMath::degToRad) - FastMath::fastSin(i/10.0),0.00001);
        EXPECT_GT(sin(i/10.0*FastMath::degToRad) - FastMath::fastSin(i/10.0),-0.00001);
    }
}

TEST(FastMath, fastCos) {

    for (double i = -36000.0; i <= 36000.0 ; i++) {
        EXPECT_LT(cos(i/10.0*FastMath::degToRad) - FastMath::fastCos(i/10.0),0.00001);
        EXPECT_GT(cos(i/10.0*FastMath::degToRad) - FastMath::fastCos(i/10.0),-0.00001);
    }
}

TEST(FastMath, fastASin) {
    double constexpr numSteps = 100.0;

    for (double i = -numSteps; i <= numSteps ; i++) {
        EXPECT_LT(asin(i/numSteps)*FastMath::radToDeg - FastMath::fastASin(i/numSteps),0.004);
        EXPECT_GT(asin(i/numSteps)*FastMath::radToDeg - FastMath::fastASin(i/numSteps),-0.004);
    }
}


TEST(FastMath, fastACos) {
    double constexpr numSteps = 100.0;

    for (double i = -numSteps; i <= numSteps ; i++) {
        EXPECT_LT(acos(i/numSteps)*FastMath::radToDeg - FastMath::fastACos(i/numSteps),0.004);
        EXPECT_GT(acos(i/numSteps)*FastMath::radToDeg - FastMath::fastACos(i/numSteps),-0.004);
    }
}

TEST(FastMath, fastATan2) {
    double constexpr numSteps = 100.0;

    for (double i = -numSteps; i <= numSteps ; i++) {
        for (double k = -numSteps; k <= numSteps ; k++) {
            double temp = atan2(i/numSteps,k/numSteps)*FastMath::radToDeg;
            // Standard atan2 returns a negative angle in the 3rd and 4th quadrant. FastMath return 0-360 deg.
            if (temp < 0.0) {
                temp += 360.0;
            }
            ASSERT_LT(temp - FastMath::fastATan2(i/numSteps,k/numSteps),0.001);
            ASSERT_GT(temp - FastMath::fastATan2(i/numSteps,k/numSteps),-0.001);
        }
    }
}

} // namespace openEV {


