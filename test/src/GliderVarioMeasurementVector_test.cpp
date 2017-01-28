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

#include "gtest/gtest.h"
#include "GliderVarioMeasurementVector.h"

using namespace openEV;

class GliderVarioMeasurementVectorTest :public ::testing::Test {
public:


	openEV::GliderVarioMeasurementVector measVector;
};

TEST_F(GliderVarioMeasurementVectorTest, InitTest) {

	// verify that all elements of the status vector are initialized to 0 except selected components
	for (int i = 0; i < measVector.MEASURE_NUM_ROWS; i++){
		EXPECT_EQ (measVector.getMeasureVector()(i),0.0f) << "Measurement vector ("<<i<<") is not 0.0";
	}

}
