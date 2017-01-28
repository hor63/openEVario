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

#include "gtest/gtest.h"
#include "GliderVarioStatus.h"


using namespace openEV;

class GliderVarioStatusTest :public ::testing::Test {
public:


	GliderVarioStatus statusVector;
};

TEST_F(GliderVarioStatusTest, InitTest) {

	// verify that all elements of the status vector are initialized to 0 except selected components
	for (int i = 0; i < statusVector.STATUS_NUM_ROWS; i++){
		switch (i) {
		case statusVector.STATUS_IND_GRAVITY:
		 EXPECT_EQ (statusVector.getStatusVector_x()(i),::GRAVITY) << "Status vector (STATUS_IND_GRAVITY) is not ::GRAVITY = " << ::GRAVITY;
		 break;
		case statusVector.STATUS_IND_LAST_PRESSURE:
		 EXPECT_EQ (statusVector.getStatusVector_x()(i),::pressureStdMSL) << "Status vector (STATUS_IND_LAST_PRESSURE) is not ::pressureStdMSL = " << ::pressureStdMSL;
		 break;
		default:
			EXPECT_EQ (statusVector.getStatusVector_x()(i),0.0f) << "Status vector ("<<i<<") is not 0.0";
		}
	}

}

TEST_F(GliderVarioStatusTest, Accessor) {

	// Set all components to different values
	for (int i = 0; i < statusVector.STATUS_NUM_ROWS; i++){
		statusVector.getStatusVector_x()(i) = FloatType(i);
	}
#error Complete this test with actual accessors
}
