/*
 * GliderVarioTransitionMatrix_test.cpp
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
#include "GliderVarioTransitionMatrix.h"

using namespace openEV;

class TransitionMatrixTest :public ::testing::Test {
public:


	openEV::GliderVarioTransitionMatrix transMatrix;
	openEV::GliderVarioStatus st1,st2;
};

TEST_F(TransitionMatrixTest, Gravity) {

	// Test the result for a given combination of input values
	// and a number of time differences
	// input values are: Gravity
	for (FloatType t = 0.01f; t<=1.0f; t+=0.11f  ) {
		for (FloatType g = 9.0f ; g <= 10.f; g+=0.01f) {
			FloatType tmp1;
			st1.gravity = g;

			transMatrix.updateStatus(st1,st2,t);

			// Now check the result
			EXPECT_EQ (st1.gravity,st2.gravity);

			// check the increment with the transition matrix
			tmp1 = st2.gravity + 0.01f * transMatrix.getTransitionMatrix().coeff(st1.STATUS_IND_GRAVITY,st1.STATUS_IND_GRAVITY);
			st1.gravity = g + 0.01f;
			transMatrix.updateStatus(st1,st2,t);
			EXPECT_EQ (tmp1,st2.gravity);


		}
	}

}

TEST_F(TransitionMatrixTest, Latitude) {

	// Test the result for a given combination of input values
	// and a number of time differences
	// input values are: Gravity
	for (FloatType t = 0.01f; t<=1.3f; t+=0.23f  ) {
		for (FloatType lat = 45.0f ; lat <= 65.0f; lat += 5.33f) {
			for (FloatType speedGroundN = 0.0f; speedGroundN <= 80.0f; speedGroundN += 6.77f) {
				for (FloatType accel = -0.5f ; accel <= 0.5f; accel += 0.193f) {
					for (FloatType heading = 0.0f ; heading < 360.0f; heading += 23.3f){
						st1.latitude = lat * 3600.0f;
						st1.groundSpeedNorth = speedGroundN;
						st1.accelHeading = accel;
						st1.heading = heading;

						transMatrix.updateStatus(st1,st2,t);

						FloatType arcSecPerM = 3600.0 / 111132.0;
						FloatType expectResult =
								lat * 3600.0f
								+ arcSecPerM * t*speedGroundN
								+ arcSecPerM * FastMath::fastCos(heading) * accel *t*t/2
								;

						EXPECT_NEAR (st2.latitude,expectResult,expectResult*0.000001) <<
								" at Latitude = " << lat << " groundSpeedN = " << speedGroundN << " acceleration = " << accel <<
								" heading = " << heading << " time = " << t;

						// Test the matrix factors
						FloatType orgResult = expectResult;
						FloatType resultDelta;
						FloatType deltaValue;


						// Modify the latitude
						deltaValue = 10.0f;
						st1.latitude = lat * 3600.0f + deltaValue;
						transMatrix.updateStatus(st1,st2,t);
						expectResult =
								(lat * 3600.0f + deltaValue)
								+ arcSecPerM * t*speedGroundN
								+ arcSecPerM * FastMath::fastCos(heading) * accel *t*t/2
								;
						resultDelta = deltaValue *
								transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_LATITUDE);

						EXPECT_NEAR (expectResult,orgResult + resultDelta,expectResult*0.000001) << " Latitude delta = " << deltaValue;
						st1.latitude = lat * 3600.0f;

						// Modify groundSpeedN
						deltaValue = 1.0f;
						st1.groundSpeedNorth = speedGroundN + deltaValue;
						transMatrix.updateStatus(st1,st2,t);
						expectResult =
								lat * 3600.0f
								+ arcSecPerM * t* (speedGroundN + deltaValue)
								+ arcSecPerM * FastMath::fastCos(heading) * accel *t*t/2
								;
						resultDelta = deltaValue *
								transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N);

						EXPECT_NEAR (expectResult,orgResult + resultDelta,expectResult*0.000001) <<
								" groundSpeedN delta = " << deltaValue <<
								" resultDelta = " << resultDelta;
						st1.groundSpeedNorth = speedGroundN;

						// Modify the acceleration
						deltaValue = 1.0f;
						st1.accelHeading = accel + deltaValue;
						transMatrix.updateStatus(st1,st2,t);
						expectResult =
								lat * 3600.0f
								+ arcSecPerM * t* speedGroundN
								+ arcSecPerM * FastMath::fastCos(heading) * (accel + deltaValue) *t*t/2
								;
						resultDelta = deltaValue *
								transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_HEADING);

						EXPECT_NEAR (expectResult,orgResult + resultDelta,expectResult*0.000001) <<
								" accelHeading delta = " << deltaValue <<
								" resultDelta = " << resultDelta;
						st1.accelHeading = accel;

						// Modify the heading
						deltaValue = 1.0f;
						st1.heading = heading + deltaValue;
						transMatrix.updateStatus(st1,st2,t);
						expectResult =
								lat * 3600.0f
								+ arcSecPerM * t*speedGroundN
								+ arcSecPerM * FastMath::fastCos(heading + deltaValue) * accel *t*t/2
								;
						resultDelta = deltaValue *
								transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_HEADING);

						EXPECT_NEAR (expectResult,orgResult + resultDelta,expectResult*0.000001) <<
								" heading delta = " << deltaValue <<
								" resultDelta = " << resultDelta <<
								" heading = " << st1.heading <<
								" acceleration = " << st1.accelHeading <<
								" time = " << t <<
								" Coefficient = " << transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_HEADING);
						st1.heading = heading;

					}
				}
			}
		}
	}


}

