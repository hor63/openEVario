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
	for (FloatType t = 0.01f; t<=1.0f; t+=0.1f  ) {
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

