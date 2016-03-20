/*
 * RotMatrixConversion.h
 *
 *  Created on: Mar 20, 2016
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

#ifndef ROTMATRIXCONVERSION_H_
#define ROTMATRIXCONVERSION_H_

#include <math.h>
#include <float.h>
#include "GliderVarioStatus.h"

namespace openEV {

/**
 *
 * The algorithm to construct a rotation matrix from two vectors comes from:
 * http://stackoverflow.com/questions/23166898/efficient-way-to-calculate-a-3x3-rotation-matrix-from-the-rotation-defined-by-tw
 *
 * The algorithm to decompose a rotation matrix into the Euler angles comes from
 * http://nghiaho.com/?page_id=846
 */
class RotMatrixConversion {
public:
	RotMatrixConversion() {
		// TODO Auto-generated constructor stub

	}
	virtual ~RotMatrixConversion();

/**
 * Calculate a rotation matrix from 2 normalized vectors.
 * v1 and v2 must be unit length.
 *
 * @param[out] rotMatrix The rotation matrix to map v1 to v2
 * @param[in] v1: the original vector
 * @param[in] v2: the resulting vector after being multiplied with rotMatrix.
 */
static void vectors2RotMatrix(RotationMatrix3DType &rotMatrix, Vector3DType const &v1, Vector3DType const &v2);

private:

/**
 * Determines the dominant axis in the vector, i.e. the dimension with the greatest length
 * @param[in] vec
 * @return 0: x-axis, 1: y-axis, 2: y-axis
 */
static int axisDominantV3Single(Vector3DType const &vec);

/**
 * Calculate the orthogonal
 * @param[out] p: The orthogonal vector to v
 * @param[in] v: Input.
 */
static void orthoV3V3(Vector3DType &p, Vector3DType const &v);

/**
 * Does the basic calculation of the rotation matrix.
 * @param[out] mat Due to a gcc 5.3 bug I cannot pass mat as a reference but I have to pass the pointer.
 * @param[in] axis
 * @param[in] angle_sin
 * @param[in] angle_cos
 */
static void axisAngleNormalizedToMat3Ex(
        RotationMatrix3DType  *mat, Vector3DType const &axis,
        const FloatType angle_sin, const FloatType angle_cos);

};


} /* namespace openEV */

#endif /* ROTMATRIXCONVERSION_H_ */
