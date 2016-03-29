/*
 * RotMatrixConversion.cpp
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

#include <math.h>

#include "RotMatrixConversion.h"
#include "FastMath.h"

namespace openEV {

RotMatrixConversion::~RotMatrixConversion() {
	// TODO Auto-generated destructor stub
}



/* -------------------------------------------------------------------- */
/* Math Lib declarations */


/*
 */

/**
 * @todo: This stuff is not yet working! The resulting rotation matrix does not even closely resemble the original rotation matrix.
 */
void RotMatrixConversion::vectors2RotMatrix(Vector3DType const &v1, Vector3DType const &v2, RotationMatrix3DType &rotMatrix)
{
    Vector3DType axis;
    /* avoid calculating the angle */
    FloatType angle_sin;
    FloatType angle_cos;

    axis = v1.cross(v2);
    //cross_v3_v3v3(axis, v1, v2);

    angle_sin = axis.norm();
    axis /= angle_sin;

    if (angle_sin > FLT_EPSILON) {
		axisAngleNormalizedToMat3Ex(&rotMatrix,axis,angle_sin,angle_cos);
    }
    else {
        /* Degenerate (co-linear) vectors */
        if (angle_cos > 0.0f) {
            /* Same vectors, zero rotation... */
        	rotMatrix.setIdentity();
        }
        else {
            /* Colinear but opposed vectors, 180 rotation... */
            orthoV3V3(axis, v1);
            axis.normalize();
            angle_sin =  0.0f;  /* sin(M_PI) */
            angle_cos = -1.0f;  /* cos(M_PI) */
    		axisAngleNormalizedToMat3Ex(&rotMatrix,axis,angle_sin,angle_cos);
        }
    }
}

void RotMatrixConversion::rotMatrix2RotVector (RotationMatrix3DType const &rotMatrix, FloatType &yaw, FloatType &pitch, FloatType &roll) {
	yaw   = FastMath::fastATan2(rotMatrix(1,0),rotMatrix(0,0));
	pitch = FastMath::fastATan2(-rotMatrix(2,0),sqrtf(rotMatrix(2,1)*rotMatrix(2,1) + rotMatrix(2,2)*rotMatrix(2,2)));
	roll  = FastMath::fastATan2(rotMatrix(2,1),rotMatrix(2,2));
}


/* -------------------------------------------------------------------- */
/* Math Lib */


int RotMatrixConversion::axisDominantV3Single(Vector3DType const &vec)
{
    const FloatType x = fabsf(static_cast<float>(vec[0]));
    const FloatType y = fabsf(static_cast<float>(vec[1]));
    const FloatType z = fabsf(static_cast<float>(vec[2]));
    return ((x > y) ?
           ((x > z) ? 0 : 2) :
           ((y > z) ? 1 : 2));
}

void RotMatrixConversion::orthoV3V3(Vector3DType &p, Vector3DType const &v)
{
    const int axis = axisDominantV3Single(v);

    switch (axis) {
        case 0:
            p[0] = -v[1] - v[2];
            p[1] =  v[0];
            p[2] =  v[0];
            break;
        case 1:
            p[0] =  v[1];
            p[1] = -v[0] - v[2];
            p[2] =  v[1];
            break;
        case 2:
            p[0] =  v[2];
            p[1] =  v[2];
            p[2] = -v[0] - v[1];
            break;
    }
}

/* axis must be unit length */
void RotMatrixConversion::axisAngleNormalizedToMat3Ex(
        RotationMatrix3DType  *mat, Vector3DType const &axis,
        const FloatType angle_sin, const FloatType angle_cos)
{
    FloatType nsi[3], ico;
    FloatType n_00, n_01, n_11, n_02, n_12, n_22;

    ico = (1.0f - angle_cos);
    nsi[0] = axis[0] * angle_sin;
    nsi[1] = axis[1] * angle_sin;
    nsi[2] = axis[2] * angle_sin;

    n_00 = (axis[0] * axis[0]) * ico;
    n_01 = (axis[0] * axis[1]) * ico;
    n_11 = (axis[1] * axis[1]) * ico;
    n_02 = (axis[0] * axis[2]) * ico;
    n_12 = (axis[1] * axis[2]) * ico;
    n_22 = (axis[2] * axis[2]) * ico;

    (*mat)(0,0) = n_00 + angle_cos;
    (*mat)(0,1) = n_01 + nsi[2];
    (*mat)(0,2) = n_02 - nsi[1];
    (*mat)(1,0) = n_01 - nsi[2];
    (*mat)(1,1) = n_11 + angle_cos;
    (*mat)(1,2) = n_12 + nsi[0];
    (*mat)(2,0) = n_02 + nsi[1];
    (*mat)(2,1) = n_12 - nsi[0];
    (*mat)(2,2) = n_22 + angle_cos;
}

} /* namespace openEV */
