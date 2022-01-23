/*
 * \brief Implementation of class \see RotationMatrix
 *
 *  Created on: Dec 22, 2015
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
#  include "config.h"
#endif

#include "util/RotationMatrix.h"
#include "util/FastMath.h"

namespace openEV
{


RotationMatrix::~RotationMatrix ()
{

}

void
RotationMatrix::calculateRotationMatrixGloToPlane ()
{

    if (matrixGloToPlaneIsValid) {
        return;
    }

    // Calculate the sin and cos values from the angles beforehand because they occur multiple times in the matrix.
    FloatType sinYaw	= FastMath::fastSin(yaw);
    FloatType cosYaw	= FastMath::fastCos(yaw);
    FloatType sinPitch	= FastMath::fastSin(pitch);
    FloatType cosPitch	= FastMath::fastCos(pitch);
    FloatType sinRoll	= FastMath::fastSin(roll);
    FloatType cosRoll	= FastMath::fastCos(roll);


    matrixGloToPlane(0,0) = cosPitch * cosYaw;
    matrixGloToPlane(0,1) = cosPitch * sinYaw;
    matrixGloToPlane(0,2) = -sinPitch;

    matrixGloToPlane(1,0) = sinRoll * sinPitch * cosYaw - cosRoll * sinYaw;
    matrixGloToPlane(1,1) = sinRoll * sinPitch * sinYaw + cosRoll * cosYaw;
    matrixGloToPlane(1,2) = sinRoll * cosPitch;

    matrixGloToPlane(2,0) = cosRoll * sinPitch * cosYaw + sinRoll * sinYaw;
    matrixGloToPlane(2,1) = cosRoll * sinPitch * sinYaw - sinRoll * cosYaw;
    matrixGloToPlane(2,2) = cosRoll * cosPitch;

    matrixGloToPlaneIsValid = true;
    matrixPlaneToGloIsValid = false;

}

} // namespace openEV
