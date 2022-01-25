/*
 * FastMath.cpp
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

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <cmath>

#include "util/FastMath.h"

namespace openEV
{

FastMath::FastMath ()
{

}

FastMath::~FastMath ()
{

}

FloatType FastMath::fastASin(FloatType y) {
    assert (-1.0f <= y && y <= 1.0f);

    if (y < 0.0f) {
        return -fastASin(-y);
    }
    if (y == 0.0f) {
        return 0.0f;
    }
    if (y == 1.0f) {
        return 90.0f;
    }

    // Calculate the result interpolating the table.
    {
        double scaledASinVal = y * static_cast<double>(sizeASinTable);
        double indexD = trunc(scaledASinVal);
        unsigned index = static_cast<unsigned>(indexD);

        assert (y >= 0.0 && y < 1.0);

        // return the interpolated value.
        return (asinTable[index] + (asinTable[index+1]-asinTable[index])*(scaledASinVal-indexD));
    }
}

FloatType FastMath::fastSinRaw(FloatType angle) {
    double scaledAngle = angle*static_cast<double>(sineSamplesPerDegree); // The angle scaled to the sinusTable scale
    double indexD = trunc(scaledAngle);
    unsigned index = static_cast<unsigned>(indexD);

    assert (angle>=0.0 && angle <360.0);


    // return the interpolated value
    return (sinusTable[index] + (sinusTable[index+1]-sinusTable[index])*(scaledAngle-indexD));

}

FloatType FastMath::fastATanRaw (FloatType tanVal) {
    double scaledTanVal = tanVal * static_cast<double>(sizeATanTable);
    double indexD = trunc(scaledTanVal);
    unsigned index = static_cast<unsigned>(indexD);

    assert (tanVal >= 0.0 && tanVal < 1.0);

    // return the interpolated value.
    return (atanTable[index] + (atanTable[index+1]-atanTable[index])*(scaledTanVal-indexD));
}

FloatType FastMath::fastSinPositive(FloatType angle) {

    // normalize values if needed
    if (angle >= 360.0) {
        // this is expensive, so do not do it :)
        return fastSinRaw (fmod(angle,360.0));
    } else {
        // normalization not required
        return fastSinRaw (angle);
    }

}

} // namespace openEV
