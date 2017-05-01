/*
 * FastMath.h
 *
 *  Created on: Dec 23, 2015
 *      Author: hor
 *
 *  Definition of class FastMath
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

#ifndef FASTMATH_H_
#define FASTMATH_H_

#include <math.h>
#include <assert.h>

#include "GliderVarioStatus.h"

// directly from GLIBC math.h ... just in case
#if !defined(M_PI)
# define M_PI           3.14159265358979323846  /* pi */
#endif



namespace openEV
{

/**
 * FastMath
 *
 * Faster implementations of CPU and time intensive functions, particular trigonometric functions.
 * For a Kalman filter the last bit of accuracy is not required. That is what the process (co)variance is for (within other inaccuracies :) ).
 *
 * All trigonometric functions here are used in degrees (0-360 deg)!
 */
class FastMath
{
public:

    static constexpr unsigned sineSamplesPerDegree = 8;   ///< the sinus table is calculated in 1/8 degree steps
    static constexpr unsigned sizeSineTable = 360*sineSamplesPerDegree;   ///< the sinus table is calculated in 1/8 degree steps
    static constexpr unsigned sizeATanTable = 256; ///< the arc tan table is defined for the 1st 45 degrees in 256 steps.
    static constexpr unsigned sizeASinTable = 512; ///< the arc sin table is defined for the 1st 90 degrees in 512 steps.
    static constexpr double   radToDeg = 180.0 / M_PI;
    static constexpr double   degToRad = M_PI / 180.0;

    FastMath ();
    virtual
    ~FastMath ();


    /**
     *
     * @param[in] angle in degrees.
     * @return The sine value of the angle
     */
    static inline FloatType fastSin(FloatType angle) {

        if (angle < 0.0) {
            return -fastSinPositive(-angle);
        } else {
            return fastSinPositive(angle);
        }

    }

    /**
     *
     * @param[in] angle in degrees
     * @return The cosine value of the angle
     */
    static inline FloatType fastCos(FloatType angle) {
        return fastSin(angle + 90.0);
    }

    /**
     *
     * Calculates the arc tan angle for the x and y component in Cartesian coordinates.
     * Based on the signs of x and y the function returns angles from the entire circle
     * The returned angle is in degrees from 0 to 360 degrees.
     *
     * Please note that x and y here are counted as in mathematics on the unit circle.
     * Meaning x to the left, y up. Angle from x axis counterclockwise to the y axis.
     * If you wonder how that fits with the world coordinate system which I am using, remember that the z axis points *down* not against you
     * like in the unit circle
     *
     * \sa <a href="https://en.wikipedia.org/wiki/Unit_circle" >Wikipedia: Unit circle</a>
     *
     * @param[in] x component
     * @param[in] y component
     * @return Angle in degrees 0-360 deg.
     */
    static inline FloatType fastATan2 (FloatType x, FloatType y) {

        if (x >= 0) {
            if (y >= 0) {
                // first quadrant
                return (fastATan2Pos(x,y));
            } else {
                // I am in the second quadrant
                return (180-fastATan2Pos(x,-y));
            }
        } else {
            if (y >= 0) {
                // I am in the fourth quadrant
                return (360-fastATan2Pos(-x,y));
            } else {
                // I am in the third quadrant
                return (180+fastATan2Pos(-x,-y));
            }
        }
    }

    /**
     *
     * @param y The ratio of the opposite (German Gegenkathete) to hypotenuse. Precondition is -1.0 <= x <= 1.0
     * @return The angle in Degrees where the opposite on a unit circle is y. Return range is -90 to +90 degrees.
     *
     * \sa <a href="https://en.wikipedia.org/wiki/Sine">Wikipedia: Sine</a> or
     * <a href="https://de.wikipedia.org/wiki/Sinus_und_Kosinus">Sinus und Cosinus</a>
     * \sa <a href="https://en.wikipedia.org/wiki/Unit_circle" >Unit Circle</a>
     *
     */
    static inline FloatType fastASin(FloatType y) {
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

    /**
     *
     * @param x The ratio of the adjacent (German Ankathete) to hypotenuse. Precondition is -1.0 <= x <= 1.0
     * @return The angle in Degrees where the opposite on a unit circle is x. Return range is 0.0 to 180.0 degrees.
     *
     * \sa <a href="https://en.wikipedia.org/wiki/Sine">Wikipedia: Sine</a> or
     * <a href="https://de.wikipedia.org/wiki/Sinus_und_Kosinus">Sinus und Cosinus</a>
     * \sa <a href="https://en.wikipedia.org/wiki/Unit_circle" >Unit Circle</a>
     *
     */
    static inline FloatType fastACos(FloatType x) {
        assert (-1.0f <= x && x <= 1.0f);

        return (90.0f - fastASin (x));
    }

protected:

    /// The table of pre-computed sine values. The table is one item longer than sizeSineTable because I need the interpolation to +360 degrees!
    static const double sinusTable [sizeSineTable+1];

    /// The table of pre-computed arc tangent values from 0 to 45 deg. Anything else is derived from this range.
    /// Here 2 larger than the number of increments: including 0, all 256 steps in between, and 1
    static const double atanTable [sizeATanTable+1];

    /// The table of pre-computed arc sine values from 0 to 90 deg. Anything else is derived from this range.
    /// Here 2 larger than the number of increments: including 0, all 512 steps in between, and 1
    static const double asinTable [sizeASinTable+1];

    /**
     *
     * @param[in] angle in degrees. The angle *must* >= 0.0 and < 360.0
     * @return The sine value of the angle
     */
    static inline FloatType fastSinRaw(FloatType angle) {
        double scaledAngle = angle*static_cast<double>(sineSamplesPerDegree); // The angle scaled to the sinusTable scale
        double indexD = trunc(scaledAngle);
        unsigned index = static_cast<unsigned>(indexD);

        assert (angle>=0.0 && angle <360.0);


        // return the interpolated value
        return (sinusTable[index] + (sinusTable[index+1]-sinusTable[index])*(scaledAngle-indexD));

    }

    /**
     * Calculates the arc tangent from the x and y component of Cartesian coordinates within the first quadrant, i.e. x and y must >= 0
     * For directions of x and y, and direction of the angle see \ref fastATan2
     *
     * @param[in] x x-component of a point in Cartesian coordinates
     * @param[in] y x-component of a point in Cartesian coordinates
     * @return the arc tangent of the ratio of x and y
     */
    static inline FloatType fastATan2Pos (FloatType x, FloatType y) {

        assert (x>=0 && y >= 0);

        if (x < y) {
            // the angle is between 0 and 45 deg.
            return (fastATanRaw(x/y));
        } else {
            if (x > y) {
                // I am between 45 and 90 deg. Return from the second half of the quadrant
                return (90-fastATanRaw(y/x));
            } else {

                if (x == 0) {
                    // x and y are 0! This is actually nonsense. Before I enter a division by 0 I just return 0 :)
                    return 0.0;
                } else {
                    // x and y are identical. Therefore the angle is 45 deg.
                    return 45.0;
                }
            }
        }
    }

    /**
     *
     * Calculate the arc tangent value of a value between 0 and < 1. This function interpolates the pre-calculated values from the table atanTable.
     * Due to the range of the input values only the first octant can be calculated. Everything must be mirrored from this partial range.
     * @param[in] tanVal: tan value, i.e. the ratio of x and y. tan value *must* be >= 0 and < 1. This function is only defined in the 1st 45 degrees.
     * @return the arc tan value in degrees.
     */
    static inline FloatType fastATanRaw (FloatType tanVal) {
        double scaledTanVal = tanVal * static_cast<double>(sizeATanTable);
        double indexD = trunc(scaledTanVal);
        unsigned index = static_cast<unsigned>(indexD);

        assert (tanVal >= 0.0 && tanVal < 1.0);

        // return the interpolated value.
        return (atanTable[index] + (atanTable[index+1]-atanTable[index])*(scaledTanVal-indexD));
    }


    /**
     *
     * @param[in] angle in degrees. The angle MUST be >= 0.
     * @return The sine value of the angle
     */
    static inline FloatType fastSinPositive(FloatType angle) {

        // normalize values if needed
        if (angle >= 360.0) {
            // this is expensive, so do not do it :)
            return fastSinRaw (fmod(angle,360.0));
        } else {
            // normalization not required
            return fastSinRaw (angle);
        }

    }

};

} // namespace openEV

#endif /* FASTMATH_H_ */
