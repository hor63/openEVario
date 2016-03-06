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
  static constexpr double   radToDeg = 180.0 / M_PI;
  static constexpr double   degToRad = M_PI / 180.0;

  FastMath ();
  virtual
  ~FastMath ();


  /**
   *
   * @param angle[in] in degrees.
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
   * @param angle[in] in degrees
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
   * @param[in] x component
   * @param[in] y component
   * @return Angle in degrees 0-360 deg.
   */
  static inline FloatType fastATan2 (FloatType y, FloatType x) {

	  if (y >= 0) {
		  if (x >= 0) {
			  // first quadrant
			  return (fastATan2Pos(y,x));
		  } else {
			  // I am in the second quadrant
			  return (180-fastATan2Pos(y,-x));
		  }
	  } else {
		  if (x >= 0) {
			  // I am in the fourth quadrant
			  return (360-fastATan2Pos(-y,x));
		  } else {
			  // I am in the third quadrant
			  return (180+fastATan2Pos(-y,-x));
		  }
	  }
  }


protected:

  /// The table of pre-computed sine values. The table is one item longer than sizeSineTable because I need the interpolation to +360 degrees!
  static const double sinusTable [sizeSineTable+1];

  /// The table of pre-computed arc sine values from 0 to 45 deg. Anything else is derived from this range.
  /// Here 2 larger than the number of increments: including 0, all 256 steps in between, and 1
  static const double atanTable [sizeATanTable+1];

  /**
   *
   * @param angle[in] in degrees. The angle *must* >= 0.0 and < 360.0
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
   * @param[in] x x-component of a point in Cartesian coordinates
   * @param[in] y x-component of a point in Cartesian coordinates
   * @return the arc tangent of the ratio of x and y
   */
  static inline FloatType fastATan2Pos (FloatType y, FloatType x) {

	  assert (y>=0 && x >= 0);

	  if (y < x) {
		  // the angle is between 0 and 45 deg.
		  return (fastATanRaw(y/x));
	  } else {
		  if (y > x) {
			  // I am between 45 and 90 deg. Return from the second half of the quadrant
			  return (90-fastATanRaw(x/y));
		  } else {
			  if (y == 0) {
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
   * @param angle[in] in degrees. The angle MUST be >= 0.
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
