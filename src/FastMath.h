/*
 * FastMath.h
 *
 *  Created on: Dec 23, 2015
 *      Author: hor
 *
 *  Definition of class FastMath
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


protected:

  /// The table of per-computed sinus values. The table is one item longer than sizeSineTable because I need the interpolation to +360 degrees!
  static const double sinusTable [sizeSineTable+1];

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
