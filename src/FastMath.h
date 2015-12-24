/**
 * \file FastMath.h
 *
 *  Created on: Dec 23, 2015
 *      Author: hor
 *
 *  Definition of class FastMath
 */

#ifndef FASTMATH_H_
#define FASTMATH_H_

#include <math.h>

#include "GliderVarioStatus.h"

namespace openEV
{

/**
 * \class FastMath
 *
 * Faster implementations of CPU and time intensive functions, particular trigonometric functions.
 * For a Kalman filter the last bit of accuracy is not required. That is what the process (co)variance is for (within other inaccuracies.
 */
class FastMath
{
public:
  FastMath ();
  virtual
  ~FastMath ();

  /**
   *
   * @param angle[in] in degrees
   * @return The sine value of the angle
   */
  static inline FloatType fastSin(FloatType angle) {
    return static_cast<FloatType>(sin(M_PI / 180.0 * angle));
  }

  /**
   *
   * @param angle[in] in degrees
   * @return The cosine value of the angle
   */
  static inline FloatType fastCos(FloatType angle) {
    return static_cast<FloatType>(cos(M_PI / 180.0 * angle));
  }

};

} // namespace openEV

#endif /* FASTMATH_H_ */
