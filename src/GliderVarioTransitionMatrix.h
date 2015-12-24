/**
 *  \file GliderVarioTransitionMatrix.h
 *
 *  Created on: Dec 8, 2015
 *      Author: openvario
 */

#ifndef GLIDERVARIOTRANSITIONMATRIX_H_
#define GLIDERVARIOTRANSITIONMATRIX_H_

#include "GliderVarioStatus.h"

namespace openEV
{

/**
 *  \class GliderVarioTransitionMatrix
 * This is the transition matrix implementation of the Kalman filter.
 * The transition matrix is re-calculated before every update step because it depends on the elapsed interval.
 */
class GliderVarioTransitionMatrix
{
public:
  typedef Eigen::Matrix<FloatType,GliderVarioStatus::NUM_ROWS,GliderVarioStatus::NUM_ROWS> TransitionMatrixType;


  GliderVarioTransitionMatrix () {
    // Set the matrix to 0. There will be a lot of 0s in the matrix.
    transitionMatrix.Zero();
  }
  virtual
  ~GliderVarioTransitionMatrix ();

  TransitionMatrixType&
  getTtransitionMatrix () {
    return transitionMatrix;
  }

  /**
   * Recalculates the transition matrix. Only active coefficients are recalculated. All other coefficients are supposed to be 0 as they were set at construction time.
   * @param timeDiff Time since last update in seconds.
   */
  void
  calcTransitionMatrix (FloatType timeDiff);

protected:
  TransitionMatrixType transitionMatrix;
};

} // namespace openEV

#endif /* GLIDERVARIOTRANSITIONMATRIX_H_ */
