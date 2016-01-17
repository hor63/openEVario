/*
 *  GliderVarioTransitionMatrix.h
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

#ifndef GLIDERVARIOTRANSITIONMATRIX_H_
#define GLIDERVARIOTRANSITIONMATRIX_H_

#include "GliderVarioStatus.h"

namespace openEV
{

/**
 * Constant of gravity acceleration.
 * exact values for Germany can be obtained from the German gravity base mesh
 * Deutsches Schweregrundnetz 1994 (DSGN 94)
 * \ref http://www.bkg.bund.de/nn_175464/SharedDocs/Download/DE-Dok/DSGN94-Punktbeschreibung-PDF-de,templateId=raw,property=publicationFile.pdf/DSGN94-Punktbeschreibung-PDF-de.pdf
 * The constant here is a rough average between Hamburg and Munich (I live in Norther Germany).
 * Since a Kalman filter is not exact numeric science any inaccuracy should be covered by the process variance.
 */
static FloatType constexpr GRAVITY = 9.81;


/**
 * This is the transition matrix implementation of the Kalman filter.
 * The transition matrix is re-calculated before every update step because it depends on the elapsed interval,
 * and on the current attitude (i.e. heading pitch and roll affect the TAS vs speed and course over ground).
 */
class GliderVarioTransitionMatrix
{
public:
  typedef Eigen::Matrix<FloatType,GliderVarioStatus::NUM_ROWS,GliderVarioStatus::NUM_ROWS> TransitionMatrixType;


  GliderVarioTransitionMatrix () {
    // Set the matrix to 0. There will be a lot of 0s in the matrix.
    transitionMatrix.setZero();
  }
  virtual
  ~GliderVarioTransitionMatrix ();

  TransitionMatrixType&
  getTransitionMatrix () {
    return transitionMatrix;
  }

  /**
   * Recalculates the transition matrix. Only active coefficients are recalculated. All other coefficients are supposed to be 0 as they were set at construction time.
   * @param[in] timeDiff Time since last update in seconds.
   */
  void
  calcTransitionMatrix (
      FloatType timeDiff,
      GliderVarioStatus const &lastStatus);


  /**
   * Extrapolates the newStatus from the oldStatus after timeDiff seconds.
   * internally recalculates the transition matrix.
   *
   * @param[in] oldStatus Last known status
   * @param[out] newStatus New status by extrapolation after timeDiff seconds
   * @param[in] timeDiff The time difference in seconds
   */
  inline void
  updateStatus (
		  GliderVarioStatus const &oldStatus ,
		  GliderVarioStatus &newStatus,
		  FloatType timeDiff
		  ){
	  calcTransitionMatrix(timeDiff,oldStatus);
	  newStatus.getStatusVector() = transitionMatrix * oldStatus.getStatusVector();
	  // correct for the gravity
	  newStatus.verticalSpeed += openEV::GRAVITY*0.1;
	  newStatus.altMSL -= openEV::GRAVITY * 0.1*0.1 / 2.0;

  }


protected:
  TransitionMatrixType transitionMatrix;
};

} // namespace openEV

#endif /* GLIDERVARIOTRANSITIONMATRIX_H_ */
