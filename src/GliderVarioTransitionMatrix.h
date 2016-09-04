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
 * This is the transition matrix implementation of the Kalman filter.
 * The transition matrix is re-calculated before every update step because it depends on the elapsed interval,
 * and on the current attitude (i.e. heading pitch and roll affect the TAS vs speed and course over ground).
 */
class GliderVarioTransitionMatrix
{
public:
  typedef Eigen::Matrix<FloatType,GliderVarioStatus::STATUS_NUM_ROWS,GliderVarioStatus::STATUS_NUM_ROWS> TransitionMatrixType;


  GliderVarioTransitionMatrix () {
    // Set the matrix to 0. There will be a lot of 0s in the matrix.
    transitionMatrix.setZero();

    // Some measured values are not propagated as time based changes, but which are only defined by the measurements or are just constants.
    // The values which depend on attitude and time difference are re-calculated at every cycle in GliderVarioTransitionMatrix::calcTransitionMatrix()

    transitionMatrix(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_LONGITUDE) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_LATITUDE) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_PITCH) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROLL) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_TAS) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_HEADING) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = 1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1;


    //--STATUS_IND_ACC_X------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_X,GliderVarioStatus::STATUS_IND_ACC_X) = 1;

    //--STATUS_IND_ACC_Y------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Y,GliderVarioStatus::STATUS_IND_ACC_Y) = 1;

    //--STATUS_IND_ACC_Z------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_ACC_Z,GliderVarioStatus::STATUS_IND_ACC_Z) = 1;

    //--STATUS_IND_ROTATION_X------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_X,GliderVarioStatus::STATUS_IND_ROTATION_X) = 1;

    //--STATUS_IND_ROTATION_Y------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Y,GliderVarioStatus::STATUS_IND_ROTATION_Y) = 1;

    //--STATUS_IND_ROTATION_Z------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_ROTATION_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z) = 1;

    //--STATUS_IND_GYRO_BIAS_X------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X) = 1;

    //--STATUS_IND_GYRO_BIAS_Y------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y) = 1;

    //--STATUS_IND_GYRO_BIAS_Z------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z) = 1;

    //--STATUS_IND_WIND_SPEED_N------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N) = 1;

    //--STATUS_IND_WIND_SPEED_E------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_WIND_SPEED_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E) = 1;

    //--STATUS_IND_ACC_X------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK) = -1;
    transitionMatrix(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED) = 1;

    //--STATUS_IND_STATUS_IND_GRAVITY------------------------------------------------------------------------------------
    transitionMatrix(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 1;



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
   * @param[in] lastStatus Most recent status vector. Used to convert world into local coordinates.
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

  }


protected:
  TransitionMatrixType transitionMatrix;
};

} // namespace openEV

#endif /* GLIDERVARIOTRANSITIONMATRIX_H_ */
