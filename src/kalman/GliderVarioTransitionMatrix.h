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
#include "util/FastMath.h"

namespace openEV
{


/**
 * This is the transition matrix implementation of the Kalman filter.
 * The transition matrix is re-calculated before every update step because it depends on the elapsed interval,
 * and on the current attitude (i.e. heading pitch and roll affect the TAS vs speed and course over ground).
 */
class OEV_PUBLIC GliderVarioTransitionMatrix
{
public:
    // typedef Eigen::Matrix<FloatType,GliderVarioStatus::STATUS_NUM_ROWS,GliderVarioStatus::STATUS_NUM_ROWS> TransitionMatrixType;
    typedef Eigen::SparseMatrix<FloatType> TransitionMatrixType;


    GliderVarioTransitionMatrix ()
    :transitionMatrix{GliderVarioStatus::STATUS_NUM_ROWS,GliderVarioStatus::STATUS_NUM_ROWS}
    {
        // Set the matrix to 0. There will be a lot of 0s in the matrix.
        //transitionMatrix.setZero();



    }
    virtual
    ~GliderVarioTransitionMatrix ();

    TransitionMatrixType&
    getTransitionMatrix () {
        return transitionMatrix;
    }

    /**
     * Recalculates the transition matrix and the new status at the same time.
     * Only active coefficients of the status transition matrix are recalculated.
     * All other coefficients are supposed to be 0 as they were set at construction time.
     * At the same time the new status is calculated from the old status.
     * Since this is (partially :) ) an EKF the new status is partially calculated from non-linear functions.
     * In these cases the status transition matrix are written with the (approximate) differential at the point of \p lastStatus.
     * @param[in] timeDiff Time since last update in seconds.
     * @param[in] lastStatus Most recent status vector. Used to convert world into local coordinates.
     * @param[out] newStatus The new status extrapolated from \p lastStatus, and \p timeDiff.
     */
    void
    calcTransitionMatrixAndStatus (
            FloatType                timeDiff,
            GliderVarioStatus const &lastStatus,
            GliderVarioStatus       &newStatus);


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
        calcTransitionMatrixAndStatus(timeDiff,oldStatus,newStatus);

        newStatus.getErrorCovariance_P() = transitionMatrix * oldStatus.getErrorCovariance_P() * transitionMatrix.transpose()
			          //+ oldStatus.getSystemNoiseCovariance_Q()
			          ;

        // Addition has to be added as a seperate statement. Eigen 3.3.4 did not like adding the noise covariance in the statement above.
        newStatus.getErrorCovariance_P() += oldStatus.getSystemNoiseCovariance_Q();

        newStatus.getSystemNoiseCovariance_Q() = oldStatus.getSystemNoiseCovariance_Q();

        newStatus.normalizeAngles();
    }

    /// See \ref staticRollTimeConstant
    static inline  FloatType getStaticRollTimeConstant() {
        return staticRollTimeConstant;
    }


    /**
     * See \ref staticRollTimeConstant
     *
     * @param staticRollTimeConstant New value of \ref staticRollTimeConstant
     */
    static inline void setStaticRollTimeConstant(FloatType staticRollTimeConstant) {
        GliderVarioTransitionMatrix::staticRollTimeConstant = staticRollTimeConstant;
    }

    /// See \ref dynamicRollTimeConstant
    static inline FloatType getDynamicRollTimeConstant () {
        return dynamicRollTimeConstant;
    }

    /**
     * See \ref dynamicRollTimeConstant
     * @param dynamicRollTimeConstant New value of \ref dynamicRollTimeConstant
     */
    static inline void getDynamicRollTimeConstant (FloatType dynamicRollTimeConstant ) {
        GliderVarioTransitionMatrix::dynamicRollTimeConstant = dynamicRollTimeConstant;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    TransitionMatrixType transitionMatrix;

    /// Time constant in seconds to correct the roll angle according to the static lateral acceleration ratio to \ref openEV::GRAVITY.
    static FloatType staticRollTimeConstant;
    /// Time constant in seconds to correct the roll angle according to the ideal bank angle for the current turn rate and True Air Speed (TAS).
    static FloatType dynamicRollTimeConstant;

    /**
     * Calculates the theoretical bank (roll) angle of a plane based on the turn rate and speed relative to the surrounding air mass.
     * The bank angle is calculated based on the ratio between the centrifugal force from turn rate and speed and \ref GRAVITY
     * on the assumption that with the bank angle centrifugal force and gravity are perfectly balanced, which is approximately true for gliders
     * which fly as efficiently as possible.
     *
     * I calculate the bank angle according to
     * <a href="https://de.wikipedia.org/wiki/Zentrifugalkraft#Rotierende_Fl.C3.BCssigkeit" >Wikipedia: Zentrifugalkraft, rotating liquids</a> or
     * <a href="https://en.wikipedia.org/wiki/Bucket_argument#Newton.27s_laws_of_motion" >Wikipedia: Bucket argument, Newton's laws of motion</a>,
     * using this formula:
     * <a href="https://wikimedia.org/api/rest_v1/media/math/render/svg/3aebc043525d0268f6a128d4f9c64636a5481b33" >Angle of the surface of a rotating liquid</a>.
     *
     * @param gloTurnRate Turn rate around the global Y axis (not around the plane axis!)
     * @param trueAirSpeed True air speed relative to the surrounding air
     * @return
     */
    static inline FloatType calcRotBankAngle (FloatType gloTurnRate,FloatType trueAirSpeed) {
        FloatType turnRateRad, turnRadius, bankAngleRot;
        if (gloTurnRate == 0.0f) {
            // When I am not turning I have no dynamic bank angle. Period.
            bankAngleRot = 0.0f;
        } else {
            turnRateRad = gloTurnRate * FastMath::degToRad;
            turnRadius = trueAirSpeed / turnRateRad;
            bankAngleRot = FastMath::fastATan2(turnRateRad*turnRateRad * turnRadius / GRAVITY,1.0f);
        }

        return bankAngleRot;
    }


};

} // namespace openEV

#endif /* GLIDERVARIOTRANSITIONMATRIX_H_ */
