/*
 * RotationMatrix_test.cpp
 *
 *  Created on: Feb 07, 2017
 *      Author: hor
 *
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2017  Kai Horstmann
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
#  include <config.h>
#endif

#include <math.h>
#include "gtest/gtest.h"
#include "util/RotationMatrix.h"

using namespace openEV;

class RotationMatrixTest :public ::testing::Test {
public:


    openEV::RotationMatrix rotMatrix;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

TEST_F(RotationMatrixTest, CoefficientTest) {

    int psi;
    int theta;
    int phi;

    // Verify the correct calculation of the coefficient of the matrix
    for (psi = 0; psi < 360 ; psi+= 5) {
        for (theta = -90; theta < 90; theta+= 5) {
            for (phi = -180 ; phi < 180 ; phi += 5) {
                double psiRad = double(psi) * M_PI / 180.0;
                double thetaRad = double(theta) * M_PI / 180.0;
                double phiRad = double(phi) * M_PI / 180.0;
                rotMatrix.setYaw(psi);
                rotMatrix.setPitch(theta);
                rotMatrix.setRoll(phi);
                RotationMatrix3DType& glo2Plane = rotMatrix.getMatrixGloToPlane();

                EXPECT_NEAR ( glo2Plane(0,0) , cos(thetaRad) * cos(psiRad) 		, 0.00001 );
                EXPECT_NEAR ( glo2Plane(0,1) , (cos(thetaRad) * sin(psiRad)) 	, 0.00001 );
                EXPECT_NEAR ( glo2Plane(0,2) , - sin(thetaRad)					, 0.00001);

                EXPECT_NEAR ( glo2Plane(1,0) , (sin(phiRad) * sin(thetaRad) * cos(psiRad) - cos(phiRad) * sin(psiRad))	,0.00001);
                EXPECT_NEAR ( glo2Plane(1,1) , (sin(phiRad) * sin(thetaRad) * sin(psiRad) + cos(phiRad) * cos(psiRad))	,0.00001);
                EXPECT_NEAR ( glo2Plane(1,2) , (sin(phiRad) * cos(thetaRad))											,0.00001);

                EXPECT_NEAR ( glo2Plane(2,0) , (cos(phiRad) * sin(thetaRad) * cos(psiRad) + sin(phiRad) * sin(psiRad))	,0.00001);
                EXPECT_NEAR ( glo2Plane(2,1) , (cos(phiRad) * sin(thetaRad) * sin(psiRad) - sin(phiRad) * cos(psiRad))	,0.00001);
                EXPECT_NEAR ( glo2Plane(2,2) , (cos(phiRad) * cos(thetaRad))											,0.00001);

            }
        }
    }

}

TEST_F(RotationMatrixTest, VectorTest) {

    int psi;
    int theta;
    int phi;

    // Verify the correct calculation of the coefficient of the matrix
    for (psi = 0; psi < 360 ; psi+= 5) {
        for (theta = -90; theta < 90; theta+= 5) {
            for (phi = -180 ; phi < 180 ; phi += 5) {
                rotMatrix.setYaw(psi);
                rotMatrix.setPitch(theta);
                rotMatrix.setRoll(phi);

                RotationMatrix3DType& glo2Plane = rotMatrix.getMatrixGloToPlane();
                RotationMatrix3DType& plane2Glo = rotMatrix.getMatrixPlaneToGlo();

                Vector3DType orgVect = {1.1f,2.2f,3.3f}, vect1, vect2;
                FloatType x,y,z;

                // vector calculation vs. component wise manual calculation
                rotMatrix.calcWorldVectorToPlaneVector(orgVect,vect1);
                x =     glo2Plane(0,0) * orgVect(0) +
                        glo2Plane(0,1) * orgVect(1) +
                        glo2Plane(0,2) * orgVect(2) ;
                y =     glo2Plane(1,0) * orgVect(0) +
                        glo2Plane(1,1) * orgVect(1) +
                        glo2Plane(1,2) * orgVect(2) ;
                z =     glo2Plane(2,0) * orgVect(0) +
                        glo2Plane(2,1) * orgVect(1) +
                        glo2Plane(2,2) * orgVect(2) ;

                EXPECT_NEAR (vect1(0),x,0.000001);
                EXPECT_NEAR (vect1(1),y,0.000001);
                EXPECT_NEAR (vect1(2),z,0.000001);

                // convert the plane vector back to the world coordinates, and compare with the original
                rotMatrix.calcPlaneVectorToWorldVector(vect1,vect2);
                EXPECT_NEAR (orgVect(0) , vect2(0),0.000001);
                EXPECT_NEAR (orgVect(1) , vect2(1),0.000001);
                EXPECT_NEAR (orgVect(2) , vect2(2),0.000001);

            }
        }
    }



}

