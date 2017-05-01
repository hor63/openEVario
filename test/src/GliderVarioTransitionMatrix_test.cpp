/*
 * GliderVarioTransitionMatrix_test.cpp
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

#include "gtest/gtest.h"
#include "GliderVarioTransitionMatrix.h"

using namespace openEV;

class TransitionMatrixTest :public ::testing::Test {
public:


    openEV::GliderVarioTransitionMatrix transMatrix;
    openEV::GliderVarioStatus st1,st2;
};

TEST_F(TransitionMatrixTest, Gravity) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Gravity
    for (FloatType t = 0.01f; t<=1.0f; t+=0.11f  ) {
        for (FloatType g = 9.0f ; g <= 10.f; g+=0.01f) {
            FloatType tmp1;
            st1.gravity = g;

            transMatrix.updateStatus(st1,st2,t);

            // Now check the result
            EXPECT_EQ (st1.gravity,st2.gravity);

            // check the increment with the transition matrix
            tmp1 = st2.gravity + 0.01f * transMatrix.getTransitionMatrix().coeff(st1.STATUS_IND_GRAVITY,st1.STATUS_IND_GRAVITY);
            st1.gravity = g + 0.01f;
            transMatrix.updateStatus(st1,st2,t);
            EXPECT_EQ (tmp1,st2.gravity);


        }
    }

}

TEST_F(TransitionMatrixTest, Latitude) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Latitude, ground speed North, lineal acceleration, heading
    for (FloatType t = 0.01f; t<=1.3f; t+=0.23f  ) {
        for (FloatType lat = 45.0f ; lat <= 65.0f; lat += 5.33f) {
            for (FloatType speedGroundN = 0.0f; speedGroundN <= 80.0f; speedGroundN += 6.77f) {
                for (FloatType accel = -0.5f ; accel <= 0.5f; accel += 0.193f) {
                    for (FloatType heading = 0.0f ; heading < 360.0f; heading += 23.3f){
                        st1.latitude = lat * 3600.0f;
                        st1.groundSpeedNorth = speedGroundN;
                        st1.accelHeading = accel;
                        st1.heading = heading;

                        transMatrix.updateStatus(st1,st2,t);

                        FloatType arcSecPerM = 3600.0 / 111132.0;
                        FloatType expectResult =
                                lat * 3600.0f
                                + arcSecPerM * t*speedGroundN
                                + arcSecPerM * FastMath::fastCos(heading) * accel *t*t/2
                                ;

                        EXPECT_NEAR (st2.latitude,expectResult,fabs(expectResult*0.000001f)) <<
                                " at Latitude = " << lat << " groundSpeedN = " << speedGroundN << " acceleration = " << accel <<
                                " heading = " << heading << " time = " << t;

                        // Test the coefficients in the matrix as derivatives.
                        FloatType orgResult = expectResult;
                        FloatType resultDelta;
                        FloatType deltaValue;


                        // Modify the latitude
                        deltaValue = 10.0f;
                        st1.latitude = lat * 3600.0f + deltaValue;
                        transMatrix.updateStatus(st1,st2,t);
                        expectResult =
                                (lat * 3600.0f + deltaValue)
                                + arcSecPerM * t*speedGroundN
                                + arcSecPerM * FastMath::fastCos(heading) * accel *t*t/2
                                ;
                        resultDelta = deltaValue *
                                transMatrix.getTransitionMatrix()
                                .coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_LATITUDE);

                        EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) << " Latitude delta = " << deltaValue;
                        st1.latitude = lat * 3600.0f;

                        // Modify groundSpeedN
                        deltaValue = 1.0f;
                        st1.groundSpeedNorth = speedGroundN + deltaValue;
                        transMatrix.updateStatus(st1,st2,t);
                        expectResult =
                                lat * 3600.0f
                                + arcSecPerM * t* (speedGroundN + deltaValue)
                                + arcSecPerM * FastMath::fastCos(heading) * accel *t*t/2
                                ;
                        resultDelta = deltaValue *
                                transMatrix.getTransitionMatrix()
                                .coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N);

                        EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) <<
                                " groundSpeedN delta = " << deltaValue <<
                                " resultDelta = " << resultDelta;
                        st1.groundSpeedNorth = speedGroundN;

                        // Modify the acceleration
                        deltaValue = 1.0f;
                        st1.accelHeading = accel + deltaValue;
                        transMatrix.updateStatus(st1,st2,t);
                        expectResult =
                                lat * 3600.0f
                                + arcSecPerM * t* speedGroundN
                                + arcSecPerM * FastMath::fastCos(heading) * (accel + deltaValue) *t*t/2
                                ;
                        resultDelta = deltaValue *
                                transMatrix.getTransitionMatrix()
                                .coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_ACC_HEADING);

                        EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) <<
                                " accelHeading delta = " << deltaValue <<
                                " resultDelta = " << resultDelta;
                        st1.accelHeading = accel;

                        // Modify the heading
                        deltaValue = 1.0f;
                        st1.heading = heading + deltaValue;
                        transMatrix.updateStatus(st1,st2,t);
                        expectResult =
                                lat * 3600.0f
                                + arcSecPerM * t*speedGroundN
                                + arcSecPerM * FastMath::fastCos(heading + deltaValue) * accel *t*t/2
                                ;
                        resultDelta = deltaValue *
                                transMatrix.getTransitionMatrix()
                                .coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_HEADING);

                        EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) <<
                                " heading delta = " << deltaValue <<
                                " resultDelta = " << resultDelta <<
                                " heading = " << st1.heading <<
                                " acceleration = " << st1.accelHeading <<
                                " time = " << t <<
                                " Coefficient = " << transMatrix.getTransitionMatrix()
                                .coeff(GliderVarioStatus::STATUS_IND_LATITUDE,GliderVarioStatus::STATUS_IND_HEADING);
                        st1.heading = heading;

                    }
                }
            }
        }
    }


}

TEST_F(TransitionMatrixTest, Longitude) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: latitude, longitude, ground speed East, lineal acceleation, heading
    for (FloatType t = 0.01f; t<=1.3f; t+=0.23f  ) {
        for (FloatType latitude = 45.0f ; latitude <= 69.0f; latitude += 10.33f) {
            for (FloatType lon = 0.1f ; lon <= 0.9f; lon += 0.183f) {
                for (FloatType speedGroundE = 0.0f; speedGroundE <= 80.0f; speedGroundE += 6.77f) {
                    for (FloatType accel = -0.5f ; accel <= 0.5f; accel += 0.193f) {
                        for (FloatType heading = 0.0f ; heading < 360.0f; heading += 23.3f){
                            st1.latitude = latitude * 3600.0f;
                            st1.longitude = lon * 3600.0f;
                            st1.groundSpeedEast = speedGroundE;
                            st1.accelHeading = accel;
                            st1.heading = heading;

                            transMatrix.updateStatus(st1,st2,t);

                            FloatType arcSecPerM = 3600.0 / 111132.0 / FastMath::fastCos(latitude);
                            FloatType expectResult =
                                    lon * 3600.0f
                                    + arcSecPerM * t*speedGroundE
                                    + arcSecPerM * FastMath::fastSin(heading) * accel *t*t/2
                                    ;

                            EXPECT_NEAR (st2.longitude,expectResult,fabs(expectResult*0.000001f)) <<
                                    " at Latitude = " << lon << " groundSpeedE = " << speedGroundE << " acceleration = " << accel <<
                                    " heading = " << heading << " time = " << t;

                            // Test the coefficients in the matrix as derivatives.
                            FloatType orgResult = expectResult;
                            FloatType resultDelta;
                            FloatType deltaValue;


                            // Modify the longitude
                            deltaValue = 10.0f;
                            st1.longitude = lon * 3600.0f + deltaValue;
                            transMatrix.updateStatus(st1,st2,t);
                            expectResult =
                                    (lon * 3600.0f + deltaValue)
                                    + arcSecPerM * t*speedGroundE
                                    + arcSecPerM * FastMath::fastSin(heading) * accel *t*t/2
                                    ;
                            resultDelta = deltaValue *
                                    transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_LONGITUDE);

                            EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) << " Latitude delta = " << deltaValue;
                            st1.longitude = lon * 3600.0f;

                            // Modify groundSpeedE
                            deltaValue = 1.0f;
                            st1.groundSpeedEast = speedGroundE + deltaValue;
                            transMatrix.updateStatus(st1,st2,t);
                            expectResult =
                                    lon * 3600.0f
                                    + arcSecPerM * t* (speedGroundE + deltaValue)
                                    + arcSecPerM * FastMath::fastSin(heading) * accel *t*t/2
                                    ;
                            resultDelta = deltaValue *
                                    transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_SPEED_GROUND_E);

                            EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) <<
                                    " groundSpeedE delta = " << deltaValue <<
                                    " resultDelta = " << resultDelta;
                            st1.groundSpeedEast = speedGroundE;

                            // Modify the acceleration
                            deltaValue = 1.0f;
                            st1.accelHeading = accel + deltaValue;
                            transMatrix.updateStatus(st1,st2,t);
                            expectResult =
                                    lon * 3600.0f
                                    + arcSecPerM * t* speedGroundE
                                    + arcSecPerM * FastMath::fastSin(heading) * (accel + deltaValue) *t*t/2
                                    ;
                            resultDelta = deltaValue *
                                    transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_ACC_HEADING);

                            EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) <<
                                    " accelHeading delta = " << deltaValue <<
                                    " resultDelta = " << resultDelta;
                            st1.accelHeading = accel;

                            // Modify the heading
                            deltaValue = 1.0f;
                            st1.heading = heading + deltaValue;
                            transMatrix.updateStatus(st1,st2,t);
                            expectResult =
                                    lon * 3600.0f
                                    + arcSecPerM * t*speedGroundE
                                    + arcSecPerM * FastMath::fastSin(heading + deltaValue) * accel *t*t/2
                                    ;
                            resultDelta = deltaValue *
                                    transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_HEADING);

                            EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) <<
                                    " heading delta = " << deltaValue <<
                                    " resultDelta = " << resultDelta <<
                                    " heading = " << st1.heading <<
                                    " acceleration = " << st1.accelHeading <<
                                    " time = " << t <<
                                    " Coefficient = " << transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE,GliderVarioStatus::STATUS_IND_HEADING);
                            st1.heading = heading;

                        }
                    }
                }
            }
        }
    }


}

TEST_F(TransitionMatrixTest, AltMSL) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: altitude MSL, vertical speed, vertical acceleration
    for (FloatType t = 0.01f; t<=1.3f; t+=0.23f  ) {
        for (FloatType alt = 20.0; alt < 1000.0; alt += 123.45) {
            for (FloatType vertSpeed = -3.0f ; vertSpeed <= 3.0f; vertSpeed += 0.83) {
                for (FloatType vertAcc = -1.0f; vertAcc <= 1.0f; vertAcc += 0.13) {
                    st1.altMSL = alt;
                    st1.verticalSpeed = vertSpeed;
                    st1.accelVertical = vertAcc;

                    transMatrix.updateStatus(st1,st2,t);

                    FloatType expectResult =
                            alt
                            - vertSpeed * t
                            - vertAcc *t*t/2
                            ;

                    EXPECT_NEAR (st2.altMSL,expectResult,fabs(expectResult*0.000001f)) <<
                            " at alt = " << alt << " vertSpeed = " << vertSpeed << " vertAcc = " << vertAcc <<
                            " time = " << t;

                    // Test the coefficients in the matrix as derivatives.
                    FloatType orgResult = expectResult;
                    FloatType resultDelta;
                    FloatType deltaValue;


                    // Modify the altitude
                    deltaValue = 3.33f;
                    st1.altMSL = alt + deltaValue;
                    transMatrix.updateStatus(st1,st2,t);
                    expectResult =
                            (alt + deltaValue)
                            - vertSpeed * t
                            - vertAcc *t*t/2
                            ;
                    resultDelta = deltaValue *
                            transMatrix.getTransitionMatrix()
                            .coeff(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL);

                    EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) << " Altitude delta = " << deltaValue;
                    st1.altMSL = alt;

                    // Modify the vertical speed
                    deltaValue = 1.33f;
                    st1.verticalSpeed = vertSpeed + deltaValue;
                    transMatrix.updateStatus(st1,st2,t);
                    expectResult =
                            alt
                            - (vertSpeed + deltaValue) * t
                            - vertAcc *t*t/2
                            ;
                    resultDelta = deltaValue *
                            transMatrix.getTransitionMatrix()
                            .coeff(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED);

                    EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) << " vertical speed delta = " << deltaValue;
                    st1.verticalSpeed = vertSpeed;

                    // Modify the vertical acceleration
                    deltaValue = 0.133f;
                    st1.accelVertical = vertAcc + deltaValue;
                    transMatrix.updateStatus(st1,st2,t);
                    expectResult =
                            alt
                            - vertSpeed * t
                            - (vertAcc + deltaValue) *t*t/2
                            ;
                    resultDelta = deltaValue *
                            transMatrix.getTransitionMatrix()
                            .coeff(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ACC_VERTICAL);

                    EXPECT_NEAR (expectResult,orgResult + resultDelta,fabs(expectResult*0.000001f)) << " vertical acceleration delta = " << deltaValue;
                    st1.accelVertical = vertAcc;

                }
            }
        }
    }
}


TEST_F(TransitionMatrixTest, Pitch) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Pitch angle, pitch rate around the y axis
    for (FloatType t = 0.01f; t<=1.3f; t+=0.23f  ) {
        for (FloatType pitch = -90.0f; pitch <= 90.0f; pitch += 13.33f) {
            for (FloatType pitchRate = -20.0f ; pitchRate <= 20.0f; pitchRate += 6.67f) {
                st1.pitchAngle = pitch;
                st1.pitchRateY = pitchRate;

                transMatrix.updateStatus(st1,st2,t);

                FloatType expectResult =
                        pitch
                        + pitchRate * t
                        ;

                // For derivates test I need the raw result without normalization.
                FloatType orgResult = expectResult;


                if (expectResult < -90.0f) {
                    expectResult = -180.0f - expectResult;
                } else if (expectResult > 90.0f) {
                    expectResult = 180.0f - expectResult;
                }

                EXPECT_NEAR (st2.pitchAngle,expectResult,0.00001f) <<
                        " at pitch = " << pitch << " pitchRate = " << pitchRate <<
                        " time = " << t;

                // Test the coefficients in the matrix as derivatives.
                FloatType resultDelta;
                FloatType deltaValue;


                // Modify the pitch
                deltaValue = 1.0f;
                st1.pitchAngle = pitch + deltaValue;
                transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        (pitch + deltaValue)
                        + pitchRate * t
                        ;
                if (expectResult < -90.0f) {
                    expectResult = -180.0f - expectResult;
                } else if (expectResult > 90.0f) {
                    expectResult = 180.0f - expectResult;
                }
                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_PITCH);

                FloatType deltaResult;
                deltaResult = orgResult + resultDelta;
                if (deltaResult < -90.0f) {
                    deltaResult = -180.0f - deltaResult;
                } else if (deltaResult > 90.0f) {
                    deltaResult = 180.0f - deltaResult;
                }

                EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " Pitch delta = " << deltaValue
                        << " pitch = " << pitch << " pitch rate = " << pitchRate << " t = " << t
                        << " resultDelta = " << resultDelta << " orgResult = " << orgResult
                        << " st2.pitchAngle = " << st2.pitchAngle;
                st1.pitchAngle = pitch;

                // Modify the pitch rate
                deltaValue = 1.0f;
                st1.pitchRateY = pitchRate + deltaValue;
                transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        pitch
                        + (pitchRate + deltaValue) * t
                        ;
                if (expectResult < -90.0f) {
                    expectResult = -180.0f - expectResult;
                } else if (expectResult > 90.0f) {
                    expectResult = 180.0f - expectResult;
                }
                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_PITCH,GliderVarioStatus::STATUS_IND_ROTATION_Y);
                deltaResult = orgResult + resultDelta;
                if (deltaResult < -90.0f) {
                    deltaResult = -180.0f - deltaResult;
                } else if (deltaResult > 90.0f) {
                    deltaResult = 180.0f - deltaResult;
                }

                EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " pitchRate delta = " << deltaValue
                        << " pitch = " << pitch << " pitch rate = " << pitchRate << " t = " << t;
                st1.pitchRateY = pitchRate;
            }
        }
    }

}


TEST_F(TransitionMatrixTest, Roll) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Roll angle, roll rate around the x axis
    for (FloatType t = 0.01f; t<=1.3f; t+=0.23f  ) {
        for (FloatType roll = -180.0f; roll <= 180.0f; roll += 23.33f) {
            for (FloatType rollRate = -20.0f ; rollRate <= 20.0f; rollRate += 6.67f) {
                st1.rollAngle = roll;
                st1.rollRateX = rollRate;

                transMatrix.updateStatus(st1,st2,t);

                FloatType expectResult =
                        roll
                        + rollRate * t
                        ;

                // For derivates test I need the raw result without normalization.
                FloatType orgResult = expectResult;

                if (expectResult < -180.0f) {
                    expectResult += 360.0f;
                } else if (expectResult > 180.0f) {
                    expectResult -= 360.0f;
                }

                EXPECT_NEAR (st2.rollAngle,expectResult,fabs(expectResult*0.00001f)) <<
                        " at roll = " << roll << " rollRate = " << rollRate <<
                        " time = " << t;

                // Test the coefficients in the matrix as derivatives.
                FloatType resultDelta;
                FloatType deltaValue;


                // Modify the roll
                deltaValue = 1.0f;
                st1.rollAngle = roll + deltaValue;
                transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        (roll + deltaValue)
                        + rollRate * t
                        ;
                if (expectResult < -180.0f) {
                    expectResult += 360.0f;
                } else if (expectResult > 180.0f) {
                    expectResult -= 360.0f;
                }
                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROLL);
                FloatType deltaResult;
                deltaResult = orgResult + resultDelta;
                if (deltaResult < -180.0f) {
                    deltaResult += 360.0f;
                } else if (deltaResult > 180.0f) {
                    deltaResult -= 360.0f;
                }

                EXPECT_NEAR (expectResult,deltaResult,fabs(expectResult*0.00001f)) << " Roll delta = " << deltaValue;
                st1.rollAngle = roll;

                // Modify the roll rate
                deltaValue = 1.0f;
                st1.rollRateX = rollRate + deltaValue;
                transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        roll
                        + (rollRate + deltaValue) * t
                        ;
                if (expectResult < -180.0f) {
                    expectResult += 360.0f;
                } else if (expectResult > 180.0f) {
                    expectResult -= 360.0f;
                }
                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_ROLL,GliderVarioStatus::STATUS_IND_ROTATION_X);
                deltaResult = orgResult + resultDelta;
                if (deltaResult < -180.0f) {
                    deltaResult += 360.0f;
                } else if (deltaResult > 180.0f) {
                    deltaResult -= 360.0f;
                }

                EXPECT_NEAR (expectResult,deltaResult,fabs(expectResult*0.00001f)) << " Roll rate delta = " << deltaValue;
                st1.rollRateX = rollRate;
            }
        }
    }
}

TEST_F(TransitionMatrixTest, Heading) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType t = 0.01f; t<=1.3f; t+=0.23f  ) {
        for (FloatType heading = 0.0f; heading <= 360.0f; heading += 32.33f) {
            for (FloatType yawRate = -20.0f ; yawRate <= 20.0f; yawRate += 6.67f) {
                st1.heading = heading;
                st1.yawRateZ = yawRate;

                transMatrix.updateStatus(st1,st2,t);

                FloatType expectResult =
                        heading
                        + yawRate * t
                        ;
                // For derivates test I need the raw result without normalization.
                FloatType orgResult = expectResult;

                if (expectResult < 0.0f) {
                    expectResult += 360.0f;
                } else if (expectResult > 360.0f) {
                    expectResult -= 360.0f;
                }

                EXPECT_NEAR (st2.heading,expectResult,fabs(expectResult*0.00001f)) <<
                        " at heading = " << heading << " yawRate = " << yawRate <<
                        " time = " << t;

                // Test the coefficients in the matrix as derivatives.
                FloatType resultDelta;
                FloatType deltaValue;


                // Modify the roll
                deltaValue = 1.0f;
                st1.heading = heading + deltaValue;
                transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        (heading + deltaValue)
                        + yawRate * t
                        ;
                if (expectResult < 0.0f) {
                    expectResult += 360.0f;
                } else if (expectResult > 360.0f) {
                    expectResult -= 360.0f;
                }
                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_HEADING);
                FloatType deltaResult;
                deltaResult = orgResult + resultDelta;
                if (deltaResult < 0.0f) {
                    deltaResult += 360.0f;
                } else if (deltaResult > 360.0f) {
                    deltaResult -= 360.0f;
                }

                EXPECT_NEAR (expectResult,deltaResult,fabs(expectResult*0.00001f)) << " Heading delta = " << deltaValue
                        << " Heading = " << heading << " yawRate = " << yawRate << " time = " << t;
                st1.heading = heading;

                // Modify the roll rate
                deltaValue = 1.0f;
                st1.yawRateZ = yawRate + deltaValue;
                transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        heading
                        + (yawRate + deltaValue) * t
                        ;
                if (expectResult < 0.0f) {
                    expectResult += 360.0f;
                } else if (expectResult > 360.0f) {
                    expectResult -= 360.0f;
                }
                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_HEADING,GliderVarioStatus::STATUS_IND_ROTATION_Z);
                deltaResult = orgResult + resultDelta;
                if (deltaResult < 0.0f) {
                    deltaResult += 360.0f;
                } else if (deltaResult > 360.0f) {
                    deltaResult -= 360.0f;
                }

                EXPECT_NEAR (expectResult,deltaResult,fabs(expectResult*0.00001f)) << " Yaw rate delta = " << deltaValue
                        << " Heading = " << heading << " heading = " << yawRate << " time = " << t;
                st1.yawRateZ = yawRate;
            }
        }
    }

}
