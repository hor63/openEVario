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

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include "gtest/gtest.h"
#include "kalman/GliderVarioTransitionMatrix.h"

using namespace openEV;

class TransitionMatrixTest :public ::testing::Test {
public:


    openEV::GliderVarioTransitionMatrix transMatrix;
    openEV::GliderVarioStatus st1,st2;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
        for (double lat = 45.0 ; lat <= 65.0; lat += 5.33) {
            for (FloatType speedGroundN = 0.0f; speedGroundN <= 80.0f; speedGroundN += 6.77f) {
                for (FloatType accel = -0.5f ; accel <= 0.5f; accel += 0.193f) {
                    for (FloatType heading = 0.0f ; heading < 360.0f; heading += 23.3f){
                    	st1.latitude(lat);
                        st1.groundSpeedNorth = speedGroundN;
                        st1.accelHeading = accel;
                        st1.heading = heading;

                        transMatrix.updateStatus(st1,st2,t);

                        double expectResult =
                                lat
                                + (t*speedGroundN
                                		//+ FastMath::fastCos(heading) * accel *t*t/2.0
										)/LEN_LAT_ARC_SEC / 3600.0;
                                ;

                        EXPECT_NEAR (st2.latitude(),expectResult,abs(expectResult*0.0000000001)) <<
                                " at Latitude = " << lat << " groundSpeedN = " << speedGroundN << " acceleration = " << accel <<
                                " heading = " << heading << " time = " << t;

                        // Test the coefficients in the matrix as derivatives.
                        double orgResult = expectResult;
                        double resultDelta;
                        double deltaValue;


                        // Modify the latitude by 10 arc seconds
                        deltaValue = 10.0 / 3600.0;
                        st1.latitude (lat + deltaValue);
                        // transMatrix.updateStatus(st1,st2,t);
                        expectResult =
                                (lat + deltaValue)
                                + (t*speedGroundN
                                		//+ FastMath::fastCos(heading) * accel *t*t/2.0
										) / LEN_LAT_ARC_SEC / 3600.0
                                ;

                        resultDelta = deltaValue *
                                transMatrix.getTransitionMatrix()
                                .coeff(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_LATITUDE_OFFS);

                        EXPECT_NEAR (expectResult,orgResult + resultDelta,abs(expectResult*0.0000000001)) << " Latitude delta = " << deltaValue;
                        st1.latitude (lat);

                        // Modify groundSpeedN
                        deltaValue = 1.0f;
                        st1.groundSpeedNorth = speedGroundN + deltaValue;
                        // transMatrix.updateStatus(st1,st2,t);
                        expectResult =
                                lat
                                + (t* (speedGroundN + deltaValue)
                                		// + FastMath::fastCos(heading) * accel *t*t/2.0
										) / LEN_LAT_ARC_SEC / 3600.0;
                                ;
                        resultDelta =
                        		(deltaValue *
                        				transMatrix.getTransitionMatrix().coeff(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,
                        						GliderVarioStatus::STATUS_IND_SPEED_GROUND_N))
								/ LEN_LAT_ARC_SEC / 3600.0;

                        EXPECT_NEAR (expectResult,orgResult + resultDelta,abs(expectResult*0.0000000001)) <<
                                " groundSpeedN delta = " << deltaValue <<
                                " resultDelta = " << resultDelta
								<< "\n (GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N) = "
								<< transMatrix.getTransitionMatrix().coeff(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_SPEED_GROUND_N);

                        st1.groundSpeedNorth = speedGroundN;

                        // Modify the heading
                        deltaValue = 1.0f;
                        st1.heading = heading + deltaValue;
                        // transMatrix.updateStatus(st1,st2,t);
                        expectResult =
                                lat
                                + (t*speedGroundN
                                		// + FastMath::fastCos(heading + deltaValue) * accel *t*t/2.0
										) / LEN_LAT_ARC_SEC / 3600.0
                                ;
                        resultDelta =
                        		(deltaValue *
                        				transMatrix.getTransitionMatrix()
										.coeff(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_HEADING))
								/ LEN_LAT_ARC_SEC / 3600.0;

                        EXPECT_NEAR (expectResult,orgResult + resultDelta,abs(expectResult*0.0000000001)) <<
                                " heading delta = " << deltaValue <<
                                " resultDelta = " << resultDelta <<
                                " heading = " << st1.heading <<
                                " acceleration = " << st1.accelHeading <<
                                " time = " << t <<
                                " Coefficient = " << transMatrix.getTransitionMatrix()
                                .coeff(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_HEADING);
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
        for (double latitude = 45.0 ; latitude <= 69.0; latitude += 10.33) {
            for (double lon = 0.1 ; lon <= 0.9; lon += 0.183) {
                for (FloatType speedGroundE = 0.0f; speedGroundE <= 80.0f; speedGroundE += 6.77f) {
                    for (FloatType accel = -0.5f ; accel <= 0.5f; accel += 0.193f) {
                        for (FloatType heading = 0.0f ; heading < 360.0f; heading += 23.3f){
                            st1.latitude (latitude);
                            st1.longitude (lon);
                            st1.groundSpeedEast = speedGroundE;
                            st1.accelHeading = accel;
                            st1.heading = heading;

                            transMatrix.updateStatus(st1,st2,t);

                            double degPerM = 1.0 / (111132.0 * FastMath::fastCos(latitude));

                            EXPECT_NEAR (degPerM, 1.0/ (st1.getLenLongitudeArcSec()*3600.0),0.0000000001);

                            double expectResult =
                                    lon
                                    + (t*speedGroundE
                                    		// + FastMath::fastSin(heading) * accel *t*t/2.0
											)
										* degPerM
                                    ;

                            EXPECT_NEAR (st2.longitude(),expectResult,0.000000001) <<
                                    " at Latitude = " << lon << " groundSpeedE = " << speedGroundE << " acceleration = " << accel <<
                                    " heading = " << heading << " time = " << t;

                            // Test the coefficients in the matrix as derivatives.
                            double orgResult = expectResult;
                            double resultDelta;
                            double deltaValue;


                            // Modify the longitude by 10 sec.
                            deltaValue = 10.0 / 3600.0;
                            st1.longitude (lon + deltaValue);
                            // transMatrix.updateStatus(st1,st2,t);
                            expectResult =
                                    lon + deltaValue
                                    + (t*speedGroundE
                                    		// + FastMath::fastSin(heading) * accel *t*t/2.0
                                    		) * degPerM
                                    ;
                            resultDelta = deltaValue *
                                    transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS);
									//* degPerM;

                            EXPECT_NEAR (expectResult,orgResult + resultDelta,0.0000000001) << " Longitude delta = " << deltaValue;
                            st1.longitude (lon);

                            // Modify groundSpeedE
                            deltaValue = 1.0f;
                            st1.groundSpeedEast = speedGroundE + deltaValue;
                            // transMatrix.updateStatus(st1,st2,t);
                            expectResult =
                                    lon
                                    + (t* (speedGroundE + deltaValue)
                                    		// + FastMath::fastSin(heading) * accel *t*t/2.0
											) * degPerM;
                                    ;
                            resultDelta = deltaValue *
                                    transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_SPEED_GROUND_E) * degPerM;

                            EXPECT_NEAR (expectResult,orgResult + resultDelta,0.0000000001) <<
                                    " groundSpeedE delta = " << deltaValue <<
                                    " resultDelta = " << resultDelta;
                            st1.groundSpeedEast = speedGroundE;

                            // Modify the heading
                            deltaValue = 1.0f;
                            st1.heading = heading + deltaValue;
                            // transMatrix.updateStatus(st1,st2,t);
                            expectResult =
                                    lon
                                    + (t*speedGroundE
                                    		// + FastMath::fastSin(heading + deltaValue) * accel *t*t/2.0
											) * degPerM
                                    ;
                            resultDelta = deltaValue *
                                    transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_HEADING) * degPerM;

                            EXPECT_NEAR (expectResult,orgResult + resultDelta,0.00000001) <<
                                    " heading delta = " << deltaValue <<
                                    " resultDelta = " << resultDelta <<
                                    " heading = " << st1.heading <<
                                    " acceleration = " << st1.accelHeading <<
                                    " time = " << t <<
                                    " Coefficient = " << transMatrix.getTransitionMatrix()
                                    .coeff(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_HEADING);
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
				st1.altMSL = alt;
				st1.verticalSpeed = vertSpeed;

				transMatrix.updateStatus(st1,st2,t);

				FloatType expectResult =
						alt
						- vertSpeed * t
						;

				EXPECT_NEAR (st2.altMSL,expectResult,fabsf(expectResult*0.000001f)) <<
						" at alt = " << alt << " vertSpeed = " << vertSpeed << " time = " << t;

				// Test the coefficients in the matrix as derivatives.
				FloatType orgResult = expectResult;
				FloatType resultDelta;
				FloatType deltaValue;


				// Modify the altitude
				deltaValue = 3.33f;
				st1.altMSL = alt + deltaValue;
				// transMatrix.updateStatus(st1,st2,t);
				expectResult =
						(alt + deltaValue)
						- vertSpeed * t
						;
				resultDelta = deltaValue *
						transMatrix.getTransitionMatrix()
						.coeff(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL);

				EXPECT_NEAR (expectResult,orgResult + resultDelta,fabsf(expectResult*0.000001f)) << " Altitude delta = " << deltaValue;
				st1.altMSL = alt;

				// Modify the vertical speed
				deltaValue = 1.33f;
				st1.verticalSpeed = vertSpeed + deltaValue;
				// transMatrix.updateStatus(st1,st2,t);
				expectResult =
						alt
						- (vertSpeed + deltaValue) * t
						;
				resultDelta = deltaValue *
						transMatrix.getTransitionMatrix()
						.coeff(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED);

				EXPECT_NEAR (expectResult,orgResult + resultDelta,fabsf(expectResult*0.000001f)) << " vertical speed delta = " << deltaValue;
				st1.verticalSpeed = vertSpeed;

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
                // transMatrix.updateStatus(st1,st2,t);
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
                // transMatrix.updateStatus(st1,st2,t);
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

                EXPECT_NEAR (st2.rollAngle,expectResult,fabsf(expectResult*0.00001f)) <<
                        " at roll = " << roll << " rollRate = " << rollRate <<
                        " time = " << t;

                // Test the coefficients in the matrix as derivatives.
                FloatType resultDelta;
                FloatType deltaValue;


                // Modify the roll
                deltaValue = 1.0f;
                st1.rollAngle = roll + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
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

                EXPECT_NEAR (expectResult,deltaResult,fabsf(expectResult*0.00001f)) << " Roll delta = " << deltaValue;
                st1.rollAngle = roll;

                // Modify the roll rate
                deltaValue = 1.0f;
                st1.rollRateX = rollRate + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
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

                EXPECT_NEAR (expectResult,deltaResult,fabsf(expectResult*0.00001f)) << " Roll rate delta = " << deltaValue;
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

                EXPECT_NEAR (st2.heading,expectResult,fabsf(expectResult*0.00001f)) <<
                        " at heading = " << heading << " yawRate = " << yawRate <<
                        " time = " << t;

                // Test the coefficients in the matrix as derivatives.
                FloatType resultDelta;
                FloatType deltaValue;


                // Modify the heading
                deltaValue = 1.0f;
                st1.heading = heading + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
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

                EXPECT_NEAR (expectResult,deltaResult,fabsf(expectResult*0.00001f)) << " Heading delta = " << deltaValue
                        << " Heading = " << heading << " yawRate = " << yawRate << " time = " << t;
                st1.heading = heading;

                // Modify the yaw rate
                deltaValue = 1.0f;
                st1.yawRateZ = yawRate + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
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

                EXPECT_NEAR (expectResult,deltaResult,fabsf(expectResult*0.00001f)) << " Yaw rate delta = " << deltaValue
                        << " Heading = " << heading << " heading = " << yawRate << " time = " << t;
                st1.yawRateZ = yawRate;
            }
        }
    }

}

TEST_F(TransitionMatrixTest, GroundSpeedNorth) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType t = 0.01f; t<=1.3f; t+=0.34f  ) {
        for (FloatType heading = 10.0f; heading <= 350.0f; heading += 52.33f) {
            for (FloatType trueAirSpeed = 0.0f ; trueAirSpeed <= 50.0f; trueAirSpeed += 8.67f) {
//                for (FloatType accelHeading = 0.0f; accelHeading <= 2.0f; accelHeading += 0.67f) {
				  for (FloatType windSpeedN = -10.0f; windSpeedN <= 12.0f; windSpeedN += 4.69f) {

						st1.heading = heading;
						st1.trueAirSpeed = trueAirSpeed;
//						st1.accelHeading = accelHeading;
						st1.windSpeedNorth = windSpeedN;

						transMatrix.updateStatus(st1,st2,t);

						FloatType expectResult =
								FastMath::fastCos(heading) * (trueAirSpeed
//										+ accelHeading*t
										)
								+ windSpeedN
								;

						EXPECT_NEAR (st2.groundSpeedNorth,expectResult,0.00001f) <<
								" at heading   = " << heading <<
								" trueAirSpeed = " << trueAirSpeed <<
//								" accelHeading = " << accelHeading <<
								" windSpeedN   = " << windSpeedN <<
								" time = " << t;

						// Test the coefficients in the matrix as derivatives.
						FloatType orgResult = expectResult;
						FloatType resultDelta;
						FloatType deltaResult;
						FloatType deltaValue;


						// Modify the heading
						deltaValue = 1.0f;
						st1.heading = heading + deltaValue;
						// transMatrix.updateStatus(st1,st2,t);
						expectResult =
								FastMath::fastCos(heading + deltaValue) * (trueAirSpeed
//										+ accelHeading*t
										)
								+ windSpeedN
								;

						resultDelta = deltaValue *
								transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_HEADING);
						deltaResult = orgResult + resultDelta;


						// Well, we are entering non-linear territory. The accuracy goes pretty much down the drain here.
						EXPECT_NEAR (expectResult,deltaResult,0.007f) << " Heading delta = " << deltaValue <<
								" at heading   = " << heading <<
								" trueAirSpeed = " << trueAirSpeed <<
//								" accelHeading = " << accelHeading <<
								" windSpeedN   = " << windSpeedN <<
								" time = " << t
								<< std::endl <<
								"   resultDelta = " << resultDelta <<
								" Coefficient = " << transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_HEADING) <<
								" expected coefficient = " << FastMath::degToRad*(-FastMath::fastSin(heading)*(trueAirSpeed
//										+ accelHeading*t
										));

						st1.heading = heading;

						// Modify the TAS
						deltaValue = 1.0f;
						st1.trueAirSpeed = trueAirSpeed + deltaValue;
						// transMatrix.updateStatus(st1,st2,t);
						expectResult =
								FastMath::fastCos(heading ) * ((trueAirSpeed+ deltaValue)
//										+ accelHeading*t
										)
								+ windSpeedN
								;

						resultDelta = deltaValue *
								transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_TAS);
						deltaResult = orgResult + resultDelta;

						EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " TAS delta = " << deltaValue <<
								" at heading   = " << heading <<
								" trueAirSpeed = " << trueAirSpeed <<
//								" accelHeading = " << accelHeading <<
								" windSpeedN   = " << windSpeedN <<
								" time = " << t;
						st1.trueAirSpeed = trueAirSpeed;

/*
 *						// Modify the acceleration
 *						deltaValue = 1.0f;
 *						st1.accelHeading = accelHeading + deltaValue;
 *						// transMatrix.updateStatus(st1,st2,t);
 *						expectResult =
 *								FastMath::fastCos(heading) * (trueAirSpeed + (accelHeading + deltaValue)*t)
 *								+ windSpeedN
 *								;
 *
 *						resultDelta = deltaValue *
 *								transMatrix.getTransitionMatrix()
 *								.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_ACC_HEADING);
 *						deltaResult = orgResult + resultDelta;
 *
 *						EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " aceleration delta = " << deltaValue <<
 *								" at heading   = " << heading <<
 *								" trueAirSpeed = " << trueAirSpeed <<
 *								" accelHeading = " << accelHeading <<
 *								" windSpeedN   = " << windSpeedN <<
 *								" time = " << t;
 *						st1.accelHeading = accelHeading;
 */
						// Modify the Northern wind component
						deltaValue = 1.0f;
						st1.windSpeedNorth = windSpeedN + deltaValue;
						// transMatrix.updateStatus(st1,st2,t);
						expectResult =
								FastMath::fastCos(heading) * (trueAirSpeed
//										+ accelHeading*t
										)
								+ (windSpeedN + deltaValue)
								;

						resultDelta = deltaValue *
								transMatrix.getTransitionMatrix()
								.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N);
						deltaResult = orgResult + resultDelta;

						EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " windSpeedN delta = " << deltaValue <<
								" at heading   = " << heading <<
								" trueAirSpeed = " << trueAirSpeed <<
//								" accelHeading = " << accelHeading <<
								" windSpeedN   = " << windSpeedN <<
								" time = " << t;
						st1.windSpeedNorth = windSpeedN;

					}

//                }
            }
        }
    }

}

TEST_F(TransitionMatrixTest, GroundSpeedEast) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType t = 0.01f; t<=1.3f; t+=0.34f  ) {
        for (FloatType heading = 10.0f; heading <= 350.0f; heading += 52.33f) {
            for (FloatType trueAirSpeed = 0.0f ; trueAirSpeed <= 50.0f; trueAirSpeed += 8.67f) {
//                for (FloatType accelHeading = 0.0f; accelHeading <= 2.0f; accelHeading += 0.67f) {
				for (FloatType windSpeedE = -10.0f; windSpeedE <= 12.0f; windSpeedE += 4.69f) {

					st1.heading = heading;
					st1.trueAirSpeed = trueAirSpeed;
//					st1.accelHeading = accelHeading;
					st1.windSpeedEast = windSpeedE;

					transMatrix.updateStatus(st1,st2,t);

					FloatType expectResult =
							FastMath::fastSin(heading) * (trueAirSpeed
									// + accelHeading*t
									)
							+ windSpeedE
							;

					EXPECT_NEAR (st2.groundSpeedEast,expectResult,0.00001f) <<
							" at heading   = " << heading <<
							" trueAirSpeed = " << trueAirSpeed <<
//							" accelHeading = " << accelHeading <<
							" windSpeedE   = " << windSpeedE <<
							" time = " << t;

					// Test the coefficients in the matrix as derivatives.
					FloatType orgResult = expectResult;
					FloatType resultDelta;
					FloatType deltaResult;
					FloatType deltaValue;


					// Modify the heading
					deltaValue = 1.0f;
					st1.heading = heading + deltaValue;
					// transMatrix.updateStatus(st1,st2,t);
					expectResult =
							FastMath::fastSin(heading + deltaValue) * (trueAirSpeed
									// + accelHeading*t
									)
							+ windSpeedE
							;

					resultDelta = deltaValue *
							transMatrix.getTransitionMatrix()
							.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_HEADING);
					deltaResult = orgResult + resultDelta;


					// Well, we are entering non-linear territory. The accuracy goes pretty much down the drain here.
					EXPECT_NEAR (expectResult,deltaResult,0.007f) << " Heading delta = " << deltaValue <<
							" at heading   = " << heading <<
							" trueAirSpeed = " << trueAirSpeed <<
//							" accelHeading = " << accelHeading <<
							" windSpeedE   = " << windSpeedE <<
							" time = " << t
							<< std::endl <<
							"   resultDelta = " << resultDelta <<
							" Coefficient = " << transMatrix.getTransitionMatrix()
							.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_HEADING) <<
							" expected coefficient = " << FastMath::degToRad*(-FastMath::fastSin(heading)*(trueAirSpeed
									// + accelHeading*t
									));

					st1.heading = heading;

					// Modify the TAS
					deltaValue = 1.0f;
					st1.trueAirSpeed = trueAirSpeed + deltaValue;
					// transMatrix.updateStatus(st1,st2,t);
					expectResult =
							FastMath::fastSin(heading ) * ((trueAirSpeed+ deltaValue)
									// + accelHeading*t
									)
							+ windSpeedE
							;

					resultDelta = deltaValue *
							transMatrix.getTransitionMatrix()
							.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_TAS);
					deltaResult = orgResult + resultDelta;

					EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " TAS delta = " << deltaValue <<
							" at heading   = " << heading <<
							" trueAirSpeed = " << trueAirSpeed <<
//							" accelHeading = " << accelHeading <<
							" windSpeedE   = " << windSpeedE <<
							" time = " << t;
					st1.trueAirSpeed = trueAirSpeed;

/*
 *					// Modify the acceleration
 *					deltaValue = 1.0f;
 *					st1.accelHeading = accelHeading + deltaValue;
 *					// transMatrix.updateStatus(st1,st2,t);
 *					expectResult =
 *							FastMath::fastSin(heading) * (trueAirSpeed + (accelHeading + deltaValue)*t)
 *							+ windSpeedE
 *							;
 *
 *					resultDelta = deltaValue *
 *							transMatrix.getTransitionMatrix()
 *							.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_ACC_HEADING);
 *					deltaResult = orgResult + resultDelta;
 *
 *					EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " aceleration delta = " << deltaValue <<
 *							" at heading   = " << heading <<
 *							" trueAirSpeed = " << trueAirSpeed <<
 *							" accelHeading = " << accelHeading <<
 *							" windSpeedE   = " << windSpeedE <<
 *							" time = " << t;
 *					st1.accelHeading = accelHeading;
 */
					// Modify the Eastern wind component
					deltaValue = 1.0f;
					st1.windSpeedEast = windSpeedE + deltaValue;
					// transMatrix.updateStatus(st1,st2,t);
					expectResult =
							FastMath::fastSin(heading) * (trueAirSpeed
									// + accelHeading*t
									)
							+ (windSpeedE + deltaValue)
							;

					resultDelta = deltaValue *
							transMatrix.getTransitionMatrix()
							.coeff(GliderVarioStatus::STATUS_IND_SPEED_GROUND_E,GliderVarioStatus::STATUS_IND_WIND_SPEED_E);
					deltaResult = orgResult + resultDelta;

					EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " windSpeedN delta = " << deltaValue <<
							" at heading   = " << heading <<
							" trueAirSpeed = " << trueAirSpeed <<
//							" accelHeading = " << accelHeading <<
							" windSpeedE   = " << windSpeedE <<
							" time = " << t;
					st1.windSpeedEast = windSpeedE;

				}

//                }
            }
        }
    }

}

TEST_F(TransitionMatrixTest, TrueAirSpeed) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType t = 0.01f; t<=1.3f; t+=0.34f  ) {
        for (FloatType trueAirSpeed = 0.0f ; trueAirSpeed <= 50.0f; trueAirSpeed += 8.67f) {
            for (FloatType accelHeading = 0.0f; accelHeading <= 2.0f; accelHeading += 0.67f) {

                st1.trueAirSpeed = trueAirSpeed;
                st1.accelHeading = accelHeading;

                transMatrix.updateStatus(st1,st2,t);

                FloatType expectResult =
                        trueAirSpeed + accelHeading*t
                        ;

                EXPECT_NEAR (st2.trueAirSpeed,expectResult,0.00001f) <<
                        " at trueAirSpeed = " << trueAirSpeed <<
                        " accelHeading = " << accelHeading <<
                        " time = " << t;

                // Test the coefficients in the matrix as derivatives.
                FloatType orgResult = expectResult;
                FloatType resultDelta;
                FloatType deltaResult;
                FloatType deltaValue;

                // Modify the TAS
                deltaValue = 1.0f;
                st1.trueAirSpeed = trueAirSpeed + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        (trueAirSpeed+ deltaValue) + accelHeading*t
                        ;

                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_TAS);
                deltaResult = orgResult + resultDelta;

                EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " TAS delta = " << deltaValue <<
                        " at trueAirSpeed = " << trueAirSpeed <<
                        " accelHeading = " << accelHeading <<
                        " time = " << t;
                st1.trueAirSpeed = trueAirSpeed;

                // Modify the acceleration
                deltaValue = 1.0f;
                st1.accelHeading = accelHeading + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        trueAirSpeed + (accelHeading + deltaValue)*t
                        ;

                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_TAS,GliderVarioStatus::STATUS_IND_ACC_HEADING);
                deltaResult = orgResult + resultDelta;

                EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " aceleration delta = " << deltaValue <<
                        " at trueAirSpeed = " << trueAirSpeed <<
                        " accelHeading = " << accelHeading <<
                        " time = " << t;
                st1.accelHeading = accelHeading;

            }

        }
    }

}

TEST_F(TransitionMatrixTest, RateOfSink) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
        for (FloatType trueAirSpeed = 10.0f ; trueAirSpeed <= 50.0f; trueAirSpeed += 8.67f) {
            for (FloatType accelHeading = -1.0f; accelHeading <= 1.0f; accelHeading += 0.27f) {

                st1.trueAirSpeed = trueAirSpeed;
                st1.accelHeading = accelHeading;

                transMatrix.updateStatus(st1,st2,0.1f);

                FloatType expectResult =
                        trueAirSpeed * accelHeading / GRAVITY;
                        ;

                EXPECT_NEAR (st2.rateOfSink,expectResult,0.00001f) <<
                        " at trueAirSpeed = " << trueAirSpeed <<
                        " accelHeading = " << accelHeading ;

                // Test the coefficients in the matrix as derivatives.
                FloatType orgResult = expectResult;
                FloatType resultDelta;
                FloatType deltaResult;
                FloatType deltaValue;

                // Modify the TAS
                deltaValue = 1.0f;
                st1.trueAirSpeed = trueAirSpeed + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        (trueAirSpeed + deltaValue) * accelHeading / GRAVITY
                        ;

                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_TAS);
                deltaResult = orgResult + resultDelta;

                EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " TAS delta = " << deltaValue <<
                        " at trueAirSpeed = " << trueAirSpeed <<
                        " accelHeading = " << accelHeading ;
                st1.trueAirSpeed = trueAirSpeed;

                // Modify the acceleration
                deltaValue = 1.0f;
                st1.accelHeading = accelHeading + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        trueAirSpeed * (accelHeading + deltaValue) / GRAVITY
                        ;

                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_RATE_OF_SINK,GliderVarioStatus::STATUS_IND_ACC_HEADING);
                deltaResult = orgResult + resultDelta;

                EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " aceleration delta = " << deltaValue <<
                        " at trueAirSpeed = " << trueAirSpeed <<
                        " accelHeading = " << accelHeading;
                st1.accelHeading = accelHeading;

            }
    }

}

TEST_F(TransitionMatrixTest, VerticalSpeed) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType t = 0.01f; t<=1.3f; t+=0.34f  ) {
        for (FloatType verticalSpeed = -10.0f ; verticalSpeed <= 10.0f; verticalSpeed += 1.67f) {
            for (FloatType accelVertical = -2.0f; accelVertical <= 2.0f; accelVertical += 0.67f) {

                st1.verticalSpeed = verticalSpeed;
                st1.accelVertical = accelVertical;

                transMatrix.updateStatus(st1,st2,t);

                FloatType expectResult =
                        verticalSpeed + accelVertical*t
                        ;

                EXPECT_NEAR (st2.verticalSpeed,expectResult,0.00001f) <<
                        " at verticalSpeed = " << verticalSpeed <<
                        " accelVertical = " << accelVertical <<
                        " time = " << t;

                // Test the coefficients in the matrix as derivatives.
                FloatType orgResult = expectResult;
                FloatType resultDelta;
                FloatType deltaResult;
                FloatType deltaValue;

                // Modify the Vertical Speed
                deltaValue = 1.0f;
                st1.verticalSpeed = verticalSpeed + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        (verticalSpeed+ deltaValue) + accelVertical*t
                        ;

                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED);
                deltaResult = orgResult + resultDelta;

                EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " Vertical speed delta = " << deltaValue <<
                        " at verticalSpeed = " << verticalSpeed <<
                        " accelVertical = " << accelVertical <<
                        " time = " << t;
                st1.verticalSpeed = verticalSpeed;

                // Modify the acceleration
                deltaValue = 1.0f;
                st1.accelVertical = accelVertical + deltaValue;
                // transMatrix.updateStatus(st1,st2,t);
                expectResult =
                        verticalSpeed + (accelVertical + deltaValue)*t
                        ;

                resultDelta = deltaValue *
                        transMatrix.getTransitionMatrix()
                        .coeff(GliderVarioStatus::STATUS_IND_VERTICAL_SPEED,GliderVarioStatus::STATUS_IND_ACC_VERTICAL);
                deltaResult = orgResult + resultDelta;

                EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " aceleration delta = " << deltaValue <<
                        " at verticalSpeed = " << verticalSpeed <<
                        " accelVertical = " << accelVertical <<
                        " time = " << t;
                st1.accelVertical = accelVertical;

            }

        }
    }

}

TEST_F(TransitionMatrixTest, ThermalSpeed) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType verticalSpeed = -10.0f ; verticalSpeed <= 10.0f; verticalSpeed += 1.67f) {
        for (FloatType rateOfSink = -10.0f; rateOfSink <= 10.0f; rateOfSink += 1.59f) {

            st1.verticalSpeed = verticalSpeed;
            st1.rateOfSink = rateOfSink;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    verticalSpeed - rateOfSink;
                    ;

            EXPECT_NEAR (st2.thermalSpeed,expectResult,0.00001f) <<
                    " at verticalSpeed = " << verticalSpeed <<
                    " rateOfSink = " << rateOfSink ;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

/* Coefficients are set 0.
 *            // Modify the Vertical Speed
 *            deltaValue = 1.0f;
 *            st1.verticalSpeed = verticalSpeed + deltaValue;
 *            // transMatrix.updateStatus(st1,st2,t);
 *            expectResult =
 *                    (verticalSpeed+ deltaValue) - rateOfSink
 *                    ;
 *
 *            resultDelta = deltaValue *
 *                    transMatrix.getTransitionMatrix()
 *                    .coeff(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_VERTICAL_SPEED);
 *            deltaResult = orgResult + resultDelta;
 *
 *            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " Vertical speed delta = " << deltaValue <<
 *                    " at verticalSpeed = " << verticalSpeed <<
 *                    " rateOfSink = " << rateOfSink;
 *            st1.verticalSpeed = verticalSpeed;
 *
 *            // Modify the rate of sink
 *            deltaValue = 1.0f;
 *            st1.rateOfSink = rateOfSink + deltaValue;
 *            // transMatrix.updateStatus(st1,st2,t);
 *            expectResult =
 *                    verticalSpeed - (rateOfSink + deltaValue)
 *                    ;
 *
 *            resultDelta = deltaValue *
 *                    transMatrix.getTransitionMatrix()
 *                    .coeff(GliderVarioStatus::STATUS_IND_THERMAL_SPEED,GliderVarioStatus::STATUS_IND_RATE_OF_SINK);
 *            deltaResult = orgResult + resultDelta;
 *
 *            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " rateOfSink delta = " << deltaValue <<
 *                    " at verticalSpeed = " << verticalSpeed <<
 *                    " rateOfSink = " << rateOfSink;
 *            st1.rateOfSink = rateOfSink;
 */
        }

    }

}

TEST_F(TransitionMatrixTest, AccHeading) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType accHeading = -10.0f ; accHeading <= 10.0f; accHeading += 1.67f) {

            st1.accelHeading = accHeading;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    accHeading * 0.9f;

            EXPECT_NEAR (st2.accelHeading,expectResult,0.00001f) <<
                    " at accHeading = " << accHeading;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the acceleration
            deltaValue = 1.0f;
            st1.accelHeading = orgResult + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (orgResult+ deltaValue * 0.9f)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_ACC_HEADING,GliderVarioStatus::STATUS_IND_ACC_HEADING);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " acceleration delta = " << deltaValue <<
                    " at orgResult = " << orgResult;
            st1.accelHeading = orgResult;

        }

}

/* Cross acceleration is no longer considered in the model
TEST_F(TransitionMatrixTest, AccCross) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType accCross = -10.0f ; accCross <= 10.0f; accCross += 1.67f) {

            st1.accelCross = accCross;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    accCross;

            EXPECT_NEAR (st2.accelCross,expectResult,0.00001f) <<
                    " at accCross = " << accCross;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the acceleration
            deltaValue = 1.0f;
            st1.accelCross = accCross + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (accCross+ deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_ACC_CROSS,GliderVarioStatus::STATUS_IND_ACC_CROSS);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " acceleration delta = " << deltaValue <<
                    " at accCross = " << accCross;
            st1.accelCross = accCross;

        }

}
*/

TEST_F(TransitionMatrixTest, AccVertical) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType accVertical = -10.0f ; accVertical <= 10.0f; accVertical += 1.67f) {

            st1.accelVertical = accVertical;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    accVertical * 0.9f;

            EXPECT_NEAR (st2.accelVertical,expectResult,0.00001f) <<
                    " at accVertical = " << accVertical;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the acceleration
            deltaValue = 1.0f;
            st1.accelCross = orgResult + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (orgResult+ deltaValue * 0.9f)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_ACC_VERTICAL,GliderVarioStatus::STATUS_IND_ACC_VERTICAL);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " acceleration delta = " << deltaValue <<
                    " at accVertical = " << accVertical;
            st1.accelVertical = orgResult;

        }

}

TEST_F(TransitionMatrixTest, RotationX) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType rotationX = -10.0f ; rotationX <= 10.0f; rotationX += 1.67f) {

            st1.rollRateX = rotationX;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    rotationX;

            EXPECT_NEAR (st2.rollRateX,expectResult,0.00001f) <<
                    " at rotationX = " << rotationX;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the rotation rate
            deltaValue = 1.0f;
            st1.rollRateX = rotationX + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (rotationX+ deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_ROTATION_X,GliderVarioStatus::STATUS_IND_ROTATION_X);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " roll rate delta = " << deltaValue <<
                    " at rotationX = " << rotationX;
            st1.rollRateX = rotationX;

        }

}

TEST_F(TransitionMatrixTest, RotationY) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType rotationY = -10.0f ; rotationY <= 10.0f; rotationY += 1.67f) {

            st1.pitchRateY = rotationY;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    rotationY;

            EXPECT_NEAR (st2.pitchRateY,expectResult,0.00001f) <<
                    " at rotationY = " << rotationY;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the rotation rate
            deltaValue = 1.0f;
            st1.pitchRateY = rotationY + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (rotationY+ deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_ROTATION_Y,GliderVarioStatus::STATUS_IND_ROTATION_Y);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " pitch rate delta = " << deltaValue <<
                    " at rotationY = " << rotationY;
            st1.pitchRateY = rotationY;

        }

}

TEST_F(TransitionMatrixTest, RotationZ) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType rotationZ = -10.0f ; rotationZ <= 10.0f; rotationZ += 1.67f) {

            st1.yawRateZ = rotationZ;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    rotationZ;

            EXPECT_NEAR (st2.yawRateZ,expectResult,0.00001f) <<
                    " at rotationZ = " << rotationZ;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the rotation rate
            deltaValue = 1.0f;
            st1.yawRateZ = rotationZ + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (rotationZ+ deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_ROTATION_Z,GliderVarioStatus::STATUS_IND_ROTATION_Z);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " yaw rate delta = " << deltaValue <<
                    " at rotationZ = " << rotationZ;
            st1.yawRateZ = rotationZ;

        }

}

TEST_F(TransitionMatrixTest, GyroBiasX) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType gyroBiasX = -10.0f ; gyroBiasX <= 10.0f; gyroBiasX += 1.67f) {

            st1.gyroBiasX = gyroBiasX;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    gyroBiasX;

            EXPECT_NEAR (st2.gyroBiasX,expectResult,0.00001f) <<
                    " at gyroBiasX = " << gyroBiasX;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the gyro bias
            deltaValue = 1.0f;
            st1.gyroBiasX = gyroBiasX + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (gyroBiasX+ deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,GliderVarioStatus::STATUS_IND_GYRO_BIAS_X);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " gyro bias delta = " << deltaValue <<
                    " at gyroBiasX = " << gyroBiasX;
            st1.gyroBiasX = gyroBiasX;

        }

}

TEST_F(TransitionMatrixTest, GyroBiasY) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType gyroBiasY = -10.0f ; gyroBiasY <= 10.0f; gyroBiasY += 1.67f) {

            st1.gyroBiasY = gyroBiasY;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    gyroBiasY;

            EXPECT_NEAR (st2.gyroBiasY,expectResult,0.00001f) <<
                    " at gyroBiasY = " << gyroBiasY;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the gyro bias
            deltaValue = 1.0f;
            st1.gyroBiasY = gyroBiasY + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (gyroBiasY+ deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " gyro bias delta = " << deltaValue <<
                    " at gyroBiasY = " << gyroBiasY;
            st1.gyroBiasY = gyroBiasY;

        }

}

TEST_F(TransitionMatrixTest, GyroBiasZ) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType gyroBiasZ = -10.0f ; gyroBiasZ <= 10.0f; gyroBiasZ += 1.67f) {

            st1.gyroBiasZ = gyroBiasZ;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    gyroBiasZ;

            EXPECT_NEAR (st2.gyroBiasZ,expectResult,0.00001f) <<
                    " at gyroBiasZ = " << gyroBiasZ;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the gyro bias
            deltaValue = 1.0f;
            st1.gyroBiasZ = gyroBiasZ + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (gyroBiasZ+ deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " gyro bias delta = " << deltaValue <<
                    " at gyroBiasZ = " << gyroBiasZ;
            st1.gyroBiasZ = gyroBiasZ;

        }

}

TEST_F(TransitionMatrixTest, MagneticDeclination) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType magneticDeclination = -10.0f ; magneticDeclination <= 10.0f; magneticDeclination += 1.67f) {

            st1.magneticDeclination = magneticDeclination;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    magneticDeclination;

            EXPECT_NEAR (st2.magneticDeclination,expectResult,0.000001f) <<
                    " at magneticDeclination = " << magneticDeclination;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the declination
            deltaValue = 1.0f;
            st1.magneticDeclination = magneticDeclination + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (magneticDeclination+ deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION,GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " declination delta = " << deltaValue <<
                    " at magneticDeclination = " << magneticDeclination;
            st1.magneticDeclination = magneticDeclination;

        }

}

TEST_F(TransitionMatrixTest, MagneticInclination) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType magneticInclination = -10.0f ; magneticInclination <= 10.0f; magneticInclination += 1.67f) {

            st1.magneticInclination = magneticInclination;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    magneticInclination;

            EXPECT_NEAR (st2.magneticInclination,expectResult,0.000001f) <<
                    " at magneticInclination = " << magneticInclination;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the declination
            deltaValue = 1.0f;
            st1.magneticDeclination = magneticInclination + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (magneticInclination + deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION,GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " declination delta = " << deltaValue <<
                    " at magneticInclination = " << magneticInclination;
            st1.magneticInclination = magneticInclination;

        }

}

TEST_F(TransitionMatrixTest, CompassDeviationX) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType compassDeviationX = -10.0f ; compassDeviationX <= 10.0f; compassDeviationX += 1.67f) {

            st1.compassDeviationX = compassDeviationX;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    compassDeviationX;

            EXPECT_NEAR (st2.compassDeviationX,expectResult,0.0000001f) <<
                    " at compassDeviationX = " << compassDeviationX;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the deviation
            deltaValue = 1.0f;
            st1.compassDeviationX = compassDeviationX + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (compassDeviationX + deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " deviation delta = " << deltaValue <<
                    " at compassDeviationX = " << compassDeviationX;
            st1.compassDeviationX = compassDeviationX;

        }

}

TEST_F(TransitionMatrixTest, CompassDeviationY) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType compassDeviationY = -10.0f ; compassDeviationY <= 10.0f; compassDeviationY += 1.67f) {

            st1.compassDeviationY = compassDeviationY;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    compassDeviationY;

            EXPECT_NEAR (st2.compassDeviationY,expectResult,0.0000001f) <<
                    " at compassDeviationY = " << compassDeviationY;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the deviation
            deltaValue = 1.0f;
            st1.compassDeviationY = compassDeviationY + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (compassDeviationY + deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " deviation delta = " << deltaValue <<
                    " at compassDeviationY = " << compassDeviationY;
            st1.compassDeviationX = compassDeviationY;

        }

}

TEST_F(TransitionMatrixTest, CompassDeviationZ) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: Heading, yaw rate around the z axis
    for (FloatType compassDeviationZ = -10.0f ; compassDeviationZ <= 10.0f; compassDeviationZ += 1.67f) {

            st1.compassDeviationZ = compassDeviationZ;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    compassDeviationZ;

            EXPECT_NEAR (st2.compassDeviationZ,expectResult,0.0000001f) <<
                    " at compassDeviationZ = " << compassDeviationZ;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the deviation
            deltaValue = 1.0f;
            st1.compassDeviationZ = compassDeviationZ + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (compassDeviationZ + deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z,GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " deviation delta = " << deltaValue <<
                    " at compassDeviationZ = " << compassDeviationZ;
            st1.compassDeviationX = compassDeviationZ;

        }

}

TEST_F(TransitionMatrixTest, WindSpeedN) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: wind speed North
    for (FloatType windSpeedNorth = -10.0f ; windSpeedNorth <= 10.0f; windSpeedNorth += 1.67f) {

            st1.windSpeedNorth = windSpeedNorth;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    windSpeedNorth;

            EXPECT_NEAR (st2.windSpeedNorth,expectResult,0.0000001f) <<
                    " at windSpeedNorth = " << windSpeedNorth;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the deviation
            deltaValue = 1.0f;
            st1.windSpeedNorth = windSpeedNorth + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (windSpeedNorth + deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_WIND_SPEED_N,GliderVarioStatus::STATUS_IND_WIND_SPEED_N);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " deviation delta = " << deltaValue <<
                    " at windSpeedNorth = " << windSpeedNorth;
            st1.windSpeedNorth = windSpeedNorth;

        }

}

TEST_F(TransitionMatrixTest, QFF) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: QFF
    for (FloatType qff = 950.0f ; qff <= 1030.0f; qff += 16.7f) {

            st1.qff = qff;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    qff;

            EXPECT_NEAR (st2.qff,expectResult,0.0000001f) <<
                    " at qff = " << qff;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the QFF
            deltaValue = 1.0f;
            st1.qff = qff + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (qff + deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " QFF delta = " << deltaValue <<
                    " at qff = " << qff;
            st1.qff = qff;

        }

}

TEST_F(TransitionMatrixTest, LastPressure) {

    // Test the result for a given combination of input values
    // and a number of time differences
    // input values are: last pressure
    for (FloatType lastPressure = 500.0f ; lastPressure <= 1030.0f; lastPressure += 16.7f) {

            st1.lastPressure = lastPressure;

            transMatrix.updateStatus(st1,st2,0.1f);

            FloatType expectResult =
                    lastPressure;

            EXPECT_NEAR (st2.lastPressure,expectResult,0.0000001f) <<
                    " at lastPressure = " << lastPressure;

            // Test the coefficients in the matrix as derivatives.
            FloatType orgResult = expectResult;
            FloatType resultDelta;
            FloatType deltaResult;
            FloatType deltaValue;

            // Modify the last pressure
            deltaValue = 1.0f;
            st1.lastPressure = lastPressure + deltaValue;
            // transMatrix.updateStatus(st1,st2,t);
            expectResult =
                    (lastPressure + deltaValue)
                    ;

            resultDelta = deltaValue *
                    transMatrix.getTransitionMatrix()
                    .coeff(GliderVarioStatus::STATUS_IND_LAST_PRESSURE,GliderVarioStatus::STATUS_IND_LAST_PRESSURE);
            deltaResult = orgResult + resultDelta;

            EXPECT_NEAR (expectResult,deltaResult,0.00001f) << " QFF delta = " << deltaValue <<
                    " at lastPressure = " << lastPressure;
            st1.lastPressure = lastPressure;

        }

}

