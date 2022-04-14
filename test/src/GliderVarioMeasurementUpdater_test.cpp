/*
 * GliderVarioMeasurementMatrix_test.cpp
 *
 *  Created on: Feb 14, 2016
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016,2017  Kai Horstmann
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
#include "kalman/GliderVarioMeasurementUpdater.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "util/FastMath.h"
#include "util/RotationMatrix.h"

using namespace openEV;

class MeasurementUpdaterTest :public ::testing::Test {
public:


    GliderVarioTransitionMatrix transMatrix;
    GliderVarioStatus st1,st2;
    GliderVarioMeasurementVector measVect;

    MeasurementUpdaterTest () {
        GliderVarioStatus::StatusCoVarianceType &errCov   = st1.getErrorCovariance_P();
        GliderVarioStatus::StatusCoVarianceType &noiseCov = st1.getSystemNoiseCovariance_Q();

        // Put the Glider VarioMeasureUpdater into test mode
        GliderVarioMeasurementUpdater::setUnitTestMode(true);

        // Initialize the status and covariances

        // The noise covariance is set under the assumption that I run the status propagation in 100ms intervals,
        // i.e. all values per second are divided by 10, and values per minute divided by 600.
        // Mid of Lueneburg airport EDHG
        double latitude = 53.2483824;
        st1.latitude(latitude);
        // 100m initial
        errCov.coeffRef(st1.STATUS_IND_LATITUDE_OFFS,st1.STATUS_IND_LATITUDE_OFFS) = 100.0f * 100.0f;
        // 100 meters per minute
        noiseCov.coeffRef(st1.STATUS_IND_LATITUDE_OFFS,st1.STATUS_IND_LATITUDE_OFFS) = (100.0f* 100.0f) / 600.0f;

        // Mid of Lueneburg airport EDHG
        double longitude = 10.458796;
        st1.longitude(longitude);
        // 100m initial
        errCov.coeffRef(st1.STATUS_IND_LONGITUDE_OFFS,st1.STATUS_IND_LONGITUDE_OFFS) = 100.0f * 100.0f;
        // 100m/min
        noiseCov.coeffRef(st1.STATUS_IND_LONGITUDE_OFFS,st1.STATUS_IND_LONGITUDE_OFFS) = (100.0f * 100.0f) / 600.0f;

        st1.altMSL = 500.0;
        errCov.coeffRef(st1.STATUS_IND_ALT_MSL,st1.STATUS_IND_ALT_MSL) = 100.0 * 100.0;
        // 20m per minute
        noiseCov.coeffRef(st1.STATUS_IND_ALT_MSL,st1.STATUS_IND_ALT_MSL) = (20.0 * 20.0) / 600.0;

        // fly 120 deg course
        st1.heading = 120.0;
        errCov.coeffRef(st1.STATUS_IND_HEADING,st1.STATUS_IND_HEADING) = 60.0 * 60.0;
        // 30 deg per minute
        noiseCov.coeffRef(st1.STATUS_IND_HEADING,st1.STATUS_IND_HEADING) = (20.0 * 20.0) / 600.0;

        st1.pitchAngle = 5.0;
        errCov.coeffRef(st1.STATUS_IND_PITCH,st1.STATUS_IND_PITCH) = 10.0 * 10.0;
        // Pitch does not change so uncontrolled. The consequences would be quite noticeable otherwise :)
        noiseCov.coeffRef(st1.STATUS_IND_PITCH,st1.STATUS_IND_PITCH) = (5.0 * 5.0) / 600;

        st1.rollAngle = 20.0;
        errCov.coeffRef(st1.STATUS_IND_ROLL,st1.STATUS_IND_ROLL) = 10.0 * 10.0;
        // Same with the roll angle
        noiseCov.coeffRef(st1.STATUS_IND_ROLL,st1.STATUS_IND_ROLL) = (10.0 * 10.0) / 600;

        // 30 m/s TAS at no wind
        st1.groundSpeedNorth = 30.0 * FastMath::fastCos(st1.heading);
        errCov.coeffRef(st1.STATUS_IND_SPEED_GROUND_N,st1.STATUS_IND_SPEED_GROUND_N) = 20.0 * 20.0;
        noiseCov.coeffRef(st1.STATUS_IND_SPEED_GROUND_N,st1.STATUS_IND_SPEED_GROUND_N) = (20.0 * 20.0) / 600.0;

        // 30 m/s TAS at no wind
        st1.groundSpeedEast = 30.0 * FastMath::fastSin(st1.heading);
        errCov.coeffRef(st1.STATUS_IND_SPEED_GROUND_E,st1.STATUS_IND_SPEED_GROUND_E) = 20.0 * 20.0;
        noiseCov.coeffRef(st1.STATUS_IND_SPEED_GROUND_E,st1.STATUS_IND_SPEED_GROUND_E) = (20.0 * 20.0) / 600.0;

        st1.trueAirSpeed = 30.0;
        errCov.coeffRef(st1.STATUS_IND_TAS,st1.STATUS_IND_TAS) = 20.0 * 20.0;
        noiseCov.coeffRef(st1.STATUS_IND_TAS,st1.STATUS_IND_TAS) = (20.0 * 20.0) / 600.0;

        st1.rateOfSink = 0.0;
        errCov.coeffRef(st1.STATUS_IND_RATE_OF_SINK,st1.STATUS_IND_RATE_OF_SINK) = 2.0 * 2.0;
        noiseCov.coeffRef(st1.STATUS_IND_RATE_OF_SINK,st1.STATUS_IND_RATE_OF_SINK) = (2.0 * 2.0) / 600.0;

        st1.verticalSpeed = -1.2;
        errCov.coeffRef(st1.STATUS_IND_VERTICAL_SPEED,st1.STATUS_IND_VERTICAL_SPEED) = 5.0 * 5.0;
        noiseCov.coeffRef(st1.STATUS_IND_VERTICAL_SPEED,st1.STATUS_IND_VERTICAL_SPEED) = (2.0 * 2.0) / 600.0;

        st1.thermalSpeed = -1.2;
        errCov.coeffRef(st1.STATUS_IND_THERMAL_SPEED,st1.STATUS_IND_THERMAL_SPEED) = 5.0 * 5.0;
        noiseCov.coeffRef(st1.STATUS_IND_THERMAL_SPEED,st1.STATUS_IND_THERMAL_SPEED) = (2.0 * 2.0) / 600.0;

        st1.accelHeading = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ACC_HEADING,st1.STATUS_IND_ACC_HEADING) = 1.0;
        noiseCov.coeffRef(st1.STATUS_IND_ACC_HEADING,st1.STATUS_IND_ACC_HEADING) = 1.0 / 10.0;

        st1.accelCross = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ACC_CROSS,st1.STATUS_IND_ACC_CROSS) = 1.0;
        noiseCov.coeffRef(st1.STATUS_IND_ACC_CROSS,st1.STATUS_IND_ACC_CROSS) = 1.0 / 10.0;

        st1.accelVertical = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ACC_VERTICAL,st1.STATUS_IND_ACC_VERTICAL) = 1.0;
        noiseCov.coeffRef(st1.STATUS_IND_ACC_VERTICAL,st1.STATUS_IND_ACC_VERTICAL) = 1.0 / 10.0;

        st1.rollRateX = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ROTATION_X,st1.STATUS_IND_ROTATION_X) = 10.0 * 10.0;
        noiseCov.coeffRef(st1.STATUS_IND_ROTATION_X,st1.STATUS_IND_ROTATION_X) = 20.0 * 20.0 / 10.0;

        st1.pitchRateY = 0.0;
        errCov.coeffRef(st1.STATUS_IND_ROTATION_Y,st1.STATUS_IND_ROTATION_Y) = 10.0 * 10;
        noiseCov.coeffRef(st1.STATUS_IND_ROTATION_Y,st1.STATUS_IND_ROTATION_Y) = 20.0 * 20.0 / 10.0;

        // turning 1deg/s clock wise
        st1.yawRateZ = 15.0;
        errCov.coeffRef(st1.STATUS_IND_ROTATION_Z,st1.STATUS_IND_ROTATION_Z) = 10.0 * 10.0;
        noiseCov.coeffRef(st1.STATUS_IND_ROTATION_Z,st1.STATUS_IND_ROTATION_Z) = 20 * 20 / 10.0;

        st1.gyroBiasX = 2.0;
        errCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_X,st1.STATUS_IND_GYRO_BIAS_X) = 10.0 * 10.0;
        // Gyro bias should drift rather slow
        noiseCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_X,st1.STATUS_IND_GYRO_BIAS_X) = 1.0 / 600.0f;

        st1.gyroBiasY = -3.0;
        errCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_Y,st1.STATUS_IND_GYRO_BIAS_Y) = 10.0 * 10.0;
        // Gyro bias should drift rather slow
        noiseCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_Y,st1.STATUS_IND_GYRO_BIAS_Y) = 1.0 / 600.0f;

        st1.gyroBiasZ = 4.5;
        errCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_Z,st1.STATUS_IND_GYRO_BIAS_Z) = 10.0 * 10.0;
        // Gyro bias should drift rather slow
        noiseCov.coeffRef(st1.STATUS_IND_GYRO_BIAS_Z,st1.STATUS_IND_GYRO_BIAS_Z) = 1.0 / 600.0f;

        st1.magneticDeclination = -2.0;
        errCov.coeffRef(st1.STATUS_IND_MAGNETIC_DECLINATION,st1.STATUS_IND_MAGNETIC_DECLINATION) = 100.0;
        noiseCov.coeffRef(st1.STATUS_IND_MAGNETIC_DECLINATION,st1.STATUS_IND_MAGNETIC_DECLINATION) = 0.2*0.2 / 600.0;

        st1.magneticInclination = MAG_INCLINATION;
        errCov.coeffRef(st1.STATUS_IND_MAGNETIC_INCLINATION,st1.STATUS_IND_MAGNETIC_INCLINATION) = 100.0f;
        noiseCov.coeffRef(st1.STATUS_IND_MAGNETIC_INCLINATION,st1.STATUS_IND_MAGNETIC_INCLINATION) = 0.2*0.2 / 600.0;

        st1.compassDeviationX = 5.0;
        // Assume Micro-Tesla, and a max. strength in the magnitute od=f the Earth magnet field
        errCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_X,st1.STATUS_IND_COMPASS_DEVIATION_X) = 50.0*50.0;
        // The deviation should fluctuate sloooowly.
        noiseCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_X,st1.STATUS_IND_COMPASS_DEVIATION_X) = 0.5*0.5 / 600.0f;

        st1.compassDeviationY = 7.0;
        // Assume Micro-Tesla, and a max. strength in the magnitute od=f the Earth magnet field
        errCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_Y,st1.STATUS_IND_COMPASS_DEVIATION_Y) = 50.0*50.0;
        // The deviation should fluctuate sloooowly.
        noiseCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_Y,st1.STATUS_IND_COMPASS_DEVIATION_Y) = 0.5*0.5 / 600.0f;

        st1.compassDeviationZ = 13.0;
        // Assume Micro-Tesla, and a max. strength in the magnitute od=f the Earth magnet field
        errCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_Z,st1.STATUS_IND_COMPASS_DEVIATION_Z) = 50.0*50.0;
        // The deviation should fluctuate sloooowly.
        noiseCov.coeffRef(st1.STATUS_IND_COMPASS_DEVIATION_Z,st1.STATUS_IND_COMPASS_DEVIATION_Z) = 0.5*0.5 / 600.0f;

        st1.windSpeedNorth = 4.8;
        errCov.coeffRef(st1.STATUS_IND_WIND_SPEED_N,st1.STATUS_IND_WIND_SPEED_N) = 20.0*20.0;
        // I am not aiming to capture the slightest gusts, but be responsive, 1 m/s per 10sec
        noiseCov.coeffRef(st1.STATUS_IND_WIND_SPEED_N,st1.STATUS_IND_WIND_SPEED_N) = 2.0*2.0 / 100.0;

        st1.windSpeedEast = 2.5;
        errCov.coeffRef(st1.STATUS_IND_WIND_SPEED_E,st1.STATUS_IND_WIND_SPEED_E) = 20.0*20.0;
        // I am not aiming to capture the slightest gusts, but be responsive, 5 m/s per 10sec
        noiseCov.coeffRef(st1.STATUS_IND_WIND_SPEED_E,st1.STATUS_IND_WIND_SPEED_E) = 5.0*5.0 / 100.0;

        st1.qff = 1010.0;
        errCov.coeffRef(st1.STATUS_IND_QFF,st1.STATUS_IND_QFF) = 100.0*100.0;
        // the pressure changes sloooowly
        noiseCov.coeffRef(st1.STATUS_IND_QFF,st1.STATUS_IND_QFF) = 1 / 600.0f;

        st1.lastPressure = st1.qff - st1.altMSL / (8.0/100.0);
        // Variance remains constant. This is only a fudge variable which is not going into any other
        // status propagation calculation.
        errCov.coeffRef(st1.STATUS_IND_LAST_PRESSURE,st1.STATUS_IND_LAST_PRESSURE) = 1.0f;
        // Therefore There is no noise. Otherwise the variance woud increase indefinitely.
        noiseCov.coeffRef(st1.STATUS_IND_LAST_PRESSURE,st1.STATUS_IND_LAST_PRESSURE) = 0.0f;


        // Now run this initial model for 10 sec. at 0.1 sec. interval.
        for (int i = 0; i<50; i++) {
            transMatrix.updateStatus(st1,st2,0.1f);
            transMatrix.updateStatus(st2,st1,0.1f);
        }

        
    }

    // Calculates the pressure at a given altitude based on the pressure at the base altitude QFF
    FloatType calcPressure (FloatType altMSL, FloatType qff, FloatType measuredTemp) {

    	static FloatType constexpr tempLapse = -1.0/100.0;

    	static FloatType constexpr exponent = (GRAVITY * M) / (R * tempLapse);


    	FloatType measuredTempK = measuredTemp + CtoK;

    	FloatType base = (measuredTempK - altMSL * tempLapse) / measuredTempK;

    	FloatType factor = powf(base,exponent);

    	FloatType calculatedPressure = qff * factor;

    	return calculatedPressure;

    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename T>
class Inverse2x2MatrixTest :public ::testing::Test {
public:
	void executeTest(){
		Eigen::SparseMatrix <T> orgMatrix(2,2);
		Eigen::SparseMatrix <T> invMatrix(2,2);
		Eigen::SparseMatrix <T> orgTimesInvMatrix(2,2);

		orgMatrix.reserve(4);
		invMatrix.reserve(4);
		orgTimesInvMatrix.reserve(4);
		uint32_t numTests=0, numInvalidInverse=0, num0sChecked=0, num1sChecked=0, num0TestFailed=0, num1TestFailed=0;
		double sum0Deviation=0.0, sum1Deviation=0.0;
		T max0Deviation=0.0, max1Deviation=0.0;

		for (T a00 = -1.0; a00 < 0.0; a00 += 1.0/7.0) {
			orgMatrix.coeffRef(0, 0) = a00;
			for (T a01 = 2.0; a01 < 3.0; a01 += 1.0/9.3) {
				orgMatrix.coeffRef(0, 1) = a01;
					for (T a10 = 50.0; a10 < 70.0; a10 += 10.0/4.6) {
						orgMatrix.coeffRef(1, 0) = a10;
						for (T a11 = -90.0; a11 < -70.0; a11 += 10.0/3.7) {
							orgMatrix.coeffRef(1, 1) = a11;

							++numTests;
							if (!GliderVarioMeasurementUpdater::calcInverse2D(invMatrix, orgMatrix)){
								//std::cout << "Matrix " << std::endl << std::setw(10)
								//		<< orgMatrix << std::endl
								//		<< "is not singular."
								//		<< std::endl;
								++numInvalidInverse;
							} else {

								orgTimesInvMatrix = orgMatrix * invMatrix;

								for (int x = 0; x < 2; x++) {
									for (int y = 0; y < 2; y++) {
										if (x==y){
											++num1sChecked;
											sum1Deviation+=(std::abs(orgTimesInvMatrix.coeff(x,y) - 1.0));
											max1Deviation=std::max(max1Deviation,std::abs(orgTimesInvMatrix.coeff(x,y) - T(1.0)));

											EXPECT_NEAR (orgTimesInvMatrix.coeff(x,y),T(1.0),T(0.01))
													<< "Test #" << numTests
													<< "\nNumber 1-tests failed = " << ++num1TestFailed
													<< "\norgMatrix ="
													<< printSparseMatrixSimple(orgMatrix)
													<< "\ninvMatrix = " << std::endl
													<< printSparseMatrixSimple(invMatrix)
													<< "\norgTimesInvMatrix = "
													<< printSparseMatrixSimple(orgTimesInvMatrix)
													<< "-----------------------------------------------"
													<< std::endl;
										} else {
											++num0sChecked;
											sum0Deviation+=std::abs(orgTimesInvMatrix.coeff(x,y));
											max0Deviation=std::max(max0Deviation,std::abs(orgTimesInvMatrix.coeff(x,y)));

											EXPECT_NEAR (orgTimesInvMatrix.coeff(x,y),T(0.0),T(0.01))
													<< "Test #" << numTests
													<< "\nNumber 0-tests failed = " << ++num0TestFailed
													<< "\norgMatrix ="
													<< printSparseMatrixSimple(orgMatrix)
													<< "\ninvMatrix = " << std::endl
													<< printSparseMatrixSimple(invMatrix)
													<< "\norgTimesInvMatrix = "
													<< printSparseMatrixSimple(orgTimesInvMatrix)
													<< "-----------------------------------------------"
													<< std::endl;
										}

									}
								}
							}
					}
				}
			}
		}

		std::cout
				<< "\n===============================================\n"
				<< "Number of tests = " << numTests
				<< "\nNumber of invalid inverses = " << numInvalidInverse
				<< "\nNumber of failed 0-tests = " << num0TestFailed
				<< "\nNumber of failed 1-tests = " << num1TestFailed
				<< "\nMaximum 0-deviation = " << max0Deviation
				<< "\nAverage 0-deviation = " << (sum0Deviation/double(num0sChecked))
				<< "\nMaximum 1-deviation = " << max1Deviation
				<< "\nAverage 1-deviation = " << (sum1Deviation/double(num1sChecked))
				<< "\n==============================================="
				<< std::endl;

	}
};

typedef Inverse2x2MatrixTest<FloatType> Inverse2x2MatrixTestFloatType;
TEST_F(Inverse2x2MatrixTestFloatType, FloatTest) {

	executeTest();

}
typedef Inverse2x2MatrixTest<double> Inverse2x2MatrixTestdouble;
TEST_F(Inverse2x2MatrixTestdouble, double) {

	executeTest();

}

template <typename T>
class Inverse3x3MatrixTest :public ::testing::Test {
public:
	void executeTest(){
		Eigen::SparseMatrix <T> orgMatrix(3,3);
		Eigen::SparseMatrix <T> invMatrix(3,3);
		Eigen::SparseMatrix <T> orgTimesInvMatrix(3,3);

		orgMatrix.reserve(9);
		invMatrix.reserve(9);
		orgTimesInvMatrix.reserve(9);
		uint32_t numTests=0, numInvalidInverse=0, num0sChecked=0, num1sChecked=0, num0TestFailed=0, num1TestFailed=0;
		double sum0Deviation=0.0, sum1Deviation=0.0;
		T max0Deviation=0.0, max1Deviation=0.0;

		for (T a00 = -10.0f; a00 < 0.0f; a00 += 10.0f/7.0f) {
			orgMatrix.coeffRef(0, 0) = a00;
			for (T a01 = 20.0f; a01 < 30.0f; a01 += 10.0f/9.3f) {
				orgMatrix.coeffRef(0, 1) = a01;
				for (T a02 = -40.0f; a02 < -30.0f; a02 += 10.0f/9.0f) {
					orgMatrix.coeffRef(0, 2) = a02;
					for (T a10 = 50.0f; a10 < 70.0f; a10 += 10.0f/4.6f) {
						orgMatrix.coeffRef(1, 0) = a10;
						for (T a11 = -90.0f; a11 < -70.0f; a11 += 10.0f/3.7f) {
							orgMatrix.coeffRef(1, 1) = a11;
							for (T a12 = 80.0f; a12 < 100.0f; a12 += 10.0f/2.5f) {
								orgMatrix.coeffRef(1, 2) = a12;
								for (T a20 = -120.0f; a20 < -100.0f; a20 += 10.0f/2.7f) {
									orgMatrix.coeffRef(2, 0) = a20;
									for (T a21 = 130.0f; a21 < 150.0f; a21 += 10.0f/1.44f) {
										orgMatrix.coeffRef(2, 1) = a21;
										for (T a22 = -170.0f; a22 < -150.0f; a22 += 10.0f/1.3f) {
											orgMatrix.coeffRef(2, 2) = a22;

											++numTests;
											if (!GliderVarioMeasurementUpdater::calcInverse3D(invMatrix, orgMatrix)){
												//std::cout << "Matrix " << std::endl << std::setw(10)
												//		<< orgMatrix << std::endl
												//		<< "is not singular."
												//		<< std::endl;
												++numInvalidInverse;
											} else {

												orgTimesInvMatrix = orgMatrix * invMatrix;

												for (int x = 0; x < 3; x++) {
													for (int y = 0; y < 3; y++) {
														if (x==y){
															++num1sChecked;
															sum1Deviation+=(std::abs(orgTimesInvMatrix.coeff(x,y) - 1.0));
															max1Deviation=std::max(max1Deviation,std::abs(orgTimesInvMatrix.coeff(x,y) - T(1.0)));

															EXPECT_NEAR (orgTimesInvMatrix.coeff(x,y),T(1.0),T(0.1))
																	<< "Test #" << numTests
																	<< "\nNumber 1-tests failed = " << ++num1TestFailed
																	<< "\norgMatrix ="
																	<< printSparseMatrixSimple(orgMatrix)
																	<< "\ninvMatrix = " << std::endl
																	<< printSparseMatrixSimple(invMatrix)
																	<< "\norgTimesInvMatrix = "
																	<< printSparseMatrixSimple(orgTimesInvMatrix)
																	<< "-----------------------------------------------"
																	<< std::endl;
														} else {
															++num0sChecked;
															sum0Deviation+=std::abs(orgTimesInvMatrix.coeff(x,y));
															max0Deviation=std::max(max0Deviation,std::abs(orgTimesInvMatrix.coeff(x,y)));

															EXPECT_NEAR (orgTimesInvMatrix.coeff(x,y),T(0.0),T(0.1))
																	<< "Test #" << numTests
																	<< "\nNumber 0-tests failed = " << ++num0TestFailed
																	<< "\norgMatrix ="
																	<< printSparseMatrixSimple(orgMatrix)
																	<< "\ninvMatrix = " << std::endl
																	<< printSparseMatrixSimple(invMatrix)
																	<< "\norgTimesInvMatrix = "
																	<< printSparseMatrixSimple(orgTimesInvMatrix)
																	<< "-----------------------------------------------"
																	<< std::endl;
														}

													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		std::cout
				<< "\n===============================================\n"
				<< "Number of tests = " << numTests
				<< "\nNumber of invalid inverses = " << numInvalidInverse
				<< "\nNumber of failed 0-tests = " << num0TestFailed
				<< "\nNumber of failed 1-tests = " << num1TestFailed
				<< "\nMaximum 0-deviation = " << max0Deviation
				<< "\nAverage 0-deviation = " << (sum0Deviation/T(num0sChecked))
				<< "\nMaximum 1-deviation = " << max1Deviation
				<< "\nAverage 1-deviation = " << (sum1Deviation/T(num1sChecked))
				<< "\n==============================================="
				<< std::endl;
	}
};


typedef Inverse3x3MatrixTest<FloatType> Inverse3x3MatrixTestFloatType;
TEST_F(Inverse3x3MatrixTestFloatType, FloatTest) {

	executeTest();

}
typedef Inverse3x3MatrixTest<double> Inverse3x3MatrixTestdouble;
TEST_F(Inverse3x3MatrixTestdouble, double) {

	executeTest();

}

TEST_F(MeasurementUpdaterTest, Latitude) {

    // Test the result for a given combination of input values
    // and a number of time differences

	// Increase by 5 arc seconds.
    double measLat = st1.latitude() + 5.0 / 3600.0;

    FloatType expectResult = st1.latitudeOffsC;

    GliderVarioMeasurementUpdater::GPSLatitudeUpd(measLat,15.0*15.0/3600.0/3600.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_LATITUDE_OFFS:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),1.0f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

TEST_F(MeasurementUpdaterTest, Longitude) {

    // Test the result for a given combination of input values
    // and a number of time differences

	// Increase by 5 arc seconds.
	//Remember measurement is in degrees, status split into base in arc seconds and offset in meters.
    double measLon = st1.longitude() + 5.0 / 3600.0 ;

    FloatType expectResult = st1.longitudeOffsC;

    GliderVarioMeasurementUpdater::GPSLongitudeUpd(measLon,15.0*15.0/3600.0/3600.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),1.0f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

TEST_F(MeasurementUpdaterTest, GPSAltitude) {

    // Test the result for a given combination of input values
    // and a number of time differences

    FloatType measAlt = st1.altMSL + 10; // Increase by 10 m.

    FloatType expectResult = st1.altMSL;

    GliderVarioMeasurementUpdater::GPSAltitudeUpd(measAlt,25.0*25.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ALT_MSL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),1.0f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

TEST_F(MeasurementUpdaterTest, GPSHeading) {

    // Test the result for a given combination of input values
    // and a number of time differences

    FloatType expectResult = FastMath::fastATan2(st1.groundSpeedEast,st1.groundSpeedNorth);

    FloatType measHeadingGrnd = expectResult + 10.0; // Increase by 10 degrees.

    // approximate derivatives
    FloatType delta = (fabsf(st1.groundSpeedEast) + fabsf(st1.groundSpeedNorth)) / 100.0;
    FloatType expectDiffNorth = (FastMath::fastATan2(st1.groundSpeedEast        ,st1.groundSpeedNorth + delta) - expectResult ) / delta;
    FloatType expectDiffEast  = (FastMath::fastATan2(st1.groundSpeedEast + delta,st1.groundSpeedNorth        ) - expectResult ) / delta;

    GliderVarioMeasurementUpdater::GPSHeadingUpd(measHeadingGrnd,10.0*10.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_SPEED_GROUND_N:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),expectDiffNorth);
            break;

        case GliderVarioStatus::STATUS_IND_SPEED_GROUND_E:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),expectDiffEast);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}

TEST_F(MeasurementUpdaterTest, GPSSpeed) {

    // Test the result for a given combination of input values
    // and a number of time differences

    FloatType expectResult = sqrtf(st1.groundSpeedEast * st1.groundSpeedEast + st1.groundSpeedNorth * st1.groundSpeedNorth);

    FloatType measSpeedGrnd = expectResult + 10.0; // Increase by 10 m/s.

    // approximate derivatives
    FloatType delta = expectResult / 100.0;
    FloatType temp1 =  st1.groundSpeedNorth + delta;
    FloatType expectDiffNorth = (sqrtf(st1.groundSpeedEast * st1.groundSpeedEast + temp1 * temp1) - expectResult ) / delta;
    temp1 =  st1.groundSpeedEast + delta;
    FloatType expectDiffEast = (sqrtf(temp1 * temp1 + st1.groundSpeedNorth * st1.groundSpeedNorth) - expectResult ) / delta;

    GliderVarioMeasurementUpdater::GPSSpeedUpd(measSpeedGrnd,5.0*5.0,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResult);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_SPEED_GROUND_N:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),expectDiffNorth);
            break;

        case GliderVarioStatus::STATUS_IND_SPEED_GROUND_E:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),expectDiffEast);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f);

        }
    }


}


TEST_F(MeasurementUpdaterTest, Accelerometer) {

    // Test the result for a given combination of input values
    // and a number of time differences

    // the plane coordinate system is in direction of the heading. Yaw angle is therefore 0
    RotationMatrix rotMat(0.0f,st1.pitchAngle,st1.rollAngle);
    RotationMatrix3DType& rotMat3D = rotMat.getMatrixGloToPlane();
    // The calculated accelation measuremants include in addition to the actual accelerations of the plane the force of gravity and the
    // centrifugal force of the plane when turning.

    FloatType turnRateRad = st1.yawRateZ * FastMath::degToRad;

    Vector3DType accelVect(
            st1.accelHeading,
            turnRateRad * st1.trueAirSpeed + st1.accelCross,
            st1.accelVertical - st1.gravity);
    Vector3DType calcAccelVect, calcAccelVectIncX, calcAccelVectIncY;
    RotationMatrix rotMatIncX (0.0f,st1.pitchAngle       ,st1.rollAngle + 1.0f);
    RotationMatrix rotMatIncY (0.0f,st1.pitchAngle + 1.0f,st1.rollAngle       );
    // The derivative of cos(0) is 0. Small increments in heading have no effect
    // RotationMatrix rotMatIncZ (1.0f,st1.pitchAngle       ,st1.rollAngle       );


    rotMat.calcWorldVectorToPlaneVector(accelVect,calcAccelVect);
    rotMatIncX.calcWorldVectorToPlaneVector(accelVect,calcAccelVectIncX);
    rotMatIncY.calcWorldVectorToPlaneVector(accelVect,calcAccelVectIncY);

    FloatType expectResultX = calcAccelVect(0);
    FloatType expectResultY = calcAccelVect(1);
    FloatType expectResultZ = calcAccelVect(2);

    FloatType measAccelX = expectResultX + 0.15f; // Increase by 0.1 m/s^2.
    FloatType measAccelY = expectResultY + 0.05f; // Increase by 0.1 m/s^2.
    FloatType measAccelZ = expectResultZ + 0.2f; // Increase by 0.1 m/s^2.

    // Calculate derivatives
    FloatType diffAccelXX = rotMat3D(0,0);
    FloatType diffAccelYX = rotMat3D(1,0);
    FloatType diffAccelZX = rotMat3D(2,0);

    FloatType diffAccelXY = rotMat3D(0,1);
    FloatType diffAccelYY = rotMat3D(1,1);
    FloatType diffAccelZY = rotMat3D(2,1);
    FloatType diffAccelXTAS = rotMat3D(0,1) * st1.yawRateZ * FastMath::degToRad;
    FloatType diffAccelYTAS = rotMat3D(1,1) * st1.yawRateZ * FastMath::degToRad;
    FloatType diffAccelZTAS = rotMat3D(2,1) * st1.yawRateZ * FastMath::degToRad;
    FloatType diffAccelXyawRate = rotMat3D(0,1) * st1.trueAirSpeed * FastMath::degToRad;
    FloatType diffAccelYyawRate = rotMat3D(1,1) * st1.trueAirSpeed * FastMath::degToRad;
    FloatType diffAccelZyawRate = rotMat3D(2,1) * st1.trueAirSpeed * FastMath::degToRad;

    FloatType diffAccelXZ = rotMat3D(0,2);
    FloatType diffAccelYZ = rotMat3D(1,2);
    FloatType diffAccelZZ = rotMat3D(2,2);
    FloatType diffAccelXGravity = -rotMat3D(0,2);
    FloatType diffAccelYGravity = -rotMat3D(1,2);
    FloatType diffAccelZGravity = -rotMat3D(2,2);

    // approximate derivatives
    FloatType diffRollX = calcAccelVectIncX(0) - expectResultX;
    FloatType diffPitchX = calcAccelVectIncY(0) - expectResultX;

    FloatType diffRollY = calcAccelVectIncX(1) - expectResultY;
    FloatType diffPitchY = calcAccelVectIncY(1) - expectResultY;

    FloatType diffRollZ = calcAccelVectIncX(2) - expectResultZ;
    FloatType diffPitchZ = calcAccelVectIncY(2) - expectResultZ;
    // There is no diffYawX because the derivative of 0 deg (remember, acceleration X is along the heading direction, i.e. 0 deg releative to the plane!

    GliderVarioMeasurementUpdater::accelUpd(measAccelX,0.2f*0.2f, measAccelY,0.2f*0.2f, measAccelZ,0.2f*0.2f ,measVect,st1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,expectResultX);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ACC_HEADING:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffAccelXX);
            break;

        case GliderVarioStatus::STATUS_IND_ACC_CROSS:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffAccelXY);
            break;

        case GliderVarioStatus::STATUS_IND_ACC_VERTICAL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffAccelXZ);
            break;

        case GliderVarioStatus::STATUS_IND_TAS:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffAccelXTAS);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Z:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffAccelXyawRate);
            break;

        case GliderVarioStatus::STATUS_IND_GRAVITY:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffAccelXGravity);
            break;

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffRollX);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffPitchX);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f)
			  << " Coefficient with index " << i << " is expected 0.0 but actually is "
			  <<  GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0);

        }
    }

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst2,expectResultY);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ACC_HEADING:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffAccelYX);
            break;

        case GliderVarioStatus::STATUS_IND_ACC_CROSS:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffAccelYY);
            break;

        case GliderVarioStatus::STATUS_IND_ACC_VERTICAL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffAccelYZ);
            break;

        case GliderVarioStatus::STATUS_IND_TAS:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffAccelYTAS);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Z:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffAccelYyawRate);
            break;

        case GliderVarioStatus::STATUS_IND_GRAVITY:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffAccelYGravity);
            break;

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffRollY);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffPitchY);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0);

        }
    }

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst3,expectResultZ);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ACC_HEADING:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffAccelZX);
            break;

        case GliderVarioStatus::STATUS_IND_ACC_CROSS:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffAccelZY);
            break;

        case GliderVarioStatus::STATUS_IND_ACC_VERTICAL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffAccelZZ);
            break;

        case GliderVarioStatus::STATUS_IND_TAS:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffAccelZTAS);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Z:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffAccelZyawRate);
            break;

        case GliderVarioStatus::STATUS_IND_GRAVITY:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffAccelZGravity);
            break;

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffRollZ);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffPitchZ);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0);

        }
    }

}

TEST_F(MeasurementUpdaterTest, Gyro) {

    // Test the result for a given combination of input values
    // and a number of time differences

    // the plane coordinate system is in direction of the heading. Yaw angle is therefore 0
    RotationMatrix rotMat(0.0f,st1.pitchAngle,st1.rollAngle);
    RotationMatrix rotMatIncX (0.0f,st1.pitchAngle       ,st1.rollAngle + 1.0f);
    RotationMatrix rotMatIncY (0.0f,st1.pitchAngle + 1.0f,st1.rollAngle       );
    // The derivative of cos(0) is 0. Small increments in heading have no effect
    // RotationMatrix rotMatIncZ (1.0f,st1.pitchAngle       ,st1.rollAngle       );
    RotationMatrix3DType rotMat3D = rotMat.getMatrixGloToPlane();

    Vector3DType const rotVectWorld (st1.rollRateX,st1.pitchRateY,st1.yawRateZ);
    Vector3DType rotVectPlane;
    Vector3DType rotVectPlaneIncX;
    Vector3DType rotVectPlaneIncY;

    FloatType calcRotRateX, calcRotRateY, calcRotRateZ;

    FloatType diffRotX, diffRotY, diffRotZ, diffRollAngle, diffPitchAngle;

    // The expected/calculated values
    // first the world rotation rates to the view of the plane
    rotMat.calcWorldVectorToPlaneVector(rotVectWorld,rotVectPlane);
    rotMatIncX.calcWorldVectorToPlaneVector(rotVectWorld,rotVectPlaneIncX);
    rotMatIncY.calcWorldVectorToPlaneVector(rotVectWorld,rotVectPlaneIncY);

    calcRotRateX = rotVectPlane(0) + st1.gyroBiasX;
    calcRotRateY = rotVectPlane(1) + st1.gyroBiasY;
    calcRotRateZ = rotVectPlane(2) + st1.gyroBiasZ;

    FloatType measRotRateX = calcRotRateX + 2.5f;
    FloatType measRotRateY = calcRotRateY + 2.5f;
    FloatType measRotRateZ = calcRotRateZ + 2.5f;

    GliderVarioMeasurementUpdater::gyroUpd(measRotRateX,5*5 ,measRotRateY,5*5 ,measRotRateZ,5*5 ,measVect,st1);

    // test of the X gyro
    diffRotX = rotMat3D(0,0);
    diffRotY = rotMat3D(0,1);
    diffRotZ = rotMat3D(0,2);
    diffRollAngle  = rotVectPlaneIncX(0) - rotVectPlane(0);
    diffPitchAngle = rotVectPlaneIncY(0) - rotVectPlane(0);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst1,calcRotRateX);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ROTATION_X:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffRotX);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Y:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffRotY);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Z:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffRotZ);
            break;

        case GliderVarioStatus::STATUS_IND_GYRO_BIAS_X:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),1.0f);
            break;

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffRollAngle);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffPitchAngle);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0);

        }
    }

    // test of the Y gyro
    diffRotX = rotMat3D(1,0);
    diffRotY = rotMat3D(1,1);
    diffRotZ = rotMat3D(1,2);
    diffRollAngle  = rotVectPlaneIncX(1) - rotVectPlane(1);
    diffPitchAngle = rotVectPlaneIncY(1) - rotVectPlane(1);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst2,calcRotRateY);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ROTATION_X:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffRotX);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Y:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffRotY);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Z:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffRotZ);
            break;

        case GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),1.0f);
            break;

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffRollAngle);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),diffPitchAngle);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0);

        }
    }

    // test of the Z gyro
    diffRotX = rotMat3D(2,0);
    diffRotY = rotMat3D(2,1);
    diffRotZ = rotMat3D(2,2);
    diffRollAngle  = rotVectPlaneIncX(2) - rotVectPlane(2);
    diffPitchAngle = rotVectPlaneIncY(2) - rotVectPlane(2);

    EXPECT_EQ (GliderVarioMeasurementUpdater::calculatedValueTst3,calcRotRateZ);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ROTATION_X:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffRotX);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Y:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffRotY);
            break;

        case GliderVarioStatus::STATUS_IND_ROTATION_Z:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffRotZ);
            break;

        case GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),1.0f);
            break;

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffRollAngle);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),diffPitchAngle);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0);

        }
    }

}

TEST_F(MeasurementUpdaterTest, Magnetometer) {

    // Test the result for a given combination of input values
    // and a number of time differences

    // the plane coordinate system is in direction of the heading. Yaw angle is therefore 0
    RotationMatrix rotMat     (st1.heading       ,st1.pitchAngle       ,st1.rollAngle);
    RotationMatrix rotMatIncX (st1.heading       ,st1.pitchAngle       ,st1.rollAngle + 1.0f);
    RotationMatrix rotMatIncY (st1.heading       ,st1.pitchAngle + 1.0f,st1.rollAngle       );
    RotationMatrix rotMatIncZ (st1.heading + 1.0f,st1.pitchAngle       ,st1.rollAngle       );

    RotationMatrix rotMatMagVect        (st1.magneticDeclination       ,st1.magneticInclination       ,0.0f);
    RotationMatrix rotMatMagVectIncDecl (st1.magneticDeclination + 1.0f,st1.magneticInclination       ,0.0f);
    RotationMatrix rotMatMagVectIncIncl (st1.magneticDeclination       ,st1.magneticInclination + 1.0f,0.0f);

    Vector3DType magVectUnit (48.0f,0.0f,0.0f);

    Vector3DType magVect        = rotMat.getMatrixGloToPlane()     * rotMatMagVect.getMatrixPlaneToGlo()        * magVectUnit;
    Vector3DType magVectIncX    = rotMatIncX.getMatrixGloToPlane() * rotMatMagVect.getMatrixPlaneToGlo()        * magVectUnit;
    Vector3DType magVectIncY    = rotMatIncY.getMatrixGloToPlane() * rotMatMagVect.getMatrixPlaneToGlo()        * magVectUnit;
    Vector3DType magVectIncZ    = rotMatIncZ.getMatrixGloToPlane() * rotMatMagVect.getMatrixPlaneToGlo()        * magVectUnit;
    Vector3DType magVectIncDecl = rotMat.getMatrixGloToPlane()     * rotMatMagVectIncDecl.getMatrixPlaneToGlo() * magVectUnit;
    Vector3DType magVectIncIncl = rotMat.getMatrixGloToPlane()     * rotMatMagVectIncIncl.getMatrixPlaneToGlo() * magVectUnit;

    FloatType calcMagX = magVect(0) + st1.compassDeviationX;
    FloatType calcMagY = magVect(1) + st1.compassDeviationY;
    FloatType calcMagZ = magVect(2) + st1.compassDeviationZ;

    // Calculation without difference between calculated and measured values. Otherwise the vector length slightly differs, and expected results slightly differ.

    GliderVarioMeasurementUpdater::compassUpd(calcMagX,calcMagY,calcMagZ,0.5f*0.5f,0.5f*0.5f,0.5f*0.5f,measVect,st1);

    EXPECT_NEAR (GliderVarioMeasurementUpdater::calculatedValueTst1,calcMagX,0.00001f);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),magVectIncX(0) - magVect(0),0.000001f);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),magVectIncY(0) - magVect(0),0.000001f);
            break;

        case GliderVarioStatus::STATUS_IND_HEADING:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),magVectIncZ(0) - magVect(0),0.000001f);
            break;

        case GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),magVectIncDecl(0) - magVect(0),0.000001f);
            break;

        case GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),magVectIncIncl(0) - magVect(0),0.000001f);
            break;

        case GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),1.0f,0.000001f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0);

        }
    }

    EXPECT_NEAR (GliderVarioMeasurementUpdater::calculatedValueTst2,calcMagY,0.00001f);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),magVectIncX(1) - magVect(1),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),magVectIncY(1) - magVect(1),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_HEADING:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),magVectIncZ(1) - magVect(1),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),magVectIncDecl(1) - magVect(1),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),magVectIncIncl(1) - magVect(1),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),1.0f,0.00001f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0);

        }
    }

    EXPECT_NEAR (GliderVarioMeasurementUpdater::calculatedValueTst3,calcMagZ,0.00001f);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_ROLL:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),magVectIncX(2) - magVect(2),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_PITCH:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),magVectIncY(2) - magVect(2),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_HEADING:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),magVectIncZ(2) - magVect(2),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_MAGNETIC_DECLINATION:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),magVectIncDecl(2) - magVect(2),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_MAGNETIC_INCLINATION:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),magVectIncIncl(2) - magVect(2),0.00001f);
            break;

        case GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),1.0f,0.00001f);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst3.coeff(i,0);

        }
    }

    GliderVarioMeasurementUpdater::compassUpd(calcMagX+0.5f,calcMagY+0.5f,calcMagZ+0.5f,0.5f*0.5f,0.5f*0.5f,0.5f*0.5f,measVect,st1);
    GliderVarioMeasurementUpdater::compassUpd(calcMagX+0.5f,calcMagY+0.5f,calcMagZ+0.5f,0.5f*0.5f,0.5f*0.5f,0.5f*0.5f,measVect,st1);
    GliderVarioMeasurementUpdater::compassUpd(calcMagX+0.5f,calcMagY+0.5f,calcMagZ+0.5f,0.5f*0.5f,0.5f*0.5f,0.5f*0.5f,measVect,st1);

}

TEST_F(MeasurementUpdaterTest, StaticPressure) {

	static FloatType constexpr tempLapse = -1.0/100.0;

	// static FloatType constexpr exponent = (GRAVITY * M) / (R * tempLapse);

	FloatType const tempSeaLevel = 20.0;
	FloatType const measuredTemp = tempSeaLevel + st1.altMSL * tempLapse;


	FloatType const calculatedPressure = calcPressure(st1.altMSL,st1.qff,measuredTemp);

	FloatType const factor = calculatedPressure / st1.qff;

	// FloatType baseDiff = (measuredTempK - (st1.altMSL + 10.0) * tempLapse) / measuredTempK;


	FloatType const calculatedPressureDiff = calcPressure(st1.altMSL + 10.0,st1.qff,measuredTemp);
	// FloatType const factorDiff = calculatedPressureDiff/st1.qff;

	FloatType const diffAltMSL = (calculatedPressureDiff - calculatedPressure) / 10.0;

	// this is for debugging only to check the altitude difference per hPa decrease (~8m/hPa or 20ft/hPa at MSL, double that at 5500 m altMSL)


	GliderVarioMeasurementUpdater::staticPressureUpd(calculatedPressure + 0.2,measuredTemp,3*3,measVect,st1);

	EXPECT_NEAR (GliderVarioMeasurementUpdater::calculatedValueTst1,calculatedPressure,0.0001);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_QFF:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst2.coeff(i,0),factor,0.0001);
            break;

        case GliderVarioStatus::STATUS_IND_ALT_MSL:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffAltMSL,0.0001);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0);

        }

    }


}

TEST_F(MeasurementUpdaterTest, DynamicPressure) {

	static FloatType constexpr tempLapse = -1.0/100.0;

	// static FloatType constexpr exponent = (GRAVITY * M) / (R * tempLapse);

	FloatType const tempSeaLevel = 20.0;
	FloatType const measuredTemp = tempSeaLevel + st1.altMSL * tempLapse;


	FloatType const calculatedPressure = calcPressure(st1.altMSL,st1.qff,measuredTemp);

	st1.lastPressure = calculatedPressure;

	const FloatType density = calculatedPressure * (100.0f / Rspec) / (measuredTemp + CtoK);

	const FloatType dynPressure = density * st1.trueAirSpeed * st1.trueAirSpeed / 2.0f;
	const FloatType dynPressure1 = density * (st1.trueAirSpeed+1.0f) * (st1.trueAirSpeed+1.0f) / 2.0f;

	// const FloatType diffDynPressure = density * st1.trueAirSpeed;
	const FloatType diffDynPressure = dynPressure1 - dynPressure;


	GliderVarioMeasurementUpdater::dynamicPressureUpd(dynPressure /*+ 5.0f*/,measuredTemp,3*3,measVect,st1);

	EXPECT_NEAR (GliderVarioMeasurementUpdater::calculatedValueTst1,dynPressure,0.00001);

    for (int i = 0; i < GliderVarioStatus::STATUS_NUM_ROWS; i++) {
        switch (i) {

        case GliderVarioStatus::STATUS_IND_TAS:
            EXPECT_NEAR (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),diffDynPressure,0.00001);
            break;

        default:
            EXPECT_EQ (GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0),0.0f)
              << " Coefficient with index " << i << " is expected 0.0 but actually is "
              <<  GliderVarioMeasurementUpdater::measRowTTst1.coeff(i,0);

        }

    }


}
