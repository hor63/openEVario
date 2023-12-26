/*
 * IMUBase.cpp
 *
 *  Created on: Jul 29, 2021
 *      Author: hor
 */

#include "main/driverBase/IMUBase.h"

namespace openEV {
namespace drivers {

#if defined HAVE_LOG4CXX_H
log4cxx::LoggerPtr IMUBase::logger = nullptr;

#endif


IMUBase::IMUBase(
		char const *driverName,
		char const *description,
		char const *instanceName,
		DriverLibBase &driverLib
		)
: DriverBase {driverName,description,instanceName,driverLib}
{
	for (int i = 0; i < SIZE_SENSOR_DATA_ARRAY; ++i) {
		sensorDataArr[i].accelDataValid = false;
		sensorDataArr[i].accelX = 0.0f;
		sensorDataArr[i].accelY = 0.0f;
		sensorDataArr[i].accelZ = 0.0f;

		sensorDataArr[i].gyroDataValid = false;
		sensorDataArr[i].gyroX = 0.0f;
		sensorDataArr[i].gyroY = 0.0f;
		sensorDataArr[i].gyroZ = 0.0f;

		sensorDataArr[i].magDataValid = false;
		sensorDataArr[i].magX = 0.0f;
		sensorDataArr[i].magY = 0.0f;
		sensorDataArr[i].magZ = 0.0f;
	}
}

IMUBase::~IMUBase() {
}

void IMUBase::applyCalibrationData(){

	readOrCreateConfigValue(*calibrationDataParameters,"magXFactor",	calibrationData.magXFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"magYFactor",	calibrationData.magYFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"magZFactor",	calibrationData.magZFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"magXBias",		calibrationData.magXBias);
	readOrCreateConfigValue(*calibrationDataParameters,"magYBias",		calibrationData.magYBias);
	readOrCreateConfigValue(*calibrationDataParameters,"magZBias",		calibrationData.magZBias);
	readOrCreateConfigValue(*calibrationDataParameters,"magXVariance",	calibrationData.magXVariance);
	readOrCreateConfigValue(*calibrationDataParameters,"magYVariance",	calibrationData.magYVariance);
	readOrCreateConfigValue(*calibrationDataParameters,"magZVariance",	calibrationData.magZVariance);

	readOrCreateConfigValue(*calibrationDataParameters,"gyrXFactor",	calibrationData.gyrXFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"gyrYFactor",	calibrationData.gyrYFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"gyrZFactor",	calibrationData.gyrZFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"gyrXBias",		calibrationData.gyrXBias);
	readOrCreateConfigValue(*calibrationDataParameters,"gyrYBias",		calibrationData.gyrYBias);
	readOrCreateConfigValue(*calibrationDataParameters,"gyrZBias",		calibrationData.gyrZBias);
	readOrCreateConfigValue(*calibrationDataParameters,"gyrXVariance",	calibrationData.gyrXVariance);
	readOrCreateConfigValue(*calibrationDataParameters,"gyrYVariance",	calibrationData.gyrYVariance);
	readOrCreateConfigValue(*calibrationDataParameters,"gyrZVariance",	calibrationData.gyrZVariance);

	readOrCreateConfigValue(*calibrationDataParameters,"accelXBias",	calibrationData.accelXBias);
	readOrCreateConfigValue(*calibrationDataParameters,"accelYBias",	calibrationData.accelYBias);
	readOrCreateConfigValue(*calibrationDataParameters,"accelZBias",	calibrationData.accelZBias);
	readOrCreateConfigValue(*calibrationDataParameters,"accelXFactor",	calibrationData.accelXFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"accelYFactor",	calibrationData.accelYFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"accelZFactor",	calibrationData.accelZFactor);
	readOrCreateConfigValue(*calibrationDataParameters,"accelXVariance",calibrationData.accelXVariance);
	readOrCreateConfigValue(*calibrationDataParameters,"accelYVariance",calibrationData.accelYVariance);
	readOrCreateConfigValue(*calibrationDataParameters,"accelZVariance",calibrationData.accelZVariance);

	readOrCreateConfigValue(*calibrationDataParameters,"gravityValue",	calibrationData.gravity);
	readOrCreateConfigValue(*calibrationDataParameters,"gravityVariance",calibrationData.gravityVariance);

	LOG4CXX_DEBUG (logger, __PRETTY_FUNCTION__ << " Set calibration data for device \"" << instanceName << "\": \n"
			<< "\tmagXFactor		 = " << calibrationData.magXFactor << "\n"
			<< "\tmagYFactor		 = " << calibrationData.magYFactor << "\n"
			<< "\tmagZFactor		 = " << calibrationData.magZFactor << "\n"
			<< "\tmagXBias		 	 = " << calibrationData.magXBias << "\n"
			<< "\tmagYBias			 = " << calibrationData.magYBias << "\n"
			<< "\tmagZBias			 = " << calibrationData.magZBias << "\n"
			<< "\tmagXVariance		 = " << calibrationData.magXVariance << "\n"
			<< "\tmagYVariance		 = " << calibrationData.magYVariance << "\n"
			<< "\tmagZVariance		 = " << calibrationData.magZVariance << "\n"

			<< "\tgyrXFactor		 = " << calibrationData.gyrXFactor << "\n"
			<< "\tgyrYFactor		 = " << calibrationData.gyrYFactor << "\n"
			<< "\tgyrZFactor		 = " << calibrationData.gyrZFactor << "\n"
			<< "\tgyrXBias			 = " << calibrationData.gyrXBias << "\n"
			<< "\tgyrYBias			 = " << calibrationData.gyrYBias << "\n"
			<< "\tgyrZBias			 = " << calibrationData.gyrZBias << "\n"
			<< "\tgyrXVariance		 = " << calibrationData.gyrXVariance << "\n"
			<< "\tgyrYVariance		 = " << calibrationData.gyrYVariance << "\n"
			<< "\tgyrZVariance		 = " << calibrationData.gyrZVariance << "\n"

			<< "\taccelXBias		 = " << calibrationData.accelXBias << "\n"
			<< "\taccelYBias		 = " << calibrationData.accelYBias << "\n"
			<< "\taccelZBias		 = " << calibrationData.accelZBias << "\n"
			<< "\taccelXFactor		 = " << calibrationData.accelXFactor << "\n"
			<< "\taccelYFactor		 = " << calibrationData.accelYFactor << "\n"
			<< "\taccelZFactor		 = " << calibrationData.accelZFactor << "\n"
			<< "\taccelXVariance	 = " << calibrationData.accelXVariance << "\n"
			<< "\taccelYVariance	 = " << calibrationData.accelYVariance << "\n"
			<< "\taccelZVariance	 = " << calibrationData.accelZVariance << "\n"

			<< "\tgravity			 = " << calibrationData.gravity << "\n"
			<< "\tgravityVariance	 = " << calibrationData.gravityVariance << "\n"
			);
}

void IMUBase::driverInit(GliderVarioMainPriv &varioMain) {

}

#define SQUARE(x) ((x)*(x))

void IMUBase::initializeStatusAccel(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain,
		struct SensorData const &sumSensorData,
		int numAccelData
		) {

	// Assume that you are on the ground, but maybe tilted to the side
	// (remember this instrument is primarily for gliders)
	// and slightly pitched up in launch position compared to level flight
	auto avgAccelX = sumSensorData.accelX / float(numAccelData);
	auto avgAccelY = sumSensorData.accelY / float(numAccelData);
	auto avgAccelZ = sumSensorData.accelZ / float(numAccelData);
	auto absoluteAccel = sqrtf(avgAccelX*avgAccelX + avgAccelY*avgAccelY + avgAccelZ*avgAccelZ);

	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__ << " baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgAccelX = " << avgAccelX);
	LOG4CXX_DEBUG(logger,"avgAccelY = " << avgAccelY);
	LOG4CXX_DEBUG(logger,"avgAccelZ = " << avgAccelZ);
	LOG4CXX_DEBUG(logger,"absoluteAccel = " << absoluteAccel);

	// Try to assess pitch and roll angle from the accelerometer.
	varioStatus.pitchAngle = FastMath::fastASin(avgAccelX/absoluteAccel);
	LOG4CXX_DEBUG(logger,"Initial pitchAngle = " << varioStatus.pitchAngle);
	varioStatus.getErrorCovariance_P().
			coeffRef(varioStatus.STATUS_IND_PITCH,varioStatus.STATUS_IND_PITCH) = 4.0f;

	varioStatus.rollAngle = -FastMath::fastASin(avgAccelY/absoluteAccel);
	LOG4CXX_DEBUG(logger,"Initial rollAngle = " << varioStatus.rollAngle);
	varioStatus.getErrorCovariance_P().
			coeffRef(varioStatus.STATUS_IND_ROLL,varioStatus.STATUS_IND_ROLL) = 4.0f;

	// Gravity and accelerometers are siamese twins. So I handle gravity here too.

	varioStatus.gravity = calibrationData.gravity;
	LOG4CXX_DEBUG(logger,"Initial gravity = " << varioStatus.gravity);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GRAVITY,varioStatus.STATUS_IND_GRAVITY) =
			0.0f; //calibrationData.gravityVariance * 2.0f;


	// Set acceleration values unconditionally. These are my turf.
	// Initialize them to 0. I am assuming you switch on the device when the plane is
	// sitting peacefully on the ground.
	// If you are re-starting in the air let's at least hope you are not in the midst of a wild rotor.
	varioStatus.accelHeading = 0.0f;
	varioStatus.getErrorCovariance_P().
			coeffRef(varioStatus.STATUS_IND_ACC_HEADING,varioStatus.STATUS_IND_ACC_HEADING) = 1.0f;

	varioStatus.accelCross = 0.0f;
	varioStatus.getErrorCovariance_P().
			coeffRef(varioStatus.STATUS_IND_ACC_CROSS,varioStatus.STATUS_IND_ACC_CROSS) = 1.0f;

	varioStatus.accelVertical = 0.0f;
	varioStatus.getErrorCovariance_P().
			coeffRef(varioStatus.STATUS_IND_ACC_VERTICAL,varioStatus.STATUS_IND_ACC_VERTICAL) = 1.0f;

}

void IMUBase::initializeStatusGyro(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain,
		struct SensorData const &sumSensorData,
		int numGyroData
		) {

/// todo Initialize the gyro bias from the initial measurements when they are obviously below values which would indicate
/// turning during flight.
/// When you detect excessive turn rates due to actual turning whilst in motion use the calibration values when available.
	// Assume that you are on the ground, but maybe tilted to the side
	// (remember this instrument is primarily for gliders)
	// and slightly pitched up
	auto avgGyroX = sumSensorData.gyroX / float(numGyroData);
	auto avgGyroY = sumSensorData.gyroY / float(numGyroData);
	auto avgGyroZ = sumSensorData.gyroZ / float(numGyroData);

	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__ << " baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgGyroX = " << avgGyroX);
	LOG4CXX_DEBUG(logger,"avgGyroY = " << avgGyroY);
	LOG4CXX_DEBUG(logger,"avgGyroZ = " << avgGyroZ);

	// Set the initial status and variances of turn rates and gyro bias unconditionally.
	// These settings are solely my turf.
	varioStatus.gyroBiasX = avgGyroX;
	LOG4CXX_DEBUG(logger,"Initial gyroBiasX = " << varioStatus.gyroBiasX);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_X,varioStatus.STATUS_IND_GYRO_BIAS_X) =
			0.0001f;

	varioStatus.rollRateX = 0;
	LOG4CXX_DEBUG(logger,"Initial rollRateX = " << varioStatus.rollRateX);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROTATION_X,varioStatus.STATUS_IND_ROTATION_X) =
			1.0f;

	varioStatus.gyroBiasY = avgGyroY;
	LOG4CXX_DEBUG(logger,"Initial gyroBiasY = " << varioStatus.gyroBiasY);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Y,varioStatus.STATUS_IND_GYRO_BIAS_Y) =
			0.0001f;

	varioStatus.pitchRateY = 0;
	LOG4CXX_DEBUG(logger,"Initial pitchRateY = " << varioStatus.pitchRateY);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROTATION_Y,varioStatus.STATUS_IND_ROTATION_Y) =
			0.0001f;

	varioStatus.gyroBiasZ = avgGyroZ;
	LOG4CXX_DEBUG(logger,"Initial gyroBiasZ = " << varioStatus.gyroBiasZ);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_GYRO_BIAS_Z,varioStatus.STATUS_IND_GYRO_BIAS_Z) =
			0.0001f;

	varioStatus.yawRateZ = 0;
	LOG4CXX_DEBUG(logger,"Initial yawRateZ = " << varioStatus.yawRateZ);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_ROTATION_Z,varioStatus.STATUS_IND_ROTATION_Z) =
			1.0f;

}

void IMUBase::initializeStatusMag(
		GliderVarioStatus &varioStatus,
		GliderVarioMainPriv &varioMain,
		struct SensorData const &sumSensorData,
		int numMagData
		) {

	/*
	 *
	 * Assume that you are on the ground, but maybe tilted to the side
	 * (remember this instrument is primarily for gliders)
	 * and slightly pitched up
	 *
	 * Another precondition is that initializeStatusAccel() has been called before so that roll and pitch angle are
	 * already determined, and I can calculate the yaw (i.e. the direction)
	 */
	auto avgMagX = sumSensorData.magX / float(numMagData) - calibrationData.magXBias;
	auto avgMagY = sumSensorData.magY / float(numMagData) - calibrationData.magYBias;
	auto avgMagZ = sumSensorData.magZ / float(numMagData) - calibrationData.magZBias ;

	double baseIntervalSec = varioMain.getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__ << "baseIntervalSec = " << baseIntervalSec);

	LOG4CXX_DEBUG(logger,"avgMagX = " << avgMagX);
	LOG4CXX_DEBUG(logger,"avgMagY = " << avgMagY);
	LOG4CXX_DEBUG(logger,"avgMagZ = " << avgMagZ);

	// If the pitch angle is nearly perpendicular to the flat plane the roll angle cannot be determined with any accuracy.
	// Albeit this is a more than unlikely scenario :D
	if (fabsf(varioStatus.pitchAngle) < 80) {
		RotationMatrix rotMatrix (0.0f,varioStatus.pitchAngle,varioStatus.rollAngle);
		Vector3DType planeMagVector (avgMagX,avgMagY,avgMagZ);
		Vector3DType worldMagVector;

		// I already determined pitch and roll angle. With these now I can move the plane coordinate system into
		// the wold coordinate system, and can determine the heading (yaw angle).
		rotMatrix.calcPlaneVectorToWorldVector(planeMagVector,worldMagVector);

		LOG4CXX_DEBUG(logger,"worldMagX = " << worldMagVector[0]);
		LOG4CXX_DEBUG(logger,"worldMagY = " << worldMagVector[1]);
		LOG4CXX_DEBUG(logger,"worldMagZ = " << worldMagVector[2]);

		if (UnInitVal == varioStatus.heading) {
			varioStatus.heading = FastMath::fastATan2(-worldMagVector[1],worldMagVector[0]);
			LOG4CXX_DEBUG(logger,"Initial heading = " << varioStatus.heading);
			varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_HEADING,varioStatus.STATUS_IND_HEADING) = 5.0f * 5.0f;
		}

		if (UnInitVal == varioStatus.magneticInclination) {
			varioStatus.magneticInclination = FastMath::fastATan2(
					-avgMagZ,sqrtf(avgMagX*avgMagX + avgMagY*avgMagY));
			if (varioStatus.magneticInclination > 90.0f) {
				varioStatus.magneticInclination -= 360.0f;
			}
			LOG4CXX_DEBUG(logger,"Initial magnetic inclination = " << varioStatus.magneticInclination);
			varioStatus.getErrorCovariance_P().
					coeffRef(varioStatus.STATUS_IND_MAGNETIC_INCLINATION,varioStatus.STATUS_IND_MAGNETIC_INCLINATION)
					= 4.0f;
		}
	}


	// Set the magnetometer bias unconditionally
	varioStatus.compassDeviationX = calibrationData.magXBias;
	LOG4CXX_DEBUG(logger,"Initial compassDeviationX = " << varioStatus.compassDeviationX);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_X,varioStatus.STATUS_IND_COMPASS_DEVIATION_X) = calibrationData.magXVariance / 100.0f;

	varioStatus.compassDeviationY = calibrationData.magYBias;
	LOG4CXX_DEBUG(logger,"Initial compassDeviationY = " << varioStatus.compassDeviationY);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_Y,varioStatus.STATUS_IND_COMPASS_DEVIATION_Y) = calibrationData.magYVariance / 100.0f;

	varioStatus.compassDeviationZ = calibrationData.magZBias;
	LOG4CXX_DEBUG(logger,"Initial compassDeviationZ = " << varioStatus.compassDeviationZ);
	varioStatus.getErrorCovariance_P().coeffRef(varioStatus.STATUS_IND_COMPASS_DEVIATION_Z,varioStatus.STATUS_IND_COMPASS_DEVIATION_Z) = calibrationData.magZVariance / 100.0f;

}

#undef SQUARE

void IMUBase::initializeStatus(
		GliderVarioStatus &varioStatus,
		GliderVarioMeasurementVector &measurements,
		GliderVarioMainPriv &varioMain) {

	// Try 20 times, and wait for a second if no data yet available
	for (int i = 0; i < 20;i++) {
		using namespace std::chrono_literals;

		struct SensorData avgSensorData {
			false,0.0f,0.0f,0.0f,
			false,0.0f,0.0f,0.0f,
			false,0.0f,0.0f,0.0f
		};
		int numAccel = 0;
		int numGyro  = 0;
		int numMag   = 0;

		// Add up the data in the ring buffer
		for (int k = 0; k < SIZE_SENSOR_DATA_ARRAY; k++) {
			struct SensorData &sensorData = sensorDataArr[k];

			if (sensorData.accelDataValid) {
				avgSensorData.accelX += sensorData.accelX;
				avgSensorData.accelY += sensorData.accelY;
				avgSensorData.accelZ += sensorData.accelZ;
				numAccel ++;
			}

			if (sensorData.gyroDataValid) {
				avgSensorData.gyroX += sensorData.gyroX;
				avgSensorData.gyroY += sensorData.gyroY;
				avgSensorData.gyroZ += sensorData.gyroZ;
				numGyro ++;
			}

			if (sensorData.magDataValid) {
				avgSensorData.magX += sensorData.magX;
				avgSensorData.magY += sensorData.magY;
				avgSensorData.magZ += sensorData.magZ;
				numMag ++;
			}
		}

		if (numAccel >= 10 && numGyro >= 10 && numMag >= 10) {
			initializeStatusAccel(varioStatus,varioMain,avgSensorData,numAccel);
			initializeStatusGyro(varioStatus,varioMain,avgSensorData,numGyro);
			initializeStatusMag(varioStatus,varioMain,avgSensorData,numMag);

			statusInitDone = true;

			break;
		}

		std::this_thread::sleep_for(1s);

	} // for (int i = 0; i < 20;i++)

}

void IMUBase::updateKalmanStatus (GliderVarioStatus &varioStatus) {

}

void IMUBase::updateKalman(SensorData &currSensorData) {
	if (getIsKalmanUpdateRunning()) {
		GliderVarioMainPriv::LockedCurrentStatus currStatus(*varioMain);

		if (currSensorData.accelDataValid) {
			GliderVarioMeasurementUpdater::accelUpd(
					currSensorData.accelX,1.0f,
					currSensorData.accelY,1.0f,
					currSensorData.accelZ,1.0f,
					*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
		}
		if (currSensorData.gyroDataValid) {
			GliderVarioMeasurementUpdater::gyroUpd(
					currSensorData.gyroX,1.0f,
					currSensorData.gyroY,1.0f,
					currSensorData.gyroZ,1.0f,
					*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
		}
		if (currSensorData.magDataValid) {
			GliderVarioMeasurementUpdater::compassUpd(
					currSensorData.magX,currSensorData.magY,currSensorData.magZ,
					2.0f,2.0f,2.0f,
					*currStatus.getMeasurementVector(),*currStatus.getCurrentStatus());
		}
	} // if (getIsKalmanUpdateRunning())
}

void IMUBase::fillCalibrationDataParameters () {

	// Lock the current status as briefly as possible.
	GliderVarioMainPriv::LockedCurrentStatus currentLockedStatus(*varioMain);
	GliderVarioStatus* currentStatus = currentLockedStatus.getCurrentStatus();
	GliderVarioStatus::StatusCoVarianceType &coVariance = currentStatus->getErrorCovariance_P();

	// If the estimated error is in a similar range or better than the current Variance update the estimated bias.
	// Allow for fluctuations of the variance otherwise necessary updates may never happen
	auto currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_X,
			GliderVarioStatus::STATUS_IND_GYRO_BIAS_X);
	if (currVariance <= calibrationData.gyrXVariance) {
		calibrationData.gyrXBias = currentStatus->gyroBiasX;
		calibrationData.gyrXVariance = currVariance;
		writeConfigValue(*calibrationDataParameters,"gyrXBias",calibrationData.gyrXBias);
		writeConfigValue(*calibrationDataParameters,"gyrXVariance",currVariance);
	}
	currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y,
			GliderVarioStatus::STATUS_IND_GYRO_BIAS_Y);
	if (currVariance <= calibrationData.gyrYVariance) {
		calibrationData.gyrYBias = currentStatus->gyroBiasY;
		calibrationData.gyrYVariance = currVariance;
		writeConfigValue(*calibrationDataParameters,"gyrYBias",calibrationData.gyrYBias);
		writeConfigValue(*calibrationDataParameters,"gyrYVariance",currVariance);
	}
	currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z,
			GliderVarioStatus::STATUS_IND_GYRO_BIAS_Z);
	if (currVariance <= calibrationData.gyrZVariance) {
		calibrationData.gyrZBias = currentStatus->gyroBiasZ;
		calibrationData.gyrZVariance = currVariance;
		writeConfigValue(*calibrationDataParameters,"gyrZBias",calibrationData.gyrZBias);
		writeConfigValue(*calibrationDataParameters,"gyrZVariance",currVariance);
	}

	currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X,
			GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_X);
	if (currVariance <= calibrationData.magXVariance) {
		calibrationData.magXBias = currentStatus->compassDeviationX;
		calibrationData.magXVariance = currVariance;
		writeConfigValue(*calibrationDataParameters,"magXBias",calibrationData.magXBias);
		writeConfigValue(*calibrationDataParameters,"magXVariance",currVariance);
	}
	currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y,
			GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Y);
	if (currVariance <= calibrationData.magYVariance) {
		calibrationData.magYBias = currentStatus->compassDeviationY;
		calibrationData.magYVariance = currVariance;
		writeConfigValue(*calibrationDataParameters,"magYBias",calibrationData.magYBias);
		writeConfigValue(*calibrationDataParameters,"magYVariance",currVariance);
	}
	currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z,
			GliderVarioStatus::STATUS_IND_COMPASS_DEVIATION_Z);
	if (currVariance <= calibrationData.magZVariance) {
		calibrationData.magZBias = currentStatus->compassDeviationZ;
		calibrationData.magZVariance = currVariance;
		writeConfigValue(*calibrationDataParameters,"magZBias",calibrationData.magZBias);
		writeConfigValue(*calibrationDataParameters,"magZVariance",currVariance);
	}

	currVariance = coVariance.coeff(GliderVarioStatus::STATUS_IND_GRAVITY,
			GliderVarioStatus::STATUS_IND_GRAVITY);
	if (currVariance <= calibrationData.gravityVariance) {
		calibrationData.gravity = currentStatus->gravity;
		calibrationData.gravityVariance = currVariance;
		writeConfigValue(*calibrationDataParameters,"gravityValue",calibrationData.gravity);
		writeConfigValue(*calibrationDataParameters,"gravityVariance",currVariance);
	}

	LOG4CXX_DEBUG (logger, __PRETTY_FUNCTION__ << " Fill calibration data for device \"" << instanceName << "\": \n"
			<< "\tmagXFactor		 = " << calibrationData.magXFactor << "\n"
			<< "\tmagYFactor		 = " << calibrationData.magYFactor << "\n"
			<< "\tmagZFactor		 = " << calibrationData.magZFactor << "\n"
			<< "\tmagXBias		 	 = " << calibrationData.magXBias << "\n"
			<< "\tmagYBias			 = " << calibrationData.magYBias << "\n"
			<< "\tmagZBias			 = " << calibrationData.magZBias << "\n"
			<< "\tmagXVariance		 = " << calibrationData.magXVariance << "\n"
			<< "\tmagYVariance		 = " << calibrationData.magYVariance << "\n"
			<< "\tmagZVariance		 = " << calibrationData.magZVariance << "\n"

			<< "\tgyrXFactor		 = " << calibrationData.gyrXFactor << "\n"
			<< "\tgyrYFactor		 = " << calibrationData.gyrYFactor << "\n"
			<< "\tgyrZFactor		 = " << calibrationData.gyrZFactor << "\n"
			<< "\tgyrXBias			 = " << calibrationData.gyrXBias << "\n"
			<< "\tgyrYBias			 = " << calibrationData.gyrYBias << "\n"
			<< "\tgyrZBias			 = " << calibrationData.gyrZBias << "\n"
			<< "\tgyrXVariance		 = " << calibrationData.gyrXVariance << "\n"
			<< "\tgyrYVariance		 = " << calibrationData.gyrYVariance << "\n"
			<< "\tgyrZVariance		 = " << calibrationData.gyrZVariance << "\n"

			<< "\taccelXBias		 = " << calibrationData.accelXBias << "\n"
			<< "\taccelYBias		 = " << calibrationData.accelYBias << "\n"
			<< "\taccelZBias		 = " << calibrationData.accelZBias << "\n"
			<< "\taccelXFactor		 = " << calibrationData.accelXFactor << "\n"
			<< "\taccelYFactor		 = " << calibrationData.accelYFactor << "\n"
			<< "\taccelZFactor		 = " << calibrationData.accelZFactor << "\n"
			<< "\taccelXVariance	 = " << calibrationData.accelXVariance << "\n"
			<< "\taccelYVariance	 = " << calibrationData.accelYVariance << "\n"
			<< "\taccelZVariance	 = " << calibrationData.accelZVariance << "\n"

			<< "\tgravity			 = " << calibrationData.gravity << "\n"
			<< "\tgravityVariance	 = " << calibrationData.gravityVariance << "\n"
			);

}


} /* namespace drivers */
} /* namespace openEV */
