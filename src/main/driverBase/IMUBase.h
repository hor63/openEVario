/*
 * IMUBase.h
 *
 *  Created on: Jul 29, 2021
 *      Author: hor
 */

#ifndef UTIL_DRIVERS_IMUBASE_H_
#define UTIL_DRIVERS_IMUBASE_H_

#include <fstream>
#include <map>


#include "CommonDefs.h"
#include "drivers/DriverBase.h"

namespace openEV {
namespace drivers {

class OEV_MAIN_PUBLIC IMUBase: public DriverBase {
public:

    /** \brief Structure holding one set of sensor data
     *
     * This structure holds *raw* sensor data without considering bias or factor.
     * Internal sensor compensation for the magnetometer is however applied,
     * and raw A/D converted to physical units.
     */
    struct SensorData {
		bool accelDataValid; ///< \brief Are \p accelX, \p accelY, and \p accelZ valid?
		float accelX; ///< \brief Acceleration along the X axis in g. X is forward.
		float accelY; ///< \brief Acceleration along the Y axis in g. Y is to the right.
		float accelZ; ///< \brief Acceleration along the Z axis in g. Z is *downward*! Gravitation measures as -1g!

		bool gyroDataValid; ///< \brief Are \p gyroX, \p gyroY, and \p gyroZ valid?
		float gyroX; ///< \brief Roll rate around the X axis in deg/sec. Positive value is rolling right.
		float gyroY; ///< \brief Pitch rate around the Y axis in deg/sec. Positive value is pitching up.
		float gyroZ; ///< \brief Yaw rate around the Z axis in deg/sec. Positive value is yawing/turning right.

		bool magDataValid; ///< \brief Are \p magX, \p magY, and \p magZ valid?
		float magX; ///< \brief Magnetic field strength along the X axis in uT.
		float magY; ///< \brief Magnetic field strength along the Y axis in uT.
		float magZ; ///< \brief Magnetic field strength along the Z axis in uT.
    };

    /** \brief Calibration data for the IMU
     *
     */
    struct SensorCalibrationData {

    	/**
    	 * Magnetometer bias can be measured for the magnetometer
    	 * measuring the magnetic field in an arbitrary direction as val1,
    	 * then turn the box 180 degrees that the measured axis points opposite and measure again as val2.
    	 * The raw bias is now:
    	 * \f[\frac{(val1 + val2)}{2}\f]
    	 *
    	 * *Note*: The Bias here is the result of multiplying the raw bias value with \ref magXFactor.
    	 */
    	double magXBias = 0.0;
    	/// \see \ref magXBias
    	double magYBias = 0.0;
    	/// \see \ref magXBias
    	double magZBias = 0.0;

    	/**
    	 * The magnetometer factors are relative to the Z axis magnetometer which itself has the assumed factor -1.0. \n
    	 * This makes sense because I am using it only for directions, i.e. I am interested in the ratios of the different
    	 * axis only. (remember the Y and Z axis' of the BMX160 are 180deg opposite to my coordinate system). \n
    	 * To obtain the values I am pointing the measured axis once upward, and once downward. Thus I am measuring the same
    	 * value with each of the three magnetometers, and can directly compare the readings. \n
    	 * For the other axis' the factor is:
    	 * \f[\frac{-rawValueZ-rawBiasZ}{rawValue-rawBias}\f]
    	 *
    	 */
    	double magXFactor =  1.0;
    	/// \see \ref magXFactor
    	double magYFactor = -1.0;
    	/// \see \ref magXFactor
    	double magZFactor = -1.0;

    	/// Standard Variance of the magnetometer measurements
    	double magXVariance = 2.0;
    	/// Standard Variance of the magnetometer measurements
    	double magYVariance = 2.0;
    	/// Standard Variance of the magnetometer measurements
    	double magZVariance = 2.0;

    	/// Gyro bias is the easiest: Let the box rest and measure the gyro values. These are the raw bias.
    	/// The bias used here is the result of multiplying the raw bias with the gyro factor.
    	double gyrXBias = 0.0;
    	/// \see \ref gyrXBias
    	double gyrYBias = 0.0;
    	/// \see \ref gyrXBias
    	double gyrZBias = 0.0;

    	/**
    	 * Unless you have a very precise vinyl turn table, and can place the entire assembly onto it
    	 * calibrating the gyro factor is not easily possible. Just rely on the factory trimming.
    	 * However the factor also takes care of the flipped over coordinate systems between sensor and OpenEVario.
    	 */
    	double gyrXFactor =  1.0;
    	/// \see \ref gyrXFactor
    	double gyrYFactor = -1.0;
    	/// \see \ref gyrXFactor
    	double gyrZFactor = -1.0;

    	/// Standard Variance of the gyro measurements. \n
    	/// Just leave the sensor box sitting still and measure a series of values and calculate the standard devition, and square it.
    	double gyrXVariance = 0.03;
    	double gyrYVariance = 0.03;
    	double gyrZVariance = 0.03;

    	/**
    	 * Accel bias is measured mostly the same way as magnetometer bias. \n
    	 * However the measured axis should point straight up for val1,
    	 * and straight down for val2. The formula for the raw bias is the same.
    	 *
    	 * *Note:* The Bias here the the product of the raw bias with the factor
    	 */
    	double accelXBias = 0.0;
    	double accelYBias = 0.0;
    	double accelZBias = 0.0;

    	/**
    	 *
    	 * For the accelerometer factor you can get the same measurements val1 and val2 at the same time
    	 * when measuring the bias. \n
    	 * I actually have a calibrated value for the Accelerometer, i.e. the gravity.
    	 * Val1 is the positive gravitation, and val 2 is negative gravity. \n
    	 * The formula is:
    	 * \f[\frac{GRAVITY}{val1 - val2}\f]
    	 *
    	 * *Note1:* The result must be \f$m/s^2\f$. Many sensors return *g* instead.
    	 * The gravity is variable throughout the Earth. So \ref gravity is a calibration value here as well, and can affect the factor too.
    	 *
    	 * *Note2:* Note the different coordinate systems of sensord and OpenEVario! Y and Z Axsis are negative.
    	 */
    	double accelXFactor =  GRAVITY;
    	double accelYFactor = -GRAVITY;
    	double accelZFactor = -GRAVITY;

    	/// Standard Variance of the accelerometer measurements
    	double accelXVariance = 0.001;
    	double accelYVariance = 0.001;
    	double accelZVariance = 0.001;


    	/// Value of the local gravity
    	/// \see \ref GRAVITY for more details of local gravity, and how to obtain your approximate local gravity value.
    	double gravity = GRAVITY;
    	double gravityVariance = 0.0001;

    } calibrationData;

    IMUBase(
			char const *driverName,
			char const *description,
			char const *instanceName,
			DriverLibBase &driverLib
			);
	virtual ~IMUBase();

    /// \brief Size of the array \ref sensorDataArr
    static constexpr int SIZE_SENSOR_DATA_ARRAY = 16;

    /** \brief Initialize the driver
     *
	 * @param varioMain mainVario object; provides all additional information like program parameters, and the parsed properties.
     * \see GliderVarioDriverBase::driverInit()
     */
    virtual void driverInit(GliderVarioMainPriv &varioMain) override;

    /** \brief Initialize the Kalman filter status from initial sensor measurements
     *
     * \see GliderVarioDriverBase::initializeStatus()
     */
    virtual void initializeStatus(
    		GliderVarioStatus &varioStatus,
			GliderVarioMeasurementVector &measurements,
			GliderVarioMainPriv &varioMain) override;

    /** \brief Callback to update the Kalman filter status based on received data.
     *
     * \see GliderVarioDriverBase::updateKalmanStatus()
     */
    virtual void updateKalmanStatus (GliderVarioStatus &varioStatus) override;

protected:

    /// \brief Is the status initialization done
    bool statusInitDone = false;

    /** \brief Array of sensor data
     *
     * Ring buffer of sensor data.
     * For continuous operation only the recent record which is indicated by \ref currSensorDataIndex is being used.
     * During initialization I use the whole array to get a recent average to prime the Kalman status.
     *
     */
    SensorData sensorDataArr [SIZE_SENSOR_DATA_ARRAY];

    /** \brief Index of current sensor data into sensorDataArr
     *
     * The array \ref sensorDataArr is filled in ring buffer fashion. For status initialization the average is calculated.
     * For continuous updates only the current record is used.
     */
    int currSensorDataIndex = 0;

#if defined HAVE_LOG4CXX_H
    static log4cxx::LoggerPtr logger;
#endif

    /** \brief Initialize the Kalman status from the accelerometer measurements
     *
     * Actually I am *not* initializing the acceleration values of the Kalman status.
     * Reason is that the acceleration of the model is not exacly the accelerometer measurements of the IMU when the body
     * is not perfectly flat, i.e. neither pitch nor roll applies. Particularly the roll angle is anything but 0 in a glider on the ground.
     *
     * Instead I am initializing the roll and pitch angle assuming that the plane is stationary during the initialization which usually happens
     * when the plane is sitting on the ground. But even in level flight that should work to a certain degree.
     *
     * @param varioStatus The Kalman status to be initialized
     * @param varioMain Vario main object
     * @param sumSensorData Sensor data summed up over \p numAccelData times.
     * @param numAccelData Number of accelerometer measurements summed up in \p numSensorData
     */
    void initializeStatusAccel(
    		GliderVarioStatus &varioStatus,
    		GliderVarioMainPriv &varioMain,
    		struct SensorData const &sumSensorData,
    		int numAccelData
    		);

    /** \brief Initialize the Kalman status with the gyroscope measurements
     *
     * Initialize the gyroscope bias which is in body coordinates anyway. So no issues with non-flat attitudes whatsoever.
     *
     * @param varioStatus The Kalman status to be initialized
     * @param varioMain Vario main object
     * @param avgSensorData Sensor data summed up over \p numGyroData times.
     * @param numGyroData Number of gyroscope measurements summed up in \p avgSensorData
     */
    void initializeStatusGyro(
    		GliderVarioStatus &varioStatus,
    		GliderVarioMainPriv &varioMain,
    		struct SensorData const &avgSensorData,
    		int numGyroData
    		);

    /** \brief Initialize the Kalman status with the magnetometer measurements
     *
     * Rotate the measured vector into the world plane, and calculate the yaw angle (direction) from the horizontal component
     *
     * This function requires that \ref initializeStatusAccel was called before to determine the roll and pitch angle.
     *
     * @param varioStatus The Kalman status to be initialized
     * @param varioMain Vario main object
     * @param sumSensorData Sensor data summed up over \p numMagData times.
     * @param numMagData Number of magnetometer measurements summed up in \p numSensorData
     */
    void initializeStatusMag(
    		GliderVarioStatus &varioStatus,
    		GliderVarioMainPriv &varioMain,
    		struct SensorData const &sumSensorData,
    		int numMagData
    		);

    /** \brief Update the Kalman filter with measurement data
     *
     * Update the Kalman filter when \ref getIsKalmanUpdateRunning() is true
     */
    void updateKalman(SensorData &currSensorData);

    virtual void fillCalibrationDataParameters () override;


};

} /* namespace drivers */
} /* namespace openEV */

#endif /* UTIL_DRIVERS_IMUBASE_H_ */
