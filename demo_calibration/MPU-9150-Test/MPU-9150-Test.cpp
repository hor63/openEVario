
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <errno.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include <thread>
#include <chrono>

#include "OEVCommon.h"
#include "log4cxx/rollingfileappender.h"
#include "log4cxx/patternlayout.h"
#include "log4cxx/rolling/sizebasedtriggeringpolicy.h"
#include "MPU-9150Defs.h"
#include "I2CPort.h"

using namespace openEV::drivers::TDK_MPU9150;

union UnionInt16{
		uint16_t uintVal;
		int16_t intVal;
	};

typedef float FloatType;

static std::string i2cName;
static uint8_t i2cAddr = 0x68;

static AK8975_mag_trim_registers trimRegisters;

static FloatType magFactorX = 1229.0f * 2.0f / (4095.0f + 4096.0f);
static FloatType magFactorY = 1229.0f * 2.0f / (4095.0f + 4096.0f);
static FloatType magFactorZ = 1229.0f * 2.0f / (4095.0f + 4096.0f);

static FloatType gyrFactor = 1.0f / 131.0f;
static FloatType accFactor = 1.0f / 8192.0f;

static uint32_t numSamples = 1000;

static log4cxx::LevelPtr loggerLevel;

static log4cxx::helpers::Pool p;

static void initLogger(){
	log4cxx::LoggerPtr rootLogger = 0;
	log4cxx::RollingFileAppenderPtr rollingFileAppender = 0;
	log4cxx::PatternLayoutPtr patternLayout = 0;

	rollingFileAppender = new log4cxx::RollingFileAppender();

	patternLayout = new log4cxx::PatternLayout();
	patternLayout->setConversionPattern("%d{dd MMM HH:mm:ss,SSS} %t %p %c %l: - %m%n");

	rollingFileAppender->setName("RollingFileAppender");
	rollingFileAppender->setFile("MPU-9150-Test.log");
	rollingFileAppender->setAppend(true);
	rollingFileAppender->setMaxBackupIndex(10);
	rollingFileAppender->setMaximumFileSize(1024*1024);
	rollingFileAppender->setLayout(patternLayout);

	rootLogger = log4cxx::Logger::getRootLogger();
	rootLogger->addAppender(rollingFileAppender);

	rollingFileAppender->activateOptions(p);

	rootLogger->setLevel(loggerLevel);

	log4cxx::BasicConfigurator::configure(rollingFileAppender);
}

static I2CPort *i2cPort = nullptr;


// Use slave 4 for single transactions
static uint8_t readByteAux (uint8_t devAddr, uint8_t slaveDevAddr, uint8_t regAddr) {
	log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("readByteAux");
	uint8_t buf[5];

	// Slave 4 register set start
	buf[0] = REG_9150_I2C_SLV4_ADDR;

	buf[1] = slaveDevAddr |
			I2C_SLV4_RW // Read from the AK8975 on the Aux bus
			;
	// I2C_SLV4_REG: The register address on the slave device
	buf[2] = regAddr;
	// I2C_SLV4_DO: Data out (unused)
	buf[3] = 0;
	// I2C_SLV4_CTRL: Activate the slave interface transaction
	buf[4] = 0
			| I2C_SLV4_EN
			// | I2C_SLV4_INT_EN No interrupts used
			// | I2C_SLV4_REG_DIS Use the register addressing mode
			;

	i2cPort->writeBlock(devAddr, buf, 5);

	// wait for the transaction to complete
	for (int i = 0; i < 100; ++i) {
		// Read I2C_SLV4_CTRL and I2C_SLV4_DI at once
		i2cPort->readBlockAtRegAddrByte(devAddr, REG_9150_I2C_SLV4_CTRL,buf,2);
		LOG4CXX_TRACE(logger,"Cycle #" << i << ": REG_9150_I2C_SLV4_CTRL = 0x" << std::hex << uint32_t(buf[0]) << std::dec);

		// check if the enable flag was cleared, i.e the transaction finished.
		if ((buf[0] & I2C_SLV4_EN) == 0) {
			break;
		}
	}

	// buf[1] already contains Data-IN from the reading in the loop.
	LOG4CXX_TRACE(logger,"REG_9150_I2C_SLV4_DI = 0x" << std::hex << uint32_t(buf[1]) << std::dec);

	return buf[1];
}

// Use slave 4 for single transactions
static void writeByteAux (uint8_t devAddr, uint8_t slaveDevAddr, uint8_t regAddr, uint8_t data) {
	log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("writeByteAux");
	uint8_t buf[5];

	// Slave 4 register set start
	buf[0] = REG_9150_I2C_SLV4_ADDR;
	// write to the AK8975 on the Aux bus (read bit not set, i.e. write)
	buf[1] = slaveDevAddr
			// | I2C_SLV4_RW // Write to the AK8975 on the Aux bus
			;
	// I2C_SLV4_REG: The register address on the slave device
	buf[2] = regAddr;
	// I2C_SLV4_DO: Data out, byte to be written
	buf[3] = data;
	// I2C_SLV4_CTRL: Activate the slave interface transaction
	buf[4] = 0
			| I2C_SLV4_EN
			// | I2C_SLV4_INT_EN No interrupts used
			// | I2C_SLV4_REG_DIS Use the register addressing mode
			;

	i2cPort->writeBlock(devAddr, buf, 5);

	// wait for the transaction to complete
	for (int i = 0; i < 1000; ++i) {
		// Read I2C_SLV4_CTRL
		i2cPort->readBlockAtRegAddrByte(devAddr, REG_9150_I2C_SLV4_CTRL,buf,1);
		LOG4CXX_TRACE(logger,"Cycle #" << i << ": REG_9150_I2C_SLV4_CTRL = 0x" << std::hex << uint32_t(buf[0]) << std::dec);

		// check if the enable flag was cleared, i.e the transaction finished.
		if ((buf[0] & I2C_SLV4_EN) == 0) {
			break;
		}
	}

}


static void setupMPU9150() {
	using namespace std::literals::chrono_literals;
	log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("setupMPU9150");
	uint8_t buf[8] = {0};

	// Reset the entire device
	buf[0] = REG_9150_PWR_MGMT_1;
	buf[1] = PWR_MGMT_DEVICE_RESET;
	LOG4CXX_DEBUG(logger,"Write REG_9150_PWR_MGMT_1 reset");
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Wait a bit letting the sensor run through the reset routine.
	std::this_thread::sleep_for(100ms);

	// Clear the sleep flag and all other flags.
	buf[0] = REG_9150_PWR_MGMT_1;
	buf[1] = 0;
	LOG4CXX_DEBUG(logger,"Write REG_9150_PWR_MGMT_1 Clear sleep flag");
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Wait a bit letting the gyros starting up.
	std::this_thread::sleep_for(100ms);

	buf[0] = i2cPort->readByteAtRegAddrByte(i2cAddr, REG_9150_WHO_AM_I);

	// Who am I is the I2C adress (ignoring bit 0 which can be set with pin AD0 (Pin 9))
	if ((i2cAddr & ~1) == buf[0]) {
		LOG4CXX_INFO(logger,"WHO AM I contains expected 0x" << std::hex << uint32_t(buf[0]) << std::dec);
	} else {
		LOG4CXX_WARN(logger,"WHO AM I contains unexpected 0x" << std::hex << uint32_t(buf[0])
				<< ". Expected was 0x" << (i2cAddr & ~1)
				<< std::dec);
	}

	// Disable Gyro self-test, and set the full-scale range to +-250deg/sec
	buf[0] = REG_9150_GYRO_CONFIG;
	buf[1] = GYRO_RANGE_250;
	LOG4CXX_DEBUG(logger,"Write REG_9150_GYRO_CONFIG");
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Disable Accel self-test, and set the full-scale range to +-4g
	buf[0] = REG_9150_ACCEL_CONFIG;
	buf[1] = ACCEL_RANGE_4G;
	LOG4CXX_DEBUG(logger,"Write REG_9150_ACCEL_CONFIG");
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Disable external frame synchronization,
	// and set the DLPF filter to 8.5ms delay and 20Hz bandwidth
	buf[0] = REG_9150_CONFIG;
	buf[1] = DLPF_20HZ;
	LOG4CXX_DEBUG(logger,"Write REG_9150_CONFIG DLPF filter");
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Setup the sample rate to 20ms. The Gyro rate will be 1kHz because I just activated the DLPF.
	buf[0] = REG_9150_SMPLRT_DIV;
	buf[1] = 19; // + 1 = 20ms = 50Hz.
	LOG4CXX_DEBUG(logger,"Write REG_9150_SMPLRT_DIV");
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Set the clock source to the X-Gyro via PLL.
	buf[0] = REG_9150_PWR_MGMT_1;
	buf[1] = 0
			// | PWR_MGMT_DEVICE_RESET
			// | PWR_MGMT_SLEEP
			// | PWR_MGMT_CYCLE
			// | PWR_MGMT_DISABLE_TEMP
			| CLKSEL_PLL_X_GYR
			;
	LOG4CXX_DEBUG(logger,"Write REG_9150_PWR_MGMT_1 clock source X-Gyro");
	i2cPort->writeBlock(i2cAddr, buf, 2);

	i2cPort->readBlockAtRegAddrByte(i2cAddr, REG_9150_TEMP_OUT_H , buf, 2);
	UnionInt16 tempRaw;

	tempRaw.uintVal = (uint16_t(buf[0]) << 8) | uint16_t(buf[1]);
	LOG4CXX_DEBUG(logger,"Temperature raw " << tempRaw.intVal << " = " << (FloatType(tempRaw.intVal)/340.0f + 35.0f));

	/*
	 *
	 */

/* This section serves as demo how to directly access the
 * AK8975 magnetometer directly from the host/main I2C bus.
 *
 *	// Enable the aux I2C pass-through
 *	buf[0] = REG_9150_INT_PIN_CFG;
 *	buf[1] = I2C_BYPASS_EN;
 *	LOG4CXX_DEBUG(logger,"Write REG_9150_INT_PIN_CFG; enable I2C aux pass-through");
 *	i2cPort->writeBlock(i2cAddr, buf, 2);
 *
 *	std::this_thread::sleep_for(10ms);
 *
 *	// Read WhoAmI and Info from the AK8975 via pass-through.
 *	i2cPort->readBlockAtRegAddrByte(AK8975_I2CAddr, REG_AK8975_WIA, buf, 2);
 *	LOG4CXX_INFO(logger,"AK8975 via pass-through: WhoAmI = 0x" << std::hex << uint16_t(buf[0])
 *			<< ", Info = 0x" << uint16_t(buf[1])
 *			<< std::dec);
 *
 */

	// Disable the aux I2C pass-through
	buf[0] = REG_9150_INT_PIN_CFG;
	buf[1] = 0
			// | I2C_BYPASS_EN
			;
	LOG4CXX_DEBUG(logger,"Write REG_9150_INT_PIN_CFG; disable I2C aux pass-through");
	i2cPort->writeBlock(i2cAddr, buf, 2);
	std::this_thread::sleep_for(10ms);

	// enable the aux master controller
	buf[0] = REG_9150_USER_CTRL;
	buf[1] = I2C_MST_EN;
	LOG4CXX_DEBUG(logger,"Write REG_9150_USER_CTRL; enable I2C aux master controller");
	i2cPort->writeBlock(i2cAddr, buf, 2);

	std::this_thread::sleep_for(10ms);


	buf[0] = readByteAux (i2cAddr,AK8975_I2CAddr,REG_AK8975_WIA);
	buf[1] = readByteAux (i2cAddr,AK8975_I2CAddr,REG_AK8975_INFO);
	LOG4CXX_INFO(logger,"AK8975 via Aux: WhoAmI = 0x" << std::hex << uint16_t(buf[0])
			<< ", Info = 0x" << uint16_t(buf[1])
			<< std::dec);

	// Enable prom read mode
	writeByteAux(i2cAddr,AK8975_I2CAddr,REG_AK8975_CNTL,AK8975_PROM_READ);
	std::this_thread::sleep_for(10ms);

	trimRegisters.asa_x = readByteAux (i2cAddr,AK8975_I2CAddr,REG_AK8975_ASAX);
	LOG4CXX_DEBUG(logger,"AK8975 via Aux: ASAX = 0x" << std::hex << uint16_t(trimRegisters.asa_x)
			<< std::dec);
	trimRegisters.asa_y = readByteAux (i2cAddr,AK8975_I2CAddr,REG_AK8975_ASAY);
	LOG4CXX_DEBUG(logger,"AK8975 via Aux: ASAY = 0x" << std::hex << uint16_t(trimRegisters.asa_y)
			<< std::dec);
	trimRegisters.asa_z = readByteAux (i2cAddr,AK8975_I2CAddr,REG_AK8975_ASAZ);
	LOG4CXX_DEBUG(logger,"AK8975 via Aux: ASAZ = 0x" << std::hex << uint16_t(trimRegisters.asa_z)
			<< std::dec);

	// Calculate the magnetometer factors adjusted by the factory trim factors
	LOG4CXX_DEBUG(logger,"Unadjusted Mag conversion  = " << magFactorX);
	magFactorX *= FloatType(int(trimRegisters.asa_x) - 128) / 256.0f + 1.0f;
	magFactorY *= FloatType(int(trimRegisters.asa_y) - 128) / 256.0f + 1.0f;
	magFactorZ *= FloatType(int(trimRegisters.asa_z) - 128) / 256.0f + 1.0f;

	LOG4CXX_DEBUG(logger,"Adjusted Mag conversion  = " << magFactorX << ", " << magFactorY << ", " << magFactorZ);

	// Set back to power-down mode
	writeByteAux(i2cAddr,AK8975_I2CAddr,REG_AK8975_CNTL,AK8975_PWR_DOWN);
	std::this_thread::sleep_for(10ms);

	// Start single measurement mode
	writeByteAux(i2cAddr,AK8975_I2CAddr,REG_AK8975_CNTL,AK8975_SINGLE_MEAS);

	// Enable the data ready interrupt
	buf[0] = REG_9150_INT_ENABLE;
	buf[1] = 1;
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Delay the data ready interrupt until the external data (i.e. magnetometer) are also received.
	buf[0] = REG_9150_I2C_MST_CTRL;
	buf[1] = WAIT_FOR_ES // WAIT_FOR_ES: DATA_RDY is not raised before external data arrived
			| I2C_MST_P_NSR // Always send a STOP between aux master transactions
			; // I2C master clock divider remains 0 = 348 kHz.
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Now set up Slave 0 and 1:
	// Slave 0 reads out the magnetometer readings
	// Slave 1 starts a new measurement cycle for reading by slave 0 in the next cycle

	// First set the byte being sent to the magnetometer, which is to set
	// the single measurement mode.
	buf[0] = REG_9150_I2C_SLV1_DO;
	buf[1] = AK8975_SINGLE_MEAS; // Single Measurement mode
	i2cPort->writeBlock(i2cAddr, buf, 2);

	// Start writing at the start of slave 0 control registers,
	// but continue writing the slave 1 registers in one transaction.
	buf[0] = REG_9150_I2C_SLV0_ADDR;
	// Slave 0
	buf[1] = I2C_SLVx_RW // read
			| AK8975_I2CAddr; // The I2C address of the magnetometer
	// The start register to read
	buf[2] = REG_AK8975_ST1;
	buf[3] = 8 // Number of bytes (status 1, 3 axis - 2-byte measurements, status2)
			| I2C_SLVx_EN // Enable slave interface
			;

	// Slave 1
	buf[4] = // I2C_SLVx_RW | // write (do not set read bit)
			 AK8975_I2CAddr; // The I2C address of the magnetometer
	// The Control register where the value in REG_9150_I2C_SLV1_DO (see above) is being written.
	buf[5] = REG_AK8975_CNTL;
	buf[6] = 1 // Number of bytes (Send one command byte)
			| I2C_SLVx_EN // Enable slave interface
			;
	i2cPort->writeBlock(i2cAddr, buf, 7);

}

static void usage(int argc, char** argv) {
	std::cerr << std::endl << "Usage: " << argv[0]
			<< " -i I2CDevice [-a I2CAddr] [-c numSamples] [-s] [-d] [-h]"
			<< std::endl << std::endl
			<< "Options:" << std::endl
			<< "	-i I2CDevice: Name of the I2C device node, e.g. /dev/i2c-0" << std::endl
			<< "	-a I2CAddr: I2C slave address; optional, default is 0x68. " << std::endl
			<< "		I2CAddr can be a decimal number, e.g. 25," << std::endl
			<< "		or a hexadecimal number, e.g. 0x19 or 0X19" << std::endl
			<< "		or an octal number, e.g. 031" << std::endl
			<< "	-c numSamples: Number of samples to obtain and print. Default is 1000" << std::endl
			<< "	-s: Silent mode. Only errors are reported in the log file \"MPU-9150-Test.log\"" << std::endl
			<< "		Default logger mode to the log file \"MPU-9150-Test.log\" is Info." << std::endl
			<< "	-d: Debug mode. Print debug information in the log file \"MPU-9150-Test.log\"." << std::endl
			<< "		Using the option twice raises the logger level to Trace mode, i.e. prints internal workings." << std::endl
			<< "		Using the option more then two times leaves the logger level at Trace mode." << std::endl
			<< "	-h: Print this help text and leave the program immediately" << std::endl
			;
}

static void readOptions(int argc, char** argv) {

	// Default level is Info
	loggerLevel = log4cxx::Level::getInfo();

	for (;;){
		int opt = getopt(argc, argv, "i:a:c:sdh");
		if (opt == -1) {
			break;
		}
		switch (opt) {
		case 'i':
			i2cName = optarg;
			break;
		case 'a':
			{
			auto addr = strtoul(optarg,nullptr,0);
			if (addr == 0) {
				std::cerr << "Error: Invalid I2C address \"" << optarg << "\"" << std::endl;
				exit(1);
			}
			if (addr > 0x7f) {
				std::cerr << "Error: I2C address \"" << optarg << "\" is not a 7-bit address" << std::endl;
				exit(1);
			}
			i2cAddr = uint8_t(addr);
			}
			break;
		case 'c':
			numSamples = strtoul(optarg,nullptr,0);
			if (numSamples == 0) {
				std::cerr << "Error: Invalid number of samples \"" << optarg << "\"" << std::endl;
			}
			break;
		case 's':
			loggerLevel = log4cxx::Level::getError();
			break;
		case 'd':
			if (loggerLevel == log4cxx::Level::getDebug()) {
				// When already at Debug level increase to Trace level
				loggerLevel == log4cxx::Level::getTrace();
			} else {
				// If the level is already at Trace level, you are already at the most chatty level.
				if (loggerLevel != log4cxx::Level::getTrace()) {
					// When the level is lower than Debug (Info, Error, Fatal, or OFF)
					// set it to Debug
					loggerLevel == log4cxx::Level::getDebug();
				}
			}
			break;
		case 'h':
		case '?':
		default:
			usage(argc, argv);
			exit(1);
			break;
		}
	}

	std::cerr << "Logger level = " << loggerLevel->toString() << std::endl;

	if (!i2cName.compare("")) {
		std::cerr << "Error: I2C device name is missing" << std::endl;
		usage(argc, argv);
		exit(1);
	}
}

struct MeasurementData {
	UnionInt16 accelX;
	UnionInt16 accelY;
	UnionInt16 accelZ;
	UnionInt16 tempRaw;
	UnionInt16 gyroX;
	UnionInt16 gyroY;
	UnionInt16 gyroZ;
	uint8_t    magStatus1;
	uint8_t    magStatus2;
	UnionInt16 magX;
	UnionInt16 magY;
	UnionInt16 magZ;
};

static MeasurementData measurementData;

int main (int argc, char** argv) {
	using namespace std::literals::chrono_literals;

//	const double accFactor = (double)0x8000 / 4.0;
//	const double gyrFactor = (double)0x8000 / 250.0;

	readOptions(argc,argv);

	initLogger();
	log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("main");

	i2cPort = new I2CPort(i2cName.c_str());
	i2cPort->open();

	memset (&trimRegisters,0,sizeof(trimRegisters));

	setupMPU9150();
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point nextStop = now;

	for (uint32_t i = 0; i < numSamples; ++i) {
		auto prev = now;
		std::chrono::microseconds dur;
		uint8_t buf[6	// Accel
					+ 2	// Temperature
					+ 6	// Gyro
					+ 8	// 8 bytes from Slave0
					];
		LOG4CXX_DEBUG(logger,"=== Start new cycle.======================");
		now = std::chrono::system_clock::now();
		for(;;) {
			buf[0] = i2cPort->readByteAtRegAddrByte(i2cAddr, REG_9150_INT_STATUS);
			LOG4CXX_DEBUG(logger,"Interrupt status = 0x" << std::hex << uint16_t(buf[0]) << std::dec);
			if ((buf[0] & DATA_RDY_INT) == DATA_RDY_INT) {
				break;
			}
			now = std::chrono::system_clock::now();
			// Reset the next stop the the next attempt to obtain the data ready status
			nextStop = now;
		}

		i2cPort->readBlockAtRegAddrByte(i2cAddr, REG_9150_ACCEL_XOUT_H, buf, sizeof(buf));
		// Start single measurement mode
		// writeByteAux(i2cAddr,AK8975_I2CAddr,REG_AK8975_CNTL,1);

		// Accel and mag data are stored big-endian
		measurementData.accelX.uintVal = (buf[0] << 8) | buf[1];
		measurementData.accelY.uintVal = (buf[2] << 8) | buf[3];
		measurementData.accelZ.uintVal = (buf[4] << 8) | buf[5];

		measurementData.tempRaw.uintVal = (buf[6] << 8) | buf[7];

		measurementData.gyroX.uintVal = (buf[8] << 8) | buf[9];
		measurementData.gyroY.uintVal = (buf[10] << 8) | buf[11];
		measurementData.gyroZ.uintVal = (buf[12] << 8) | buf[13];

		measurementData.magStatus1 = buf[14];
		// Mag data are stored little-endian,
		// the magnetometer from another manufacturer slapped onto the primary die)
		measurementData.magX.uintVal = (buf[16] << 8) | buf[15];
		measurementData.magY.uintVal = (buf[18] << 8) | buf[17];
		measurementData.magZ.uintVal = (buf[20] << 8) | buf[19];

		measurementData.magStatus2 = buf[21];

		// NOTE! The Y and X axes of the magnetometer are rotated by 90 deg
		// compared to accelerometer and gyroscope!
		// Therefore I am printing out magnetometer X as Y and vice versa.

		LOG4CXX_TRACE(logger,"Accel  data " << measurementData.accelX.intVal
				<< ", " << measurementData.accelY.intVal
				<< ", " << measurementData.accelZ.intVal
				);
		LOG4CXX_TRACE(logger,"Accel  data in g = "
				<< (FloatType(measurementData.accelX.intVal) * accFactor) << ", "
				<< (FloatType(measurementData.accelY.intVal) * accFactor) << ", "
				<< (FloatType(measurementData.accelZ.intVal) * accFactor)
				);
		LOG4CXX_TRACE(logger,"Gyro   data " << measurementData.gyroX.intVal
				<< ", " << measurementData.gyroY.intVal
				<< ", " << measurementData.gyroZ.intVal
				);
		LOG4CXX_TRACE(logger,"Gyro  data in deg/s = "
				<< (FloatType(measurementData.gyroX.intVal) * gyrFactor) << ", "
				<< (FloatType(measurementData.gyroY.intVal) * gyrFactor) << ", "
				<< (FloatType(measurementData.gyroZ.intVal) * gyrFactor)
				);
		LOG4CXX_TRACE(logger,"Magnet data " << measurementData.magY.intVal
				<< ", " << measurementData.magX.intVal
				<< ", " << measurementData.magZ.intVal
				);
		LOG4CXX_TRACE(logger,"Magnet data in uT = "
				<< (FloatType(measurementData.magY.intVal) * magFactorY) << ", "
				<< (FloatType(measurementData.magX.intVal) * magFactorX) << ", "
				<< (FloatType(measurementData.magZ.intVal) * magFactorZ)
				);
		LOG4CXX_DEBUG(logger,"Magnet status1 = 0x" << std::hex << uint16_t(measurementData.magStatus1)
				<< ", status2 = 0x" << std::hex << uint16_t(measurementData.magStatus2));
		LOG4CXX_DEBUG(logger,"Temperature raw " << measurementData.tempRaw.intVal << " = "
				<< (FloatType(measurementData.tempRaw.intVal)/340.0f + 35.0f));

		dur = std::chrono::duration_cast<std::chrono::microseconds>(now - prev) ;

		// Print the elapsed time in uSec.
		printf("%6ld\t",dur.count());

		// NOTE! The Y and X axes of the magnetometer are rotated by 90 deg
		// compared to accelerometer and gyroscope!
		// Therefore I am printing out magnetometer X as Y and vice versa here too.
		printf (
				"% 10.6f % 10.6f % 10.6f",
				(FloatType(measurementData.magY.intVal) * magFactorY),
				(FloatType(measurementData.magX.intVal) * magFactorX),
				(FloatType(measurementData.magZ.intVal) * magFactorZ)
				);
		printf (
				"% 10.6f % 10.6f % 10.6f",
				(FloatType(measurementData.gyroX.intVal) * gyrFactor),
				(FloatType(measurementData.gyroY.intVal) * gyrFactor),
				(FloatType(measurementData.gyroZ.intVal) * gyrFactor)
				);
		printf (
				"% 10.6f % 10.6f % 10.6f\n",
				(FloatType(measurementData.accelX.intVal) * accFactor),
				(FloatType(measurementData.accelY.intVal) * accFactor),
				(FloatType(measurementData.accelZ.intVal) * accFactor)
				);


		// Smooth out variances in the local clock by adding another interval to the original time stamp
		// Set the interval slightly smaller to the 20ms cycle on the sensor.
		// Some time the data will not yet be ready, and next stop is being reset above in the inner for(;;) loop
		nextStop += 19ms;
		std::this_thread::sleep_until(nextStop);
	}

	delete i2cPort;
	return 0;
}

GliderVarioExceptionBase::GliderVarioExceptionBase(
		char const *source,
		int line,
		char const *description,
		int err)
	: source{source},
	  line{line},
	  description{description},
	  err{err}
{

	std::ostringstream ostr;
	ostr << "Exception at " << source << "[" << line << "] : " << description;
	if (err != 0) {
		ostr << " err = " << err << " = 0x" << std::hex << err << std::dec << ": \"" << strerror(err) << '"';
	}
	whatString = ostr.str();

}

GliderVarioExceptionBase::~GliderVarioExceptionBase() {

}

const char* GliderVarioExceptionBase::what() const noexcept{
	return whatString.c_str();
}
