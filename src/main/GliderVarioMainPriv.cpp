/*
 * GliderVarioMainPriv.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2018  Kai Horstmann
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

#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>



#if HAVE_ARGP_H == 1
#	include <argp.h>
#endif

// Another options library
#if HAVE_GETOPT_H == 1
#	include <getopt.h>
#endif

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "GliderVarioMain.h"
#include "main/GliderVarioMainPriv.h"
#include "util/io/PortBase.h"

#if WITH_IP_PORT_DRIVERS
#	include "util/io/TCPPort.h"
#	include "util/io/UDPPort.h"
#endif

#if WITH_SERIAL_PORT_DRIVER
#	include "util/io/SerialPort.h"
#endif

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;
static log4cxx::LoggerPtr rootLogger = 0;
#endif

#define PROGRAM_DESCRIPTION "\n openEVario: Electronic variometer application using inertial and pressure sensors \n" \
		"(typically I2C on ARM SoC) and GPS input."

#if HAVE_ARGP_PARSE == 1

// extern C to avoid C++ mangeling. Argp uses these variables when defined.
extern "C" {
// Set global variables for options --help and --version
const char * argp_program_version = PACKAGE_STRING;
const char * argp_program_bug_address = PACKAGE_BUGREPORT;

static const char* argpDoc= PROGRAM_DESCRIPTION;
} // extern "C"

static struct argp_option options[] = {
  {"configuration",        'c', "configFileName",   0, "Name of the configuration file [" defaultConfigFileName "]"},
  {"logger-configuration", 'l', "loggerConfigFile", 0, "Name of logger configuration file [" defaultLoggerConfigFileName "]"},
  {"debug",                'd', 0,                  0, "Increase default logger level (Silent-[Error]-Info-Debug)"},
  {"quiet",                'q', 0,                  0, "Shhhh. Be quiet. Suppress any logger output, i.e. set logger level to Silent (see -d)"},
  {"silent",               's', 0,                  OPTION_ALIAS, 0 },
  {0}
};

static error_t
parse_opt (int key, char *arg, struct argp_state *state)
{
  /* Get the input argument from argp_parse, which we
   * know is a pointer to our ProgramOptions struct.
   */
	openEV::ProgramOptions &programOptions = *((openEV::ProgramOptions*)(state->input));

    switch (key)
        {
        case 'q': case 's':
          programOptions.defaultLoggerLevel = 0;
          break;
        case 'd':
          programOptions.defaultLoggerLevel ++;
          if (programOptions.defaultLoggerLevel > 3) {programOptions.defaultLoggerLevel = 3;}
          break;
        case 'c':
          programOptions.configFile = arg;
          break;

        case 'l':
          programOptions.loggerConfigFile = arg;
          break;

        default:
          return ARGP_ERR_UNKNOWN;
        }
    return 0;
}


#else /* HAVE_ARGP_PARSE == 1 */
    #if HAVE_GETOPT_LONG == 1

static struct option longOptions[] =
  {
          {"configuration"          ,  required_argument, 0, 'c'},
          {"logger-configuration"   ,  required_argument, 0, 'l'},
          {"debug"                  ,  no_argument      , 0, 'd'},
          {"quiet"                  ,  no_argument      , 0, 'q'},
          {"silent"                 ,  no_argument      , 0, 's'},
          {"help"                   ,  no_argument      , 0, 'h'},
          {"usage"                  ,  no_argument      , 0, 'h'},
          {"version"                ,  no_argument      , 0, 'V'},

          {0, 0, 0, 0}
  };


    #else /* HAVE_GETOPT_LONG == 1 */
        #if HAVE_GETOPT_LONG == 1
        #else /* HAVE_GETOPT_LONG == 1 */
        #endif /* HAVE_GETOPT_LONG == 1 */
    #endif /* HAVE_GETOPT_LONG == 1 */
#endif /* HAVE_ARGP_PARSE == 1 */

#if HAVE_ARGP_PARSE != 1

static void usage(std::ostream& outStr){
    static char const * const usageText =

#if HAVE_GETOPT_LONG == 1

"            Usage: openEVario [OPTION...]\n"
"\n"
"             openEVario: Electronic variometer application using inertial and pressure sensors \n"
"            (typically I2C on ARM SoC) and GPS input.\n"
"\n"
"              -c, --configuration=configFileName\n"
"                                         Name of the configuration file [" defaultConfigFileName "]\n"
"              -d, --debug                Increase default logger level\n"
"                                         (Silent-[Error]-Info-Debug)\n"
"              -l, --logger-configuration=loggerConfigFile\n"
"                                         Name of logger configuration file\n"
"                                         [" defaultLoggerConfigFileName "]\n"
"              -q, -s, --quiet, --silent  Shhhh. Be quiet. Suppress any logger output,\n"
"                                         i.e. set logger level to Silent (see -d)\n"
"              -?, --help                 Give this help list\n"
"                  --usage                Give a short usage message\n"
"              -V, --version              Print program version\n"
"\n"
"            Mandatory or optional arguments to long options are also mandatory or optional\n"
"            for any corresponding short options.\n"
"\n"
"            Report bugs to https://github.com/hor63/openEVario/issues."

#else /* HAVE_GETOPT_LONG == 1 */

"            Usage: openEVario [OPTION...]\n"
"\n"
"             openEVario: Electronic variometer application using inertial and pressure sensors \n"
"            (typically I2C on ARM SoC) and GPS input.\n"
"\n"
"              -c configFileName\n"
"                                         Name of the configuration file [./openEvario.cfg]\n"
"              -d                         Increase default logger level\n"
"                                         (Silent-[Error]-Info-Debug)\n"
"              -l loggerConfigFile        Name of logger configuration file\n"
"                                         [./log4cxx.properties]\n"
"              -q, -s                     Shhhh. Be quiet. Suppress any logger output,\n"
"                                         i.e. set logger level to Silent (see -d)\n"
"              -?                         Give this help list\n"
"              -V                         Print program version\n"
"\n"
"            Mandatory or optional arguments to long options are also mandatory or optional\n"
"            for any corresponding short options.\n"
"\n"
"            Report bugs to https://github.com/hor63/openEVario/issues.\n"


#endif /* HAVE_GETOPT_LONG == 1 */
;

    outStr << usageText << std::endl;

}

#endif  // HAVE_ARGP_PARSE != 1

/**
 * Reads command line arguments and extracts options
 *
 * @param argc Number of arguments
 * @param argv Array of C strings containing the arguments
 * @param programOptions Reference to struct openEV::ProgramOptions which will receive the option values.
 * @return
 */
static int readOptions (int& argc, char*argv[],openEV::ProgramOptions &programOptions) {
    int rc = 0;

#if HAVE_ARGP_PARSE == 1

    static struct argp arrgp = {options, parse_opt, 0, argpDoc};

    rc = argp_parse (&arrgp, argc, argv, 0, 0, &programOptions);

#else /* HAVE_ARGP_PARSE == 1 */

    int key;

    do {
        int optionIndex = 0;
#if HAVE_GETOPT_LONG == 1

         key = getopt_long (argc, argv, ":c:l:dqsh?",
                          longOptions, &optionIndex);
#else /* HAVE_GETOPT_LONG == 1 */
         key = getopt (argc, argv, ":c:l:dqs?V");
#endif /* HAVE_GETOPT_LONG == 1 */

         /* Detect the end of the options. */
         if (key == -1)
           break;

         switch (key)
             {
             case 'q': case 's':
               programOptions.defaultLoggerLevel = 0;
               break;
             case 'd':
               programOptions.defaultLoggerLevel ++;
               if (programOptions.defaultLoggerLevel > 3) {programOptions.defaultLoggerLevel = 3;}
               break;
             case 'c':
               programOptions.configFile = optarg;
               break;

             case 'l':
               programOptions.loggerConfigFile = optarg;
               break;

             case '?':
             case 'h':
               usage(std::cout);
               exit(0);
               break;

             case ':':
                 if (argv[0]) {
                     std::cerr << "Try `" << argv[0] << " --help' or `" << argv[0] << " --usage' for more information." << std::endl;
                 }
                 break;

             default:
                 if (argv[0]) {
                	 std::ostringstream errMsg;
                	 errMsg << argv[0] << ": getopt_long returned unexpected value" << key <<". Program aborting";
                	 throw openEV::GliderVarioExceptionBase(__FILE__,__LINE__,errMsg.str().c_str());
                 }
              exit(1);
             }


    } while (key != -1);

#endif /* HAVE_ARGP_PARSE == 1 */


    return rc;
}




namespace openEV {


GliderVarioMainPriv::GliderVarioMainPriv(int argc, const char *argv[])
	:driverList {programOptions}
{

	int i;

#if defined HAVE_LOG4CXX_H
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Main.GliderVarioMain");
	}
	if (!rootLogger) {
		rootLogger = log4cxx::Logger::getRootLogger();
	}
#endif /* HAVE_LOG4CXX_H */

	lastPredictionUpdate = std::chrono::system_clock::now();

	this->argc = argc;

	if (argc > 0) {

		this->argv = (char**) (malloc( sizeof(char*) * argc));

		for (i=0;i < argc; i++) {
			this->argv[i] = new char[strlen(argv[i])+1];
			strcpy(this->argv[i],argv[i]);
		}
	} else {
		this->argv = NULL;
	}

}

GliderVarioMainPriv::~GliderVarioMainPriv() {

	int i;

	if (argc > 0) {
		for (i=0;i < argc; i++) {
			delete argv[i];
		}

		free (argv);
	}


}

void GliderVarioMainPriv::startup () {
    int rc = 0;

    rc = readOptions (argc,argv,programOptions);

#if defined HAVE_LOG4CXX_H

	log4cxx::BasicConfigurator::configure();

    // Silent-[Error]-Info-Debug
    switch (programOptions.defaultLoggerLevel) {

    	case 0: // Silent. Fatal errors causing immediate termination are still reported.
    	    rootLogger->setLevel(log4cxx::Level::getFatal());
    	    break;

    	case 1:
    		rootLogger->setLevel(log4cxx::Level::getError());
    	    break;

		case 3:
			rootLogger->setLevel(log4cxx::Level::getDebug());
			break;

		case 2:
		default:
			rootLogger->setLevel(log4cxx::Level::getInfo());

    }


    // The configuration file (when I can load it) will overwrite the command line settings.
    log4cxx::PropertyConfigurator::configure(log4cxx::File(programOptions.loggerConfigFile));

#endif /* defined HAVE_LOG4CXX_H */

	std::cout << "logger->getName() " << logger->getName() << std::endl;
	std::cout << "logger->getLevel() " << logger->getLevel() << std::endl;
	std::cout << "logger->getParent()->getName() " << logger->getParent()->getName() << std::endl;
	std::cout << "logger->getParent()->getLevel() " << logger->getParent()->getLevel() << std::endl;

    LOG4CXX_INFO(logger,"programOptions.configFile = " << programOptions.configFile);
    LOG4CXX_INFO(logger,"programOptions.defaultLoggerLevel = " << programOptions.defaultLoggerLevel);
    LOG4CXX_INFO(logger,"programOptions.loggerConfigFile = " << programOptions.loggerConfigFile);

	LOG4CXX_DEBUG(logger,"argc = " << argc);
	if (argc > 0) {

		int i;

		for (i=0;i < argc; i++) {
			LOG4CXX_DEBUG(logger,"argv[" << i << "] = \"" << argv[i] << "\"");
		}
	}

	// Register the communication port drivers.
	// The port drivers are static
	// in contrast to the device drivers which are dynamic loaded shared libraries.
	registerPortDrivers();

	// Read the configuration into memory and read out the global configuration properties.
	readConfiguration();

	// Read the port configuration and create and load the ports
	io::PortBase::loadPorts(configuration);

	// Read the driver shared libraries, open them and initialize the driver libraries
	// Register the drivers implemented by the libraries
    driverList.loadDriverLibs(configuration);

    // Read the driver instances from the configuration, and create them for the specified drivers.
	driverList.loadDriverInstances(configuration);

	// Perform additional initialization of drivers which depend on all drivers been loaded.
	driverList.initDrivers(*this);

    // Start the internal threads of the drivers. These will open the ports, and start acquiring sensor data
    driverList.startupDrivers(*this);

	intializeStatus();

}

void GliderVarioMainPriv::readConfiguration () {


	try {
	configuration.setFileName(programOptions.configFile);
	configuration.readConfiguration();
	} catch (Properties4CXX::ExceptionBase const& e) {
		std::ostringstream os;
		os << "Error reading the configuration from file \"" << programOptions.configFile << "\": " << e.what();
		LOG4CXX_FATAL(logger,os.str());
		throw;
	}

	try {
		Properties4CXX::Property const* prop = configuration.searchProperty("terminateOnDriverLoadError");
		if (prop && prop->isBool()) {
			programOptions.terminateOnDriverLoadError = prop->getBoolValue();
		} else {
			LOG4CXX_ERROR(logger, "Property \"terminateOnDriverLoadError\" is not boolean. Use default value");
		}
	} catch (Properties4CXX::ExceptionBase const& e) {
			LOG4CXX_DEBUG(logger,"Property \"terminateOnDriverLoadError\" does not exist. Use default value");
		}
	LOG4CXX_DEBUG(logger, "programOptions.terminateOnDriverLoadError = " << programOptions.terminateOnDriverLoadError);

	try {
		Properties4CXX::Property const* prop = configuration.searchProperty("idlePredictionCycle");
		if (prop && (prop->isDouble()||prop->isInteger())) {
			programOptions.idlePredictionCycleMilliSec =  prop->getDoubleValue();
			programOptions.idlePredictionCycle = std::chrono::milliseconds( prop->getIntVal());
		} else {
			LOG4CXX_ERROR(logger, "Property \"idlePredictionCycle\" is not double or integer. Use default value");
		}
	} catch (Properties4CXX::ExceptionBase const& e) {
			LOG4CXX_DEBUG(logger,"Property \"idlePredictionCycle\" does not exist. Use default value");
		}
	LOG4CXX_DEBUG(logger, "programOptions.idlePredictionCycle = " << programOptions.idlePredictionCycle.count());
	LOG4CXX_DEBUG(logger, "programOptions.idlePredictionCycleMilliSec = " << programOptions.idlePredictionCycleMilliSec);

	try {
		Properties4CXX::Property const* prop = configuration.searchProperty("maxTimeBetweenPredictionAndMeasurementUpdate");
		if (prop && (prop->isDouble()||prop->isInteger())) {
			programOptions.maxTimeBetweenPredictionAndMeasurementUpdate = std::chrono::milliseconds(prop->getIntVal());
		} else {
			LOG4CXX_ERROR(logger, "Property \"maxTimeBetweenPredictionAndMeasurementUpdate\" is not double or integer. Use default value");
		}
	} catch (Properties4CXX::ExceptionBase const& e) {
			LOG4CXX_DEBUG(logger,"Property \"maxTimeBetweenPredictionAndMeasurementUpdate\" does not exist. Use default value");
		}
	LOG4CXX_DEBUG(logger, "programOptions.maxTimeBetweenPredictionAndMeasurementUpdate = " << programOptions.maxTimeBetweenPredictionAndMeasurementUpdate.count());


}


GliderVarioStatus *GliderVarioMainPriv::getCurrentStatusAndLock(GliderVarioMeasurementVector* & measurementVector) {

	currentStatusLock.lock();

	predictAndSwapStatus();

	measurementVector = &(this->measurementVector);

	return currentStatus;

}


void GliderVarioMainPriv::releaseCurrentStatus () {
	currentStatusLock.unlock();
}

void GliderVarioMainPriv::predictAndSwapStatus() {

	auto timeBeforePredict = std::chrono::system_clock::now();

	auto timeDiff = timeBeforePredict - lastPredictionUpdate;

	if (timeDiff >= programOptions.maxTimeBetweenPredictionAndMeasurementUpdate) {

		// Calculate fractional seconds from the system clock ticks with double arithmetic
		FloatType timeDiffSec = double(timeDiff.count()) * (double(std::chrono::system_clock::period::num)/double(std::chrono::system_clock::period::den));

		transitionMatrix.updateStatus(
				*currentStatus,
				*nextStatus,
				timeDiffSec);

		lastPredictionUpdate = timeBeforePredict;

		// Swap status buffers
		auto tempStatus = currentStatus;
		currentStatus = nextStatus;
		nextStatus = tempStatus;
	}

}

void GliderVarioMainPriv::registerPortDrivers() {

#if WITH_IP_PORT_DRIVERS
	io::TCPPort::registerTcpPortType();
	io::UDPPort::registerUdpPortType();
#endif

#if WITH_SERIAL_PORT_DRIVER
	io::SerialPort::registerSerialPortType();
#endif

}

void GliderVarioMainPriv::intializeStatus() {

	// Initialize the first Kalman status with initial sensor readings
	driverList.initializeKalmanStatus(*currentStatus,*this);

	double baseIntervalSec = programOptions.idlePredictionCycleMilliSec / 1000.0;
	LOG4CXX_DEBUG(logger,"baseIntervalSec = " << baseIntervalSec);


#define SQUARE(x) ((x)*(x))

	//
	// Initialize the components which were not initialized by the drivers
	// Note each value which is being initialized by which sensor driver by driverList.initializeKalmanStatus() above.
	// This initialization of course applies only when the driver is actually configured, and active
	//
	// Variance values here are usually set a bit tamer based on the assumption of a 1-second rate updated by position and altitude only
	// Drivers with very short cycles will set the variances much more aggressive.
	//

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GRAVITY,currentStatus->STATUS_IND_GRAVITY) == 0.0f) {
		currentStatus->gravity = GRAVITY;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GRAVITY,currentStatus->STATUS_IND_GRAVITY) = 0.01f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GRAVITY,currentStatus->STATUS_IND_GRAVITY) =
				SQUARE(0.1) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS) == 0.0) {
		// Lüneburg airport EDHG
		currentStatus->latitude(53.2483333333);
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS) = 10000.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS) =
				SQUARE(7.0) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS) == 0.0f) {
		// Lüneburg airport EDHG
		currentStatus->longitude(10.4586111111);
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS) = 10000.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS) =
				SQUARE(7.0) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL) == 0.0f) {
		// Lüneburg airport EDHG
		currentStatus->altMSL = 49.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL) = 1000.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL) =
				SQUARE(10.0) * baseIntervalSec;
	}

	// Initialized by magnetometer, e.g. BMXSensorBoardDriver::initializeStatusMag()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) == 0.0f) {
		currentStatus->heading = 45.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) = 10.0f * 10.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) =
				SQUARE(5.0) * baseIntervalSec;
	}


	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH) == 0.0f) {
		currentStatus->pitchAngle = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH) = 10.0f * 10.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH) =
				SQUARE(5.0) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL) == 0.0f) {
		currentStatus->rollAngle = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL) = 10.0f * 10.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL) =
				SQUARE(5.0) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N) == 0.0f) {
		currentStatus->groundSpeedNorth = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N) =
				SQUARE(3.0) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E) == 0.0f) {
		currentStatus->groundSpeedEast = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E) =
				SQUARE(3.0) * baseIntervalSec;
	}

	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS) == 0.0f) {
		currentStatus->trueAirSpeed = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS) =
				SQUARE(1.0) * baseIntervalSec;
	}

	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK) == 0.0f) {
		currentStatus->rateOfSink = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK) = 50.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK) =
				SQUARE(1.0) * baseIntervalSec;
	}

	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED) == 0.0f) {
		currentStatus->verticalSpeed = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED) =
				SQUARE(3.0) * baseIntervalSec;
	}

    if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_THERMAL_SPEED,currentStatus->STATUS_IND_THERMAL_SPEED) == 0.0f) {
		currentStatus->thermalSpeed = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_THERMAL_SPEED,currentStatus->STATUS_IND_THERMAL_SPEED) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_THERMAL_SPEED,currentStatus->STATUS_IND_THERMAL_SPEED) =
				SQUARE(1.0) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING) == 0.0f) {
		currentStatus->accelHeading = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING) = 4.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING) =
				SQUARE(1.0) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_ACC_CROSS,currentStatus->STATUS_IND_ACC_CROSS) == 0.0f) {
		currentStatus->accelCross = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_CROSS,currentStatus->STATUS_IND_ACC_CROSS) = 1.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_CROSS,currentStatus->STATUS_IND_ACC_CROSS) =
				SQUARE(1.0) * baseIntervalSec;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL) == 0.0f) {
		currentStatus->accelVertical = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL) = 4.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL) =
				SQUARE(1.0) * baseIntervalSec;
	}

	// Initialized by gyroscope, e.g. BMXSensorBoardDriver::initializeStatusGyro()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_ROTATION_X,currentStatus->STATUS_IND_ROTATION_X) == 0.0f) {
		currentStatus->rollRateX = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_X,currentStatus->STATUS_IND_ROTATION_X) = 2.0f * 2.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_X,currentStatus->STATUS_IND_ROTATION_X) =
				SQUARE(3.0) * baseIntervalSec;
	}

	// Initialized by gyroscope, e.g. BMXSensorBoardDriver::initializeStatusGyro()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_ROTATION_Y,currentStatus->STATUS_IND_ROTATION_Y) == 0.0f) {
		currentStatus->pitchRateY = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_Y,currentStatus->STATUS_IND_ROTATION_Y) = 2.0f * 2.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Y,currentStatus->STATUS_IND_ROTATION_Y) =
				SQUARE(3.0) * baseIntervalSec;
	}

	// Initialized by gyroscope, e.g. BMXSensorBoardDriver::initializeStatusGyro()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z) == 0.0f) {
		currentStatus->yawRateZ = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z) = 2.0f * 2.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z) =
				SQUARE(3.0) * baseIntervalSec;
	}

	// Initialized by gyroscope, e.g. BMXSensorBoardDriver::initializeStatusGyro()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X) == 0.0f) {
		currentStatus->gyroBiasX = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X) = 2.0f * 2.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X) =
				SQUARE(0.1) * baseIntervalSec;
	}

	// Initialized by gyroscope, e.g. BMXSensorBoardDriver::initializeStatusGyro()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y) == 0.0f) {
		currentStatus->gyroBiasY = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y) = 2.0f * 2.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y) =
				SQUARE(0.1) * baseIntervalSec;
	}

	// Initialized by gyroscope, e.g. BMXSensorBoardDriver::initializeStatusGyro()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z) == 0.0f) {
		currentStatus->gyroBiasZ = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z) = 2.0f * 2.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_MAGNETIC_DECLINATION,currentStatus->STATUS_IND_MAGNETIC_DECLINATION) == 0.0f) {
		currentStatus->magneticDeclination = 2.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_MAGNETIC_DECLINATION,currentStatus->STATUS_IND_MAGNETIC_DECLINATION) = 10.0f * 10.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_MAGNETIC_DECLINATION,currentStatus->STATUS_IND_MAGNETIC_DECLINATION) =
				SQUARE(0.1) * baseIntervalSec;
	}

	// Initialized by magnetometer, e.g. BMXSensorBoardDriver::initializeStatusMag()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_MAGNETIC_INCLINATION,currentStatus->STATUS_IND_MAGNETIC_INCLINATION) == 0.0f) {
		currentStatus->magneticInclination = MAG_INCLINATION;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_MAGNETIC_INCLINATION,currentStatus->STATUS_IND_MAGNETIC_INCLINATION) = 10.0f * 10.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_MAGNETIC_INCLINATION,currentStatus->STATUS_IND_MAGNETIC_INCLINATION) =
				SQUARE(0.1) * baseIntervalSec;
	}

	// Initialized by magnetometer, e.g. BMXSensorBoardDriver::initializeStatusMag()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_COMPASS_DEVIATION_X,currentStatus->STATUS_IND_COMPASS_DEVIATION_X) == 0.0f) {
		currentStatus->compassDeviationX = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_X,currentStatus->STATUS_IND_COMPASS_DEVIATION_X) = 50.0f * 50.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_X,currentStatus->STATUS_IND_COMPASS_DEVIATION_X) =
				SQUARE(0.1) * baseIntervalSec;
	}

	// Initialized by magnetometer, e.g. BMXSensorBoardDriver::initializeStatusMag()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_COMPASS_DEVIATION_Y,currentStatus->STATUS_IND_COMPASS_DEVIATION_Y) == 0.0f) {
		currentStatus->compassDeviationY = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Y,currentStatus->STATUS_IND_COMPASS_DEVIATION_Y) = 50.0f * 50.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Y,currentStatus->STATUS_IND_COMPASS_DEVIATION_Y) =
				SQUARE(0.1) * baseIntervalSec;
	}

	// Initialized by magnetometer, e.g. BMXSensorBoardDriver::initializeStatusMag()
	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_COMPASS_DEVIATION_Z,currentStatus->STATUS_IND_COMPASS_DEVIATION_Z) == 0.0f) {
		currentStatus->compassDeviationZ = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Z,currentStatus->STATUS_IND_COMPASS_DEVIATION_Z) = 50.0f * 50.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Z,currentStatus->STATUS_IND_COMPASS_DEVIATION_Z) =
				SQUARE(0.1) * baseIntervalSec;
	}

	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_WIND_SPEED_N,currentStatus->STATUS_IND_WIND_SPEED_N) == 0.0f) {
		currentStatus->windSpeedNorth = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_N,currentStatus->STATUS_IND_WIND_SPEED_N) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_N,currentStatus->STATUS_IND_WIND_SPEED_N) =
				SQUARE(1.0) * baseIntervalSec;
	}

	if (currentStatus->getErrorCovariance_P().coeff(currentStatus->STATUS_IND_WIND_SPEED_E,currentStatus->STATUS_IND_WIND_SPEED_E) == 0.0f) {
		currentStatus->windSpeedEast = 0.0f;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_E,currentStatus->STATUS_IND_WIND_SPEED_E) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_E,currentStatus->STATUS_IND_WIND_SPEED_E) =
				SQUARE(1.0) * baseIntervalSec;
	}

	if (currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_QFF,currentStatus->STATUS_IND_QFF) == 0.0) {
		currentStatus->qff = pressureStdMSL;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_QFF,currentStatus->STATUS_IND_QFF) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_QFF,currentStatus->STATUS_IND_QFF) =
				SQUARE(0.1) * baseIntervalSec; // Slow (1mbar / 10sec)
	}

	if (currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LAST_PRESSURE,currentStatus->STATUS_IND_LAST_PRESSURE) == 0.0) {
		currentStatus->lastPressure = pressureStdMSL;
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LAST_PRESSURE,currentStatus->STATUS_IND_LAST_PRESSURE) = 100.0f;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LAST_PRESSURE,currentStatus->STATUS_IND_LAST_PRESSURE) =
				SQUARE(0.1) * baseIntervalSec;
	}

	LOG4CXX_DEBUG(logger,"GliderVarioMainPriv::intializeStatus():" );
	for (auto i = 0U; i < currentStatus->STATUS_NUM_ROWS;i++) {
		LOG4CXX_DEBUG(logger,'	' << GliderVarioStatus::StatusComponentIndex(i)
				<< ": Initial value = " << currentStatus->getStatusVector_x().coeff(i)
				<< ", Variance = " << currentStatus->getErrorCovariance_P().coeffRef(i,i)
				<< ", variance increment = "
				<< currentStatus->getSystemNoiseCovariance_Q().coeff(i,i) / baseIntervalSec << "/sec"
				);
	}

}

void GliderVarioMainPriv::startMainLoop() {

	driverList.runDrivers();

	if (!driverList.isDriverRunningIdleLoop()) {
		if (!idleLoopRunning) {
			// Stop signal has been sent
			if (idleLoopThread.joinable()) {
				// When the thread is still running wait until it terminates itself.
				idleLoopThread.join();
			}

			idleLoopRunning = true;
		}

		if (!idleLoopThread.joinable()) {
			// idleLoopRunning is now set but the thread is not running
			// therefore start it now
			idleLoopThread = std::thread (GliderVarioMainPriv::idleLoopTreadEntry,this);
		}
	}

}

void GliderVarioMainPriv::stopMainLoop() {

	driverList.stopDrivers();

	idleLoopRunning = false;

	if (idleLoopThread.joinable()) {
		idleLoopThread.join();
	}

}

void GliderVarioMainPriv::idleLoop() {

	std::chrono::system_clock::time_point nextLoopTime =  std::chrono::system_clock::now();
	std::chrono::system_clock::time_point now;

	while (idleLoopRunning) {
		GliderVarioMeasurementVector* measVector = 0;

		// just retrieve the current status, and immediately release it.
		// If the interval was exceeded the function will automatically calculate the next prediction.
		getCurrentStatusAndLock(measVector);
		releaseCurrentStatus();

		now = std::chrono::system_clock::now();
		do {
			nextLoopTime += programOptions.idlePredictionCycle;

		} while (nextLoopTime <= now);

		// go to bed
		std::this_thread::sleep_until(nextLoopTime);
	}


}

void GliderVarioMainPriv::getAndLockInternalStatusForDebug (
		GliderVarioStatus **&currentStatus,
		GliderVarioStatus **&nextStatus,
		GliderVarioTransitionMatrix *&transitionMatrix,
		GliderVarioMeasurementVector *&measurementVector
		) {

	currentStatusLock.lock();

	currentStatus		= &(this->currentStatus);
	nextStatus			= &(this->nextStatus);
	transitionMatrix	= &(this->transitionMatrix);
	measurementVector	= &(this->measurementVector);

}

void GliderVarioMainPriv::idleLoopTreadEntry (GliderVarioMainPriv *vario) {
	vario->idleLoop();
}

} /* namespace openEV */

