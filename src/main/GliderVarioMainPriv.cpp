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
#  include "config.h"
#endif

#if HAVE_ARGP_H == 1
#	include <argp.h>
#endif

// Another options library
#if HAVE_GETOPT_H == 1
#	include <getopt.h>
#endif

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "fmt/format.h"

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

#if HAVE_I2C_H
#	include "util/io/I2CPort.h"
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
          if (programOptions.defaultLoggerLevel > 4) {programOptions.defaultLoggerLevel = 4;}
          break;
        case 'c':
          programOptions.configFileName = arg;
          break;

        case 'l':
          programOptions.loggerConfigFileName = arg;
          break;

        default:
          return ARGP_ERR_UNKNOWN;
        }
    return 0;
}


#else /* HAVE_ARGP_PARSE == 1 */
#	if HAVE_GETOPT_LONG == 1

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

#	endif /* HAVE_GETOPT_LONG == 1 */
#endif /* HAVE_ARGP_PARSE == 1 */

//#if HAVE_ARGP_PARSE != 1

static void usage(std::ostream& outStr){

#if HAVE_GETOPT_LONG == 1
    std::string const usageText = fmt::format(
    		_(
    				"            Usage: openEVario [OPTION...]\n"
    				"\n"
    				"              -c, --configuration=configFileName\n"
    				"                                         Name of the configuration file [{0}]\n"
    				"              -d, --debug                Increase default logger level\n"
    				"                                         (Silent-[Error]-Info-Debug-Trace)\n"
    				"              -l, --logger-configuration=loggerConfigFile\n"
    				"                                         Name of logger configuration file\n"
    				"                                         [{1}]\n"
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
    		)
			,defaultConfigFileName,defaultLoggerConfigFileName);

#else /* HAVE_GETOPT_LONG == 1 */
    std::string const usageText = fmt::format(
    		_(
    				"            Usage: openEVario [OPTION...]\n"
    				"\n"
    				"              -c configFileName\n"
    				"                                         Name of the configuration file [{0}]\n"
    				"              -d                         Increase default logger level\n"
    				"                                         (Silent-[Error]-Info-Debug-Trace)\n"
    				"              -l loggerConfigFile        Name of logger configuration file\n"
    				"                                         [{1}]\n"
    				"              -q, -s                     Shhhh. Be quiet. Suppress any logger output,\n"
    				"                                         i.e. set logger level to Silent (see -d)\n"
    				"              -?                         Give this help list\n"
    				"              -V                         Print program version\n"
    				"\n"
    				"            Report bugs to https://github.com/hor63/openEVario/issues.\n"
    		)
			,defaultConfigFileName,defaultLoggerConfigFileName);

#endif /* HAVE_GETOPT_LONG == 1 */

    outStr << usageText << std::endl;

}

//#endif  // HAVE_ARGP_PARSE != 1

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

    static std::string configFileDoc = fmt::format(_("Name of the configuration file [{0}]"),defaultConfigFileName);
    static std::string loggerConfigFileDoc = fmt::format(_("Name of logger configuration file [{0}]"),defaultLoggerConfigFileName);

    static struct argp_option options[] = {
      {"configuration",        'c', "configFileName",   0, configFileDoc.c_str(),0},
      {"logger-configuration", 'l', "loggerConfigFile", 0, loggerConfigFileDoc.c_str(),0},
      {"debug",                'd', 0,                  0, _("Increase default logger level (Silent-[Error]-Info-Debug-Trace)"),0},
      {"quiet",                'q', 0,                  0, _("Shhhh. Be quiet. Suppress any logger output, i.e. set logger level to Silent"),0},
      {"silent",               's', 0,                  OPTION_ALIAS, 0 ,0},
      {0,0,0,0,0,0}
    };


#if HAVE_ARGP_PARSE == 1

    static struct argp arrgp = {options, parse_opt, 0, argpDoc,0,0,0};

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
               if (programOptions.defaultLoggerLevel > 4) {programOptions.defaultLoggerLevel = 4;}
               break;
             case 'c':
               programOptions.configFileName = optarg;
               break;

             case 'l':
               programOptions.loggerConfigFileName = optarg;
               break;

             case '?':
             case 'h':
               usage(std::cout);
               exit(0);
               break;

             case ':':
                 if (argv[0]) {
                     std::cerr << std::format(_("Try \"{0} --help\" or \"{1} --usage\" for more information."),argv[0],argv[0]) << std::endl;
                 }
                 break;

             default:
                 if (argv[0]) {
                	 auto errMsg = fmt::format(_("{0}: getopt_long returned unexpected value {1}. Program aborting"),argv[0],key);
                	 throw openEV::GliderVarioExceptionBase(__FILE__,__LINE__,errMsg.c_str());
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

	lastPredictionUpdate = OEVClock::now();

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

#if defined ENABLE_NLS && ENABLE_NLS
	bindtextdomain (PACKAGE, LOCALEDIR);
#endif // #if defined ENABLE_NLS && ENABLE_NLS


	readOptions (argc,argv,programOptions);

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

		case 4:
			rootLogger->setLevel(log4cxx::Level::getTrace());
			break;

		case 2:
		default:
			rootLogger->setLevel(log4cxx::Level::getInfo());

    }


    // The configuration file (when I can load it) will overwrite the command line settings.
    log4cxx::PropertyConfigurator::configure(log4cxx::File(programOptions.loggerConfigFileName));

    LOG4CXX_INFO(logger,fmt::format(_("{0}, version {1} starting up."),PACKAGE_NAME,PACKAGE_VERSION));

#endif /* defined HAVE_LOG4CXX_H */

	std::cout << "logger->getName() " << logger->getName() << std::endl;
	std::cout << "logger->getLevel() " << logger->getLevel() << std::endl;
	std::cout << "logger->getParent()->getName() " << logger->getParent()->getName() << std::endl;
	std::cout << "logger->getParent()->getLevel() " << logger->getParent()->getLevel() << std::endl;

    LOG4CXX_INFO(logger,"programOptions.configFile = " << programOptions.configFileName);
    LOG4CXX_INFO(logger,"programOptions.defaultLoggerLevel = " << programOptions.defaultLoggerLevel);
    LOG4CXX_INFO(logger,"programOptions.loggerConfigFile = " << programOptions.loggerConfigFileName);

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

    // Initialize the status to UnInitVal. Let the drivers initialize the Kalman status with initial measurements.
    // Finally initialize all remaining items still at UnInitVal with default values.
	intializeStatus();

	driverList.startupCalibrationDataUpdateThread();

}

void GliderVarioMainPriv::readConfiguration () {


	try {
	configuration.setFileName(programOptions.configFileName);
	configuration.readConfiguration();
	} catch (Properties4CXX::ExceptionBase const& e) {

		std::string errStr = fmt::format(_("Error reading the configuration from file \"{0}\" is: {1}"),
				programOptions.configFileName, e.what());
		LOG4CXX_FATAL(logger,errStr);
		throw GliderVarioFatalConfigException(__FILE__,__LINE__,errStr.c_str());
	}

	try {
		Properties4CXX::Property const* prop = configuration.searchProperty("terminateOnDriverLoadError");
		if (prop && prop->isBool()) {
			programOptions.terminateOnDriverLoadError = prop->getBoolValue();
		} else {
			LOG4CXX_ERROR(logger, fmt::format(_("Property \"terminateOnDriverLoadError\" is not boolean. Use default value {0}"),
					programOptions.terminateOnDriverLoadError));
		}
	} catch (Properties4CXX::ExceptionBase const& e) {
			LOG4CXX_DEBUG(logger,"Property \"terminateOnDriverLoadError\" does not exist. Use default value ");
		}
	LOG4CXX_DEBUG(logger, "programOptions.terminateOnDriverLoadError = " << programOptions.terminateOnDriverLoadError);

	try {
		Properties4CXX::Property const* prop = configuration.searchProperty("idlePredictionCycle");
		if (prop && (prop->isDouble()||prop->isInteger())) {
			programOptions.idlePredictionCycleMilliSec =  prop->getDoubleValue();
			programOptions.idlePredictionCycle = std::chrono::milliseconds( prop->getIntVal());
		} else {
			LOG4CXX_ERROR(logger, fmt::format(_("Property \"idlePredictionCycle\" is not double or integer. Use default value {0}ms"),
					programOptions.idlePredictionCycleMilliSec));
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
			LOG4CXX_ERROR(logger, fmt::format(_("Property \"maxTimeBetweenPredictionAndMeasurementUpdate\" is not double or integer. Use default value {0}ms"),
					std::chrono::duration_cast<std::chrono::milliseconds>(programOptions.maxTimeBetweenPredictionAndMeasurementUpdate).count()));
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

	auto timeBeforePredict = OEVClock::now();

	auto timeDiff = timeBeforePredict - lastPredictionUpdate;

	if (timeDiff >= programOptions.maxTimeBetweenPredictionAndMeasurementUpdate) {

		// Calculate fractional seconds from the system clock ticks with double arithmetic
		FloatType timeDiffSec = double(timeDiff.count()) * (double(OEVClock::period::num)/double(OEVClock::period::den));

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

#if HAVE_I2C_DEV_H
	io::I2CPort::registerI2CPortType();
#else
#	error Sorry folks. Use of the I2C Linux kernel device interface is hard-coded in the driver. Without most sensors do no work.
#endif

}

void GliderVarioMainPriv::intializeStatus() {

	double baseIntervalSec = programOptions.idlePredictionCycleMilliSec / 1000.0;
	LOG4CXX_DEBUG(logger,"baseIntervalSec = " << baseIntervalSec);


#define SQUARE(x) ((x)*(x))

	//
	// First set all values to UnInitVal.
	// Then let the drivers initialize the status and variances
	// What is afterwards still UnInitVal will be initialized with default values down below.
	//

	for (int i=0;i < currentStatus->STATUS_NUM_ROWS; i++) {
		currentStatus->getErrorCovariance_P().coeffRef(i,i) = UnInitVal;
		currentStatus->getStatusVector_x()(i) = UnInitVal;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(i,i) = UnInitVal;
	}

	// Re-initialize some values like in the constructor.
	currentStatus->getStatusVector_x()(GliderVarioStatus::STATUS_IND_GRAVITY)= GRAVITY;
	currentStatus->getStatusVector_x()(GliderVarioStatus::STATUS_IND_LAST_PRESSURE) = PressureStdMSL;
	currentStatus->getStatusVector_x()(GliderVarioStatus::STATUS_IND_QFF) = PressureStdMSL;


	// Initialize the Kalman status with initial sensor readings
	// and variances based on sensor cycle and performance
	driverList.initializeKalmanStatus(*currentStatus,measurementVector,*this);

	setKalmanErrorIncrements ();

	//
	// Initialize the components which were not initialized by the drivers
	// Note each value which is being initialized by which sensor driver by driverList.initializeKalmanStatus() above.
	// This initialization of course applies only when the driver is actually configured, and active
	//
	// Variance values here are usually set a bit tamer based on the assumption of a 1-second rate updated by position and altitude only
	// Drivers with very short cycles will set the variances much more aggressive.
	//

	if (UnInitVal == currentStatus->gravity) {
		currentStatus->gravity = GRAVITY;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GRAVITY,currentStatus->STATUS_IND_GRAVITY)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GRAVITY,currentStatus->STATUS_IND_GRAVITY) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GRAVITY,currentStatus->STATUS_IND_GRAVITY)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GRAVITY,currentStatus->STATUS_IND_GRAVITY) =
				0.0f;
	}

	if (UnInitVal == currentStatus->latitudeOffsC) {
		// Lüneburg airport EDHG
		currentStatus->latitude(53.2483333333);
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS) =
				0.0f;
	}

	if (UnInitVal == currentStatus->longitudeOffsC) {
		// Lüneburg airport EDHG
		currentStatus->longitude(10.4586111111);
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS) =
				0.0f;
	}

	if (UnInitVal == currentStatus->altMSL) {
		// Lüneburg airport EDHG
		currentStatus->altMSL = 49.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL)) {
	currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL) =
				0.0f;
	}

	// Initialized by magnetometer, e.g. BMXSensorBoardDriver::initializeStatusMag()
	if (UnInitVal == currentStatus->heading) {
		currentStatus->heading = 45.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) =
				0.0f;
	}

	if (UnInitVal == currentStatus->pitchAngle) {
		currentStatus->pitchAngle = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH) =
				0.0f;
	}

	// Initialized by accelerometer, e.g. BMXSensorBoardDriver::initializeStatusAccel()
	if (UnInitVal == currentStatus->rollAngle) {
		currentStatus->rollAngle = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL) =
				0.0f;
	}

	if (UnInitVal == currentStatus->groundSpeedNorth) {
		currentStatus->groundSpeedNorth = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N) =
				0.0f;
	}

	if (UnInitVal == currentStatus->groundSpeedEast) {
		currentStatus->groundSpeedEast = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E) =
				0.0f;
	}

	if (UnInitVal == currentStatus->trueAirSpeed) {
		currentStatus->trueAirSpeed = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS) =
				0.0f;
	}

	if (UnInitVal == currentStatus->rateOfSink) {
		currentStatus->rateOfSink = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK) =
				0.0f;

	}
	if (UnInitVal == currentStatus->verticalSpeed) {
		currentStatus->verticalSpeed = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED) =
				0.0f;
	}
	if (UnInitVal == currentStatus->thermalSpeed) {
		currentStatus->thermalSpeed = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_THERMAL_SPEED,currentStatus->STATUS_IND_THERMAL_SPEED)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_THERMAL_SPEED,currentStatus->STATUS_IND_THERMAL_SPEED) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_THERMAL_SPEED,currentStatus->STATUS_IND_THERMAL_SPEED)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_THERMAL_SPEED,currentStatus->STATUS_IND_THERMAL_SPEED) =
				0.0f;
	}

	if (UnInitVal == currentStatus->accelHeading) {
		currentStatus->accelHeading = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING) =
				0.0f;
	}

	if (UnInitVal == currentStatus->accelCross) {
		currentStatus->accelCross = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_CROSS,currentStatus->STATUS_IND_ACC_CROSS)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_CROSS,currentStatus->STATUS_IND_ACC_CROSS) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_CROSS,currentStatus->STATUS_IND_ACC_CROSS)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_CROSS,currentStatus->STATUS_IND_ACC_CROSS) =
				0.0f;
	}

	if (UnInitVal == currentStatus->accelVertical) {
		currentStatus->accelVertical = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL) =
				0.0f;
	}

	if (UnInitVal == currentStatus->rollRateX) {
		currentStatus->rollRateX = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_X,currentStatus->STATUS_IND_ROTATION_X)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_X,currentStatus->STATUS_IND_ROTATION_X) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_X,currentStatus->STATUS_IND_ROTATION_X)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_X,currentStatus->STATUS_IND_ROTATION_X) =
				0.0f;
	}

	if (UnInitVal == currentStatus->pitchRateY) {
		currentStatus->pitchRateY = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_Y,currentStatus->STATUS_IND_ROTATION_Y)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_Y,currentStatus->STATUS_IND_ROTATION_Y) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Y,currentStatus->STATUS_IND_ROTATION_Y)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Y,currentStatus->STATUS_IND_ROTATION_Y) =
				0.0f;
	}

	if (UnInitVal == currentStatus->yawRateZ) {
		currentStatus->yawRateZ = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z) =
				0.0f;
	}

	if (UnInitVal == currentStatus->gyroBiasX) {
		currentStatus->gyroBiasX = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X) =
				0.0f;
	}

	if (UnInitVal == currentStatus->gyroBiasY) {
		currentStatus->gyroBiasY = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y) =
				0.0f;
	}

	if (UnInitVal == currentStatus->gyroBiasZ) {
		currentStatus->gyroBiasZ = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z) =
				0.0f;
	}

	if (UnInitVal == currentStatus->magneticDeclination) {
		currentStatus->magneticDeclination = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_MAGNETIC_DECLINATION,currentStatus->STATUS_IND_MAGNETIC_DECLINATION)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_MAGNETIC_DECLINATION,currentStatus->STATUS_IND_MAGNETIC_DECLINATION) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_MAGNETIC_DECLINATION,currentStatus->STATUS_IND_MAGNETIC_DECLINATION)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_MAGNETIC_DECLINATION,currentStatus->STATUS_IND_MAGNETIC_DECLINATION) =
				0.0f;
	}

	if (UnInitVal == currentStatus->magneticInclination) {
		currentStatus->magneticInclination = MAG_INCLINATION;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_MAGNETIC_INCLINATION,currentStatus->STATUS_IND_MAGNETIC_INCLINATION)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_MAGNETIC_INCLINATION,currentStatus->STATUS_IND_MAGNETIC_INCLINATION) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_MAGNETIC_INCLINATION,currentStatus->STATUS_IND_MAGNETIC_INCLINATION)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_MAGNETIC_INCLINATION,currentStatus->STATUS_IND_MAGNETIC_INCLINATION) =
				0.0f;
	}

	if (UnInitVal == currentStatus->compassDeviationX) {
		currentStatus->compassDeviationX = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_X,currentStatus->STATUS_IND_COMPASS_DEVIATION_X)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_X,currentStatus->STATUS_IND_COMPASS_DEVIATION_X) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_X,currentStatus->STATUS_IND_COMPASS_DEVIATION_X)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_X,currentStatus->STATUS_IND_COMPASS_DEVIATION_X) =
				0.0f;
	}

	if (UnInitVal == currentStatus->compassDeviationY) {
		currentStatus->compassDeviationY = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Y,currentStatus->STATUS_IND_COMPASS_DEVIATION_Y)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Y,currentStatus->STATUS_IND_COMPASS_DEVIATION_Y) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Y,currentStatus->STATUS_IND_COMPASS_DEVIATION_Y)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Y,currentStatus->STATUS_IND_COMPASS_DEVIATION_Y) =
				0.0f;
	}

	if (UnInitVal == currentStatus->compassDeviationZ) {
		currentStatus->compassDeviationZ = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Z,currentStatus->STATUS_IND_COMPASS_DEVIATION_Z)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Z,currentStatus->STATUS_IND_COMPASS_DEVIATION_Z) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Z,currentStatus->STATUS_IND_COMPASS_DEVIATION_Z)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_COMPASS_DEVIATION_Z,currentStatus->STATUS_IND_COMPASS_DEVIATION_Z) =
				0.0f;

	}

	if (UnInitVal == currentStatus->windSpeedNorth) {
		currentStatus->windSpeedNorth = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_N,currentStatus->STATUS_IND_WIND_SPEED_N)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_N,currentStatus->STATUS_IND_WIND_SPEED_N) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_N,currentStatus->STATUS_IND_WIND_SPEED_N)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_N,currentStatus->STATUS_IND_WIND_SPEED_N) =
				0.0f;
	}

	if (UnInitVal == currentStatus->windSpeedEast) {
		currentStatus->windSpeedEast = 0.0f;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_E,currentStatus->STATUS_IND_WIND_SPEED_E)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_E,currentStatus->STATUS_IND_WIND_SPEED_E) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_E,currentStatus->STATUS_IND_WIND_SPEED_E)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_E,currentStatus->STATUS_IND_WIND_SPEED_E) =
				0.0f;
	}


	if (UnInitVal == currentStatus->qff) {
		currentStatus->qff = PressureStdMSL;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_QFF,currentStatus->STATUS_IND_QFF)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_QFF,currentStatus->STATUS_IND_QFF) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_QFF,currentStatus->STATUS_IND_QFF)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_QFF,currentStatus->STATUS_IND_QFF) =
				0.0f;
	}

	if (UnInitVal == currentStatus->lastPressure) {
		currentStatus->lastPressure = PressureStdMSL;
	}
	if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LAST_PRESSURE,currentStatus->STATUS_IND_LAST_PRESSURE)) {
		currentStatus->getErrorCovariance_P().coeffRef(currentStatus->STATUS_IND_LAST_PRESSURE,currentStatus->STATUS_IND_LAST_PRESSURE) = 0.0f;
	}
	if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LAST_PRESSURE,currentStatus->STATUS_IND_LAST_PRESSURE)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LAST_PRESSURE,currentStatus->STATUS_IND_LAST_PRESSURE) = 0.0f;
	}

	LOG4CXX_DEBUG(logger,"GliderVarioMainPriv::intializeStatus():" );
	for (auto i = 0U; i < currentStatus->STATUS_NUM_ROWS;i++) {
		LOG4CXX_DEBUG(logger,'	' << GliderVarioStatus::StatusComponentIndex(i)
				<< ": Initial value = " << currentStatus->getStatusVector_x()(i)
				<< ", Variance = " << currentStatus->getErrorCovariance_P().coeffRef(i,i)
				<< ", variance increment = "
				<< currentStatus->getSystemNoiseCovariance_Q().coeff(i,i) / baseIntervalSec << "/sec"
				);


		if (UnInitVal == currentStatus->getStatusVector_x()(i)) {
			auto indexName = GliderVarioStatus::StatusComponentIndexHelperObj.getString(
					static_cast<GliderVarioStatus::StatusComponentIndex>(i));
			LOG4CXX_WARN(logger,fmt::format(
					_("{0}: currentStatus->getStatusVector_x()({1}) is not initialized."),
					__PRETTY_FUNCTION__,
					indexName));
			// throw GliderVarioExceptionBase(__FILE__, __LINE__, str.str().c_str());
		}
		if (UnInitVal == currentStatus->getErrorCovariance_P().coeffRef(i,i)) {
			auto indexName = GliderVarioStatus::StatusComponentIndexHelperObj.getString(
					static_cast<GliderVarioStatus::StatusComponentIndex>(i));
			LOG4CXX_WARN(logger,fmt::format(
					_("{0}: currentStatus->getErrorCovariance_P().coeffRef({1},{2}) is not initialized."),
					__PRETTY_FUNCTION__,
					indexName,indexName));
			// throw GliderVarioExceptionBase(__FILE__, __LINE__, str.str().c_str());
		}
		if (UnInitVal == currentStatus->getSystemNoiseCovariance_Q().coeffRef(i,i)) {
			auto indexName = GliderVarioStatus::StatusComponentIndexHelperObj.getString(
					static_cast<GliderVarioStatus::StatusComponentIndex>(i));
			LOG4CXX_WARN(logger,fmt::format(
					_("{0}: urrentStatus->getSystemNoiseCovariance_Q().coeffRef({1},{2}) is not initialized."),
					__PRETTY_FUNCTION__,
					indexName,indexName));
			// throw GliderVarioExceptionBase(__FILE__, __LINE__, str.str().c_str());
		}
	}

}

void GliderVarioMainPriv::setKalmanErrorIncrements () {
	double baseIntervalSec = programOptions.idlePredictionCycleMilliSec / 1000.0;

	// Set the error increment rate based on the connected sensor capabilities.
	// First sum up the capabilities of all instantiated drivers.
	uint_fast32_t allCapabilities = 0;
	for (auto & device : driverList.getDriverInstanceList()) {
		allCapabilities |= device.second->getSensorCapabilities();
	}

	// GPS Position
	if (allCapabilities & (1UL<<drivers::DriverBase::GPS_POSITION)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS) =
				SQUARE(3.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS) =
				SQUARE(3.0) * baseIntervalSec;

		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N) =
				SQUARE(3.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E) =
				SQUARE(3.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS) =
				SQUARE(3.0) * baseIntervalSec;

		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) =
				SQUARE(3.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z) =
				SQUARE(3.0) * baseIntervalSec;

		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_N,currentStatus->STATUS_IND_WIND_SPEED_N) =
				SQUARE(0.1) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_WIND_SPEED_E,currentStatus->STATUS_IND_WIND_SPEED_E) =
				SQUARE(0.1) * baseIntervalSec;

	}

	// GPS Altitude
	if (allCapabilities & (1UL<<drivers::DriverBase::GPS_ALTITUDE_MSL)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL) =
				SQUARE(3.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED) =
				SQUARE(3.0) * baseIntervalSec;

	}

	// GPS heading (handle with care, that is derived from GPS positions in the GPS receiver internally too)
	if (allCapabilities & (1UL<<drivers::DriverBase::GPS_HEADING)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) =
				SQUARE(3.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z) =
				SQUARE(3.0) * baseIntervalSec;

	}

    // Static pressure i.e. barometric altimeter
	if (allCapabilities & (1UL<<drivers::DriverBase::STATIC_PRESSURE)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ALT_MSL,currentStatus->STATUS_IND_ALT_MSL) =
				SQUARE(3.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED) =
				SQUARE(3.0) * baseIntervalSec;

		if (allCapabilities & (1UL<<drivers::DriverBase::GPS_ALTITUDE_MSL)) {
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_QFF,currentStatus->STATUS_IND_QFF) =
					SQUARE(0.003) * baseIntervalSec; // about 10mB/h
		}

	}


	/* Ignore GPS speed because that is derived from GPS positions within the GPS receiver too, but not a direct measurement
	 * if (allCapabilities & (1UL<<drivers::DriverBase::GPS_SPEED)) {
	 *
	 * }
	 */

    // Dynamic pressure calculates indicated and true airspeed (the latter when I know my surrounding pressure, i.e. altitude)
	if (allCapabilities & (1UL<<drivers::DriverBase::DYNAMIC_PRESSURE)) {

		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS) =
				SQUARE(1.0) * baseIntervalSec;

		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING) =
				SQUARE(2.0) * baseIntervalSec;

		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK) =
				SQUARE(2.0) * baseIntervalSec;

		// If I have GPS positions I adjust the error increments
		// Otherwise it will default to 0 later.
		// Without absolute position corrections and determination initially
		// any position calculation from the speed is moot. Because also without
		// positions I cannot determine my course over ground which is equally important.
		if (allCapabilities & (1UL<<drivers::DriverBase::GPS_POSITION)) {
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LATITUDE_OFFS,currentStatus->STATUS_IND_LATITUDE_OFFS) =
					SQUARE(1.0) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_LONGITUDE_OFFS,currentStatus->STATUS_IND_LONGITUDE_OFFS) =
					SQUARE(1.0) * baseIntervalSec;

			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N) =
					SQUARE(2.0) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E) =
					SQUARE(2.0) * baseIntervalSec;

		}

	}

    // Accelerometer
	if (allCapabilities & (1UL<<drivers::DriverBase::ACCEL_3D)) {
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL) =
				SQUARE(1.0) * baseIntervalSec;

	    // With accelerometer I can calculate my attitude
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL) =
				SQUARE(3.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH) =
				SQUARE(3.0) * baseIntervalSec;

	    // With accelerometer I can calculate speeds with higher accuracy
		if (UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N)) {
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_N,currentStatus->STATUS_IND_SPEED_GROUND_N) =
					SQUARE(2.0) * baseIntervalSec;
// Take ACC_CROSS completely out of the equation. I have no adjusting counterpart to the accelerometer, and the accelerometer measurement itself
// is ambiguous between acceleration and gravitational pull due to roll angle.
// Since cross acceleration does not play a great role anyway I am accepting small errors here.
//			// Cross acceleration usually does not happen, but cross acceleration measurement
//			// is more likely caused by gravity and a banked aircraft (not turning)
//			// But I need to push the value back by absolute position.
//			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_CROSS,currentStatus->STATUS_IND_ACC_CROSS) =
//					SQUARE(0.5) * baseIntervalSec;
		}
		if (UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E)) {
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_SPEED_GROUND_E,currentStatus->STATUS_IND_SPEED_GROUND_E) =
					SQUARE(2.0) * baseIntervalSec;
		}
		if (UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS)) {
			// Only account for actual acceleration in contrast to attitude (tilt) when I can correct by speed measurements.
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_HEADING,currentStatus->STATUS_IND_ACC_HEADING) =
					SQUARE(2.0) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_TAS,currentStatus->STATUS_IND_TAS) =
					SQUARE(1.0) * baseIntervalSec;
		}
		if (UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED)) {
			// Only account for actual acceleration in contrast to attitude when I can correct by speed measurements.
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ACC_VERTICAL,currentStatus->STATUS_IND_ACC_VERTICAL) =
					SQUARE(1.0) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_VERTICAL_SPEED,currentStatus->STATUS_IND_VERTICAL_SPEED) =
					SQUARE(1.0) * baseIntervalSec;
		}
		if (UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK)) {
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_RATE_OF_SINK,currentStatus->STATUS_IND_RATE_OF_SINK) =
					SQUARE(1.0) * baseIntervalSec;
		}


	}

	// Gyroscopes
	if (allCapabilities & (1UL<<drivers::DriverBase::GYRO_3D)) {
		// Of course, first of all turn rates on all axes
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_X,currentStatus->STATUS_IND_ROTATION_X) =
				SQUARE(5.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Y,currentStatus->STATUS_IND_ROTATION_Y) =
				SQUARE(5.0) * baseIntervalSec;
		currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROTATION_Z,currentStatus->STATUS_IND_ROTATION_Z) =
				SQUARE(5.0) * baseIntervalSec;

		// ... and the biases when I have some second references for the angles
		if (
				UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) &&
				UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL) &&
				UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH)
				) {
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X) =
					SQUARE(0.0001) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y) =
					SQUARE(0.0001) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z) =
					SQUARE(0.0001) * baseIntervalSec;
		}

	}

    // Magnetometer is a only helper for others, but I am not changing dynamics here.
	// Only allow adjusting biases, and magnetic orientations when second measurements are available.
	if (allCapabilities & (1UL<<drivers::DriverBase::MAGNETOMETER_3D)) {

		if (
				UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_HEADING,currentStatus->STATUS_IND_HEADING) &&
				UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_ROLL,currentStatus->STATUS_IND_ROLL) &&
				UnInitVal != currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_PITCH,currentStatus->STATUS_IND_PITCH)
				) {
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_X,currentStatus->STATUS_IND_GYRO_BIAS_X) =
					SQUARE(0.0001) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Y,currentStatus->STATUS_IND_GYRO_BIAS_Y) =
					SQUARE(0.0001) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_GYRO_BIAS_Z,currentStatus->STATUS_IND_GYRO_BIAS_Z) =
					SQUARE(0.0001) * baseIntervalSec;

			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_MAGNETIC_DECLINATION,currentStatus->STATUS_IND_MAGNETIC_DECLINATION) =
					SQUARE(0.0001) * baseIntervalSec;
			currentStatus->getSystemNoiseCovariance_Q().coeffRef(currentStatus->STATUS_IND_MAGNETIC_INCLINATION,currentStatus->STATUS_IND_MAGNETIC_INCLINATION) =
					SQUARE(0.0001) * baseIntervalSec;
		}
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

	OEVClock::time_point nextLoopTime =  OEVClock::now();
	OEVClock::time_point now;

	while (idleLoopRunning) {
		GliderVarioMeasurementVector* measVector = 0;

		// just retrieve the current status, and immediately release it.
		// If the interval was exceeded the function will automatically calculate the next prediction.
		getCurrentStatusAndLock(measVector);
		releaseCurrentStatus();

		now = OEVClock::now();
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

