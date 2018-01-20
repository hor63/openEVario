/*
 *  openEVario.cpp
 *
 *  start module. Contains the main() function. Startup, initialization, and start of the main loop.
 *
 *  Created on: Dec 08, 2015
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
#  include <config.h>
#endif

#include <iostream>
#include <random>
#include <string>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#if HAVE_I2C_DEV_H == 1
    #include <linux/i2c-dev.h>
#else /* HAVE_I2C_DEV_H */

#   if HAVE_I2C_H == 1
#       include <linux/i2c.h>
#   endif
#endif /* HAVE_I2C_DEV_H */

#if HAVE_ARGP_H == 1
    #include <argp.h>
#endif

#if HAVE_GETOPT_H == 1
    #include <getopt.h>
#endif

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "kalman/GliderVarioStatus.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "util/RotationMatrix.h"
#include "util/FastMath.h"
#include "kalman/GliderVarioMeasurementVector.h"
#include "kalman/GliderVarioMeasurementUpdater.h"
#include "drivers/GliderVarioDriverBase.h"
#include "drivers/sensorDriver.h"

#if HAVE_LOG4CXX_H == 1
#	include "log4cxx/helpers/exception.h"
#endif

using namespace std;
using namespace openEV;

mt19937 randomGenerator;

FloatType x = 0;

#define defaultConfigFileName "./openEVario.cfg"
#define defaultLoggerConfigFileName "./openEVario.logger.properties"

static struct {
    string configFile       = defaultConfigFileName;
    string loggerConfigFile = defaultLoggerConfigFileName;
    /// logger level can be
    /// 0: Quiet. No output at all
    /// 1: Errors are reported
    /// 2: Info. Major events and activities
    /// 3: Debug. Be really chatty
    int defaultLoggerLevel = 1;
} programOptions;

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
     know is a pointer to our arguments structure. */

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


#endif /* HAVE_ARGP_PARSE == 1 */
;

    outStr << usageText << std::endl;

}

/**
 * Reads command line arguments and extracts options
 *
 * @param argc
 * @param argv
 * @return
 */
static int readOptions (int& argc, char*argv[]) {
    int rc = 0;

#if HAVE_ARGP_PARSE == 1

    static struct argp arrgp = {options, parse_opt, 0, argpDoc};

    rc = argp_parse (&arrgp, argc, argv, 0, 0, 0);

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
                     std::cerr << "Try `" << argv[0] << " --help' or `" << argv[0] << " --usage' for more information." << endl;
                 }
                 break;

             default:
                 if (argv[0]) {
                     std::cerr << argv[0] << ": getopt_long returned unexpected value" << key <<". Program aborting" << endl;
                 }
              exit(1);
             }


    } while (key != -1);

#endif /* HAVE_ARGP_PARSE == 1 */


    return rc;
}


/**
 * \brief The one and only main() function
 * Startup and initialization. Demonization if required. Entry into the main processing loop.
 * @param argc
 * @param argv
 * @return Return code of the program. 0 means without error.
 */
int main (int argc, char *argv[]) {
    int rc = 0;

    rc = readOptions (argc,argv);

#if defined HAVE_LOG4CXX_H

	log4cxx::BasicConfigurator::configure();

    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("openEV");

    // Silent-[Error]-Info-Debug
    switch (programOptions.defaultLoggerLevel) {

    	case 0: // Silent. Fatal errors causing immediate termination are still reported.
    	    logger->setLevel(log4cxx::Level::getFatal());
    	    break;

    	case 2:
    	    logger->setLevel(log4cxx::Level::getInfo());
    	    break;

		case 3:
			logger->setLevel(log4cxx::Level::getDebug());
			break;

		case 1:
		default:
			logger->setLevel(log4cxx::Level::getError());

    }


    // The configuration file (when I can load it) will overwrite the command line settings.
    log4cxx::PropertyConfigurator::configure(log4cxx::File(programOptions.loggerConfigFile));

#endif /* defined HAVE_LOG4CXX_H */

    LOG4CXX_INFO(logger," = " << programOptions.configFile);
    LOG4CXX_INFO(logger," = " << programOptions.defaultLoggerLevel);
    LOG4CXX_INFO(logger," = " << programOptions.loggerConfigFile);

    return rc;
}
/**
 * TODO remove all the test code, and replace it by real application code.
 */
static int doInternalTests () {
    double U1,U2,V1,V2,S,polarFactor,X,Y,dummy = 0.0	;
    mt19937::result_type min;
    double x,y, res,fastRes,maxDev = 0.0,maxDevAt = -100.0;
    double range;
    double sqrSum = 0.0,sum = 0.0,absSum =0.0;
    GliderVarioStatus ovStatus1, ovStatus2;
    GliderVarioStatus *ovStatusOld = &ovStatus1;
    GliderVarioStatus *ovStatusNew = &ovStatus2;

    GliderVarioTransitionMatrix ovTransition;
    GliderVarioMeasurementVector ovMeasurement;
    GliderVarioMeasurementVector::MeasureVectorType ovMeasureVector;
    GliderVarioMeasurementUpdater ovMeasureMatrix;
    int i;
    struct timeval tv1,tv2,tv3,tv4;

#if HAVE_I2C_DEV_H == 1 || HAVE_I2C_H == 1
    // test of MS4515DO differential pressure sensor
    // measurement range is -2.490889*20 to 2.490889*20 mb (measurement actually given as -20 to +20 in H2O)
    // This is the B model with a range from -90% to +90% digital range.
    const char* i2cBusName = "/dev/i2c-7";
    int i2cBus = open(i2cBusName,O_RDWR);
    uint8_t sensorReadBuf [2];
    uint16_t sensorAddr = 0x46;
    int rc = 0;

    struct i2c_msg msg = {
        sensorAddr, // u16 addr; /* slave address            */
        I2C_M_RD,     // unsigned short flags;
        sizeof (sensorReadBuf), // short len;      /* msg length               */
        (char*)sensorReadBuf // char *buf;      /* pointer to msg data          */
    };

    struct i2c_rdwr_ioctl_data rdwrData = {
        &msg, // struct i2c_msg *msgs;   /* pointers to i2c_msgs */
        1U,   // __u32 nmsgs;            /* number of i2c_msgs */
    };


    if (i2cBus == -1) {
        cerr << "Error while opening i2c bus \"" << i2cBusName << "\": " << strerror(errno) << endl;
        exit (1);
    }

    // first a little performance test:
    gettimeofday(&tv1,NULL);
    // read 1 word of data from the sensor 100 times
    for (i=0 ; i<100; i++) {
       rc = ioctl (i2cBus,I2C_RDWR,&rdwrData);
       if (rc == -1) {
           cerr << "Error reading from sensor: " << strerror(errno) << endl;
           exit (1);
       }
    }
    gettimeofday(&tv2,NULL);
    cout << " 100 I2C reads of 2 bytes took " << double(tv2.tv_sec-tv1.tv_sec)+double(tv2.tv_usec-tv1.tv_usec)/1000000 << endl;

    // read the sensor readings for ever
    while (1) {
        uint16_t flags;
        uint16_t sensor_reading;
        double diffPressure;
        struct timespec twoMS = {
                0,2000000
        };
        double constexpr sensorRange = 2.490889 * 20;
        double constexpr fact = 16383.0 * 0.9 / sensorRange / 2.0;
        // nanosleep (&twoMS,NULL);



        msg.addr  = sensorAddr;
        msg.buf   = (char*)sensorReadBuf;
        msg.flags = I2C_M_RD;
        msg.len   = 2;

        rdwrData.msgs  = &msg;
        rdwrData.nmsgs = 1;

        rc = ioctl (i2cBus,I2C_RDWR,&rdwrData);
        if (rc == -1) {
            cerr << "Error reading from sensor: " << strerror(errno) << endl;
            exit (1);
        }

        if (msg.len != 2) {
            cerr << "Length of data != 2. Length is " << msg.len << endl;
            exit(1);
        }

        flags = sensorReadBuf[0] >> 6;
        sensor_reading = (uint16_t(sensorReadBuf[0])&0b00111111) * 256 + uint16_t(sensorReadBuf[1]);
        diffPressure = (sensor_reading - 16383.0*0.05 - fact * sensorRange) / fact;

        cout <<
                "flags = " << flags <<
                ", sensorReading = " << sensor_reading <<
                ", diffPressure = " << diffPressure << endl;

        // when an error occurred or valid data were read
        // initiate the next conversion immediately
        // set the slave address

        if (flags != 2) {
            /*
            rc = ioctl(i2cBus, I2C_SLAVE, long (sensorAddr));
            if (rc == -1) {
                cerr << "Error setting sensor address: " << strerror(errno) << endl;
                exit (1);
            }
            rc = i2c_smbus_write_quick (i2cBus,I2C_SMBUS_READ);
            cout << "i2c_smbus_write_quick returned " << rc << endl;
            */
            // issue a 1-byte length read command to initiate a measurement.
            msg.len = 1;
            rc = ioctl (i2cBus,I2C_RDWR,&rdwrData);
            if (rc == -1) {
                cerr << "Error reading from sensor: " << strerror(errno) << endl;
                exit (1);
            }
        }

    }

    exit(0);
#endif /* HAVE_I2C_DEV_H == 1 || HAVE_I2C_H == 1 */

    // Test of fastMath.
    FloatType j;

    cout << "-----------------------" << endl;
    cout << "Test of fastMath." << endl;

    cout << "Start test sin" << endl;
    gettimeofday(&tv1,NULL);
    for (i=0;i<10000;i++) {
        for (j=0.0;j<=360;j+=1.0){
            dummy += sin(j);
        }
    }
    gettimeofday(&tv2,NULL);
    cout << 10000*360 << "End test sin" << endl;
    cout << " sin calls took " << double(tv2.tv_sec-tv1.tv_sec)+double(tv2.tv_usec-tv1.tv_usec)/1000000 << endl;
    gettimeofday(&tv3,NULL);
    for (i=0;i<10000;i++) {
        for (j=0.0;j<=360;j+=1.0){
            dummy += FastMath::fastSin(j);
        }
    }
    gettimeofday(&tv4,NULL);
    cout << "End test fastSin" << endl;
    cout << 10000*360 << " fastSin calls took " << double(tv4.tv_sec-tv3.tv_sec)+double(tv4.tv_usec-tv3.tv_usec)/1000000 << endl;
    cout << "Dummy = " << dummy << endl;
    cout << endl;

    cout << "-------------------" << endl
            << "Test of fast arc tangent " << endl;
    for (x = -1 ; x<=1 ; x+=0.2) {
        for (y = -1 ; y<=1 ; y+=0.2) {
            res = atan2(y,x) * FastMath::radToDeg;
            fastRes = FastMath::fastATan2(y,x);
            if (fastRes > 180.0) {
                fastRes = -360 + fastRes;
            }
            if (fabs(res-fastRes) > fabs(maxDev) && fabs(res-fastRes) < 359.0) {
                maxDev = res-fastRes;
            }
        }
        cout << "x = " << x << ", y = " << y << ", tanRes = " << res << ", fastRes = " << fastRes << endl;
    }
    cout << "maxDev = " << maxDev << endl;

    cout << "-------------------" << endl
            << "Test of fast arc sin " << endl;
    maxDev = 0.0;
    for (x = -100 ; x<=100 ; x++) {
        res = asin(x/100.0) * FastMath::radToDeg;
        fastRes = FastMath::fastASin(x/100.0);
        if (fabs(res-fastRes)>fabs(maxDev)) {
            maxDev = res-fastRes;
            maxDevAt = x/100;
        }
    }
    cout << "maxDev = " << maxDev << " at " << maxDevAt << endl;

    cout << "-------------------" << endl
            << "Test of fast arc cos " << endl;
    maxDev = 0.0;
    for (x = -100 ; x<=100 ; x++) {
        res = acos(x/100.0) * FastMath::radToDeg;
        fastRes = FastMath::fastACos(x/100.0);
        if (fabs(res-fastRes)>fabs(maxDev)) {
            maxDev = res-fastRes;
            maxDevAt = x/100;
        }
    }
    cout << "maxDev = " << maxDev << " at " << maxDevAt << endl;




    cout << "-----------------------" << endl;
    cout << "Test of rotation matrix" << endl;
    RotationMatrix rotMat (30.0f,0.0f,-20.0f);
    Vector3DType worldRot (0.0f,0.0f,-18.0f);
    Vector3DType planeRot;
    rotMat.calcWorldVectorToPlaneVector(worldRot,planeRot);
    cout << "-----------" << endl;
    cout << "Yaw, pitch roll = " << rotMat.getYaw() << ", " << rotMat.getPitch() << ", " << rotMat.getRoll() << endl;
    cout << "-----------" << endl;
    cout << "rotMatWorldToPlane = " << endl;
    cout << rotMat.getMatrixGloToPlane() << endl;
    cout << "-----------" << endl;
    cout << "worldRot = " << endl;
    cout << worldRot << endl;
    cout << "-----------" << endl;
    cout << "planeRot = " << endl;
    cout << planeRot << endl;

    cout << endl <<
            "-----------------------" << endl;
    cout << "Test conversion of 3d vectors into rotation matrixes, and into rotation vectors" << endl;
    cout << "Crucial to convert magnetic vector measurements into rotation vectors for attitude and direction" << endl;

    RotationMatrix rotMag (-5,-70,0); // magnetic field 5 deg West, 70deg inclination (realistic in N-Germany)
    Vector3DType uniVector (1,0,0), magField (0,0,0),magField1(0,0,0);
    rotMag.calcPlaneVectorToWorldVector(uniVector,magField);
    cout << "Magnetic field vector at 5Deg W, 70Deg incl. = " << endl << magField << endl;
    cout << "length of magnetic field vector is " << sqrtf(magField(0)*magField(0) + magField(1)*magField(1) + magField(2)*magField(2)) << endl;
    cout << "The rotation matrix of the vector is " << endl << rotMag.getMatrixGloToPlane() << endl;

    rotMag.setYaw(-2.0f); // magnetic field 2 deg West variation, 70deg inclination downward (realistic in N-Germany)
    rotMag.setPitch(-70.0f);
    rotMag.setRoll(0.0f);
    rotMag.calcPlaneVectorToWorldVector(uniVector,magField);
    cout << "Unit vector is " << endl << uniVector << endl;
    cout << "Magnetic field vector at 2Deg W, 70Deg incl. = " << endl << magField << endl;
    cout << "length of magnetic field vector is " << sqrtf(magField(0)*magField(0) + magField(1)*magField(1) + magField(2)*magField(2)) << endl;
    cout << "The rotation matrix of the vector is " << endl << rotMag.getMatrixGloToPlane() << endl;
    // Now add -3 deg deviation
    rotMat.setYaw(-3.0f);
    rotMat.setPitch(0.0f);
    rotMat.setRoll(0.0f);
    magField1 = (rotMat.getMatrixPlaneToGlo() * rotMag.getMatrixPlaneToGlo()) * uniVector;
    cout << "Magnetic field vector with 3Deg W Deviation added = " << endl << magField1 << endl;
    cout << "length of magnetic field vector is " << sqrtf(magField1(0)*magField1(0) + magField1(1)*magField1(1) + magField1(2)*magField1(2)) << endl;
    cout << "The rotation matrix of the deviation is " << endl << rotMat.getMatrixGloToPlane() << endl;

    rotMat.calcPlaneVectorToWorldVector(magField,magField1);
    cout << "Magnetic field vector with 3Deg W Deviation added = " << endl << magField1 << endl;

    cout << endl <<
            "-----------------------" << endl;
    cout << "test the transition matrix" << endl;



    // Initialize the status vector
    ovStatusOld->longitude = 55.0f * 3600.0f;
    ovStatusOld->latitude = 10.0f * 3600.0f;
    ovStatusOld->altMSL = 500.0f;
    // ovStatusOld->yawAngle = 30.0f;
    ovStatusOld->pitchAngle = 0.0f;
    ovStatusOld->rollAngle = 20.0f;

    // Speeds and directions
    ovStatusOld->groundSpeedNorth = 15.0f;
    ovStatusOld->groundSpeedEast = 25.0f;
    ovStatusOld->trueAirSpeed = 30.0f;
    ovStatusOld->heading = 30.0f;
    ovStatusOld->rateOfSink = 0.5f;
    ovStatusOld->verticalSpeed = -2.0;

    // Accelerations in reference to the body coordinate system. Accelerations are on the axis of the *plane*.
    // If the plane is pitched up an acceleration on the X axis would speed the plane upward, not forward.
    ovStatusOld->accelHeading = 0.0f;
    ovStatusOld->accelCross = 0.0f;
    ovStatusOld->accelVertical = 0.0;

    // Turn rates in reference to the body coordinate system
    ovStatusOld->rollRateX = 0.0f;
    ovStatusOld->pitchRateY = 0.0f;
    {   // intermediate calculation to estimate the turn radius, and the resulting turn rate.
        FloatType angularAccel = 9.81 * FastMath::fastSin(ovStatusOld->rollAngle)/FastMath::fastCos(ovStatusOld->rollAngle);
        // circular radius and and acceleration are defined as:
        // a = v^2 / r, i.e. r = v^2 / a
        FloatType radius = ovStatusOld->trueAirSpeed*ovStatusOld->trueAirSpeed/angularAccel;
        // the circumfence of the circle is 2PI*r, with the speed the time for a full 360 deg circle is therefore
        // t = 2PIr/TAS
        FloatType timeFullCircle = 2.0*M_PI*radius/ovStatusOld->trueAirSpeed;
        RotationMatrix rotMat1 (ovStatusOld->heading,ovStatusOld->pitchAngle,ovStatusOld->rollAngle);

        ovStatusOld->yawRateZ = 360.0/timeFullCircle;

        cout << " -- yaw rate calculation ------------" << endl;
        cout << " angularAccel      = " << angularAccel << endl;
        cout << " radius            = " << radius << endl;
        cout << " timeFullCircle    = " << timeFullCircle << endl;
        cout << " ovStatusOld->yawRateZ = " << ovStatusOld->yawRateZ << endl;
        cout << " World rotation rate vector = " << endl << worldRot << endl;
        cout << " Plane rotation rate vector = " << endl << planeRot << endl;

    }

    // Derived values which improve the responsiveness of the Kalman filter. Some are also the true goals of the filter
    ovStatusOld->gyroBiasX = 0.0f;
    ovStatusOld->gyroBiasY = 0.0f;
    ovStatusOld->gyroBiasZ = 0.0f;
    ovStatusOld->windSpeedNorth = 0.0f;
    ovStatusOld->windSpeedEast = 0.0f;
    ovStatusOld->thermalSpeed = 0.0f;

    cout << "-- ovStatus T = " << endl << *ovStatusOld;


    // initialize the error covariance and the system noise covariance
    for ( i=0 ; i < GliderVarioStatus::STATUS_NUM_ROWS ; i++) {
        ovStatus1.getErrorCovariance_P().insert(i,i) = 1.0f;
        ovStatus2.getErrorCovariance_P().insert(i,i) = 1.0f;
        ovStatus1.getSystemNoiseCovariance_Q().insert(i,i) = 0.1f;
        ovStatus2.getSystemNoiseCovariance_Q().insert(i,i) = 0.1f;
    }

    // Gravity has a bit of variance in the error covariance, but none in the system noise. So it should not increase.
    ovStatus1.getSystemNoiseCovariance_Q().coeffRef(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 0.0f;
    ovStatus1.getErrorCovariance_P().coeffRef(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 0.01f;
    ovStatus2.getSystemNoiseCovariance_Q().coeffRef(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 0.0f;
    ovStatus2.getErrorCovariance_P().coeffRef(GliderVarioStatus::STATUS_IND_GRAVITY,GliderVarioStatus::STATUS_IND_GRAVITY) = 0.01f;

    ovTransition.updateStatus(*ovStatusOld,*ovStatusNew,0.1f);
    cout << "-- ovTransition = " << endl << ovTransition.getTransitionMatrix() << endl;
    cout << "-- ovStatus after 0.1 sec = " << endl << *ovStatusNew << endl;
    cout << "-- Error Covariance = " << endl << ovStatusNew->getErrorCovariance_P() << endl;

    GliderVarioStatus *tmp = ovStatusOld;
    ovStatusOld=ovStatusNew;
    ovStatusNew = tmp;

    for (i = 0; i< 20; i++) {
        ovTransition.updateStatus(*ovStatusOld,*ovStatusNew,0.01f);
        cout << *ovStatusNew;

        tmp = ovStatusOld;
        ovStatusOld=ovStatusNew;
        ovStatusNew = tmp;
    }

    cout << "-- Error Covariance after 21 iterations = " << endl << ovStatusNew->getErrorCovariance_P() << endl;

    gettimeofday(&tv1,NULL);
    for (i = 0; i< 10000; i++) {
        ovTransition.updateStatus(*ovStatusOld,*ovStatusNew,0.01f);

        tmp = ovStatusOld;
        ovStatusOld=ovStatusNew;
        ovStatusNew = tmp;
    }
    gettimeofday(&tv2,NULL);
    cout << 10000 << " timesovTransition.updateStatus";
    cout << " calls took " << double(tv2.tv_sec-tv1.tv_sec)+double(tv2.tv_usec-tv1.tv_usec)/1000000 << endl;

    cout << "-- Error Covariance after 10021 iterations = " << endl << ovStatusNew->getErrorCovariance_P() << endl;

    {
        GliderVarioMeasurementVector measurementVector;

        cout << endl <<
                "-----------------------" << endl;
        cout << "test measurement updates" << endl;


        GliderVarioMeasurementUpdater::GPSLongitudeUpd(55.0f,9.0e-05f,measurementVector,*ovStatusNew);
        GliderVarioMeasurementUpdater::GPSLatitudeUpd(55.0f,2.0e-04f,measurementVector,*ovStatusNew);
        GliderVarioMeasurementUpdater::GPSAltitudeUpd(500.0f,15.0f,measurementVector,*ovStatusNew);


    }

    cout << endl <<
            "-----------------------" << endl;
    cout << "test normalizing angles" << endl;
    cout << "normalizing yaw" << endl;
    for (i = -720; i <= 720; i+=360) {
        ovStatusOld->pitchAngle = 5;
        ovStatusOld->rollAngle = -20;
        ovStatusOld->heading = static_cast<FloatType>(i) - 1.0f;
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

        ovStatusOld->pitchAngle = 5;
        ovStatusOld->rollAngle = -20;
        ovStatusOld->heading = static_cast<FloatType>(i);
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

        ovStatusOld->pitchAngle = 5;
        ovStatusOld->rollAngle = -20;
        ovStatusOld->heading = static_cast<FloatType>(i) + 1.0f;
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;
    }

    cout << "normalizing roll" << endl;
    for (i = -360-180; i <= 360+180; i+=180) {
        ovStatusOld->pitchAngle = 5;
        ovStatusOld->heading = 30;
        ovStatusOld->rollAngle = static_cast<FloatType>(i) - 1.0f;
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

        ovStatusOld->pitchAngle = 5;
        ovStatusOld->heading = 30;
        ovStatusOld->rollAngle = static_cast<FloatType>(i);
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

        ovStatusOld->pitchAngle = 5;
        ovStatusOld->heading = 30;
        ovStatusOld->rollAngle = static_cast<FloatType>(i) + 1.0f;
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;
    }

    cout << "normalizing Pitch" << endl;
    for (i = -360-180; i <= 360+180; i+=90) {
        ovStatusOld->rollAngle = 20;
        ovStatusOld->heading = 30;
        ovStatusOld->pitchAngle = static_cast<FloatType>(i) - 1.0f;
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

        ovStatusOld->rollAngle = 20;
        ovStatusOld->heading = 30;
        ovStatusOld->pitchAngle = static_cast<FloatType>(i);
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;

        ovStatusOld->rollAngle = 20;
        ovStatusOld->heading = 30;
        ovStatusOld->pitchAngle = static_cast<FloatType>(i) + 1.0f;
        cout << "Before: Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl;
        ovStatusOld->normalizeAngles();
        cout << "After:  Yaw,Pitch,Roll = " << ovStatusOld->heading << ", " << ovStatusOld->pitchAngle << ", " << ovStatusOld->rollAngle << endl << endl;
    }


    cout << endl <<
            "-----------------------" << endl;
    cout << "test the barometric formula" << endl;

    /*
   Equation 1:

    P = P b ⋅ [ T b T b + L b ⋅ ( h − h b ) ] g 0 ⋅ M R ∗ ⋅ L b {\displaystyle {P}=P_{b}\cdot \left[{\frac {T_{b}}{T_{b}+L_{b}\cdot (h-h_{b})}}\right]^{\textstyle {\frac {g_{0}\cdot M}{R^{*}\cdot L_{b}}}}} {P}=P_{b}\cdot \left[{\frac {T_{b}}{T_{b}+L_{b}\cdot (h-h_{b})}}\right]^{{\textstyle {\frac {g_{0}\cdot M}{R^{*}\cdot L_{b}}}}}


where

    P b {\displaystyle P_{b}} P_{b} = static pressure (Pa)
    T b {\displaystyle T_{b}} T_{b} = standard temperature (K)
    L b {\displaystyle L_{b}} L_{b} = standard temperature lapse rate (K/m) in ISA
    h {\displaystyle h} h = height above sea level (m)
    h b {\displaystyle h_{b}} h_b = height at bottom of layer b (meters; e.g., h1 = 11 000 m)
    R ∗ {\displaystyle R^{*}} R^{*} = universal gas constant: 8.3144598  J /mol/K
    g 0 {\displaystyle g_{0}} g_{0} = gravitational acceleration: 9.80665 m/s2
    M {\displaystyle M} M = molar mass of Earth's air: 0.0289644 kg/mol

     */
    FloatType qff = 1013.250;          // 1013.25 mb
    FloatType temp = 273.15 + 15.0;   // 15C
    FloatType tempLapse = -0.01;      // -1C/100m
    FloatType height    = 1000;       // m
    FloatType R         = 8.3144598;   // 8.3144598  J /mol/K
    FloatType Rspec     = 287.058;     // Specific R for dry air
    FloatType M         = 0.0289644; // 0.0289644 kg/mol
    FloatType ex = GRAVITY * M / R / tempLapse;
    FloatType p;
    FloatType density;
    FloatType speed;

    height = 1000.0f;
    p = qff * powf ((temp / (temp + tempLapse * height)),ex);
    cout << "Pressure at " << height << "m is " << p << "mb." << endl;

    height = 2000.0f;
    p = qff * powf ((temp / (temp + tempLapse * height)),ex);
    cout << "Pressure at " << height << "m is " << p << "mb." << endl;

    height = 3000.0f;
    p = qff * powf ((temp / (temp + tempLapse * height)),ex);
    cout << "Pressure at " << height << "m is " << p << "mb." << endl;

    height = 5500.0f;
    p = qff * powf ((temp / (temp + tempLapse * height)),ex);
    cout << "Pressure at " << height << "m is " << p << "mb." << endl;


    cout << endl <<
            "-----------------------" << endl;
    cout << "test the dynamic pressure formula" << endl;

    cout << endl << "Pressure = 1013.25 mb, Temp = 15C" << endl;
    density = 101325 / Rspec / temp;
    cout << "Density in standard air is" << density << endl;

    speed = 60.0f / 3.6f;
    p = 0.5f * density * speed * speed;
    cout << "Dynamic pressure at " << speed*3.6f << "km/h is " << p/100.0f << "mb." << endl;

    speed = 80.0f / 3.6f;
    p = 0.5f * density * speed * speed;
    cout << "Dynamic pressure at " << speed*3.6f << "km/h is " << p/100.0f << "mb." << endl;

    speed = 100.0f / 3.6f;
    p = 0.5f * density * speed * speed;
    cout << "Dynamic pressure at " << speed*3.6f << "km/h is " << p/100.0f << "mb." << endl;

    speed = 120.0f / 3.6f;
    p = 0.5f * density * speed * speed;
    cout << "Dynamic pressure at " << speed*3.6f << "km/h is " << p/100.0f << "mb." << endl;

    speed = 150.0f / 3.6f;
    p = 0.5f * density * speed * speed;
    cout << "Dynamic pressure at " << speed*3.6f << "km/h is " << p/100.0f << "mb." << endl;

    speed = 200.0f / 3.6f;
    p = 0.5f * density * speed * speed;
    cout << "Dynamic pressure at " << speed*3.6f << "km/h is " << p/100.0f << "mb." << endl;

    speed = 250.0f / 3.6f;
    p = 0.5f * density * speed * speed;
    cout << "Dynamic pressure at " << speed*3.6f << "km/h is " << p/100.0f << "mb." << endl;

    speed = 300.0f / 3.6f;
    p = 0.5f * density * speed * speed;
    cout << "Dynamic pressure at " << speed*3.6f << "km/h is " << p/100.0f << "mb." << endl;

    cout << endl <<
            "-----------------------" << endl;
    cout << "test the random generator" << endl;

    min = randomGenerator.min();
    range = double(randomGenerator.max()-randomGenerator.min());

    cout << "Hello World" << endl; /* prints Hello World */

    cout << " min of randomGenerator = " << min << endl;
    cout << " range of randomGenerator = " << range << endl;

    for (i=0; i<10000000;i++) {
        do {
            U1 = double((randomGenerator()) - min) / range;
            U2 = double((randomGenerator()) - min) / range;
            V1=2.0 * U1 -1.0;            /* V1=[-1,1] */
            V2=2.0 * U2 - 1.0;           /* V2=[-1,1] */
            S=V1 * V1 + V2 * V2;
        }  while (S >=1.0);
        polarFactor = sqrt(-2.0 * log(S) / S);
        X = polarFactor * V1;
        Y = polarFactor * V2;
        sum += X + Y;
        sqrSum += X*X + Y*Y;
        absSum += fabs(X) + fabs(Y);

    }


    cout << "sum = " << sum << " Mean of " << i*2 << " samples is " << sum / double(i*2)  << endl;
    cout << "Average deviation of " << i*2 << " samples is " << absSum / double(i*2)  << endl;
    cout << "the variance of " << i*2 << " samples is " << sqrSum / double(i*2) << endl;
    cout << "the standard deviation is " << sqrt (sqrSum / double(i*2)) << endl;
    cout << "the sample variance of " << i*2 << " samples is " << sqrSum / double(i*2-1) << endl;
    cout << "the sample standard deviation is " << sqrt (sqrSum / double(i*2-1)) << endl;

    cout << "Size of status vector = " << static_cast<int>(openEV::GliderVarioStatus::STATUS_NUM_ROWS)<<endl;

    return 0;
}
