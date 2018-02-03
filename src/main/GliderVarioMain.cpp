/*
 * GliderVarioMain.cpp
 *
 *  Created on: Jan 28, 2018
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

#include "GliderVarioMain.h"


#include <iostream>
#include <string>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

// One options library
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

#if HAVE_CONFUSE_H == 1
#	include <confuse.h>
#endif

#if defined HAVE_LOG4CXX_H
log4cxx::LoggerPtr logger = 0;
#endif

#define defaultConfigFileName "./openEVario.properties"
#define defaultLoggerConfigFileName "./openEVario.logger.properties"

static struct {
    std::string configFile       = defaultConfigFileName;
    std::string loggerConfigFile = defaultLoggerConfigFileName;
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
                     std::cerr << "Try `" << argv[0] << " --help' or `" << argv[0] << " --usage' for more information." << std::endl;
                 }
                 break;

             default:
                 if (argv[0]) {
                     std::cerr << argv[0] << ": getopt_long returned unexpected value" << key <<". Program aborting" << std::endl;
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
namespace openEV {



GliderVarioMain::GliderVarioMain(int argc, const char *argv[]) {

	int i;

	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Main.GliderVarioMain");
	}

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

GliderVarioMain::~GliderVarioMain() {

	int i;

	if (argc > 0) {
		for (i=0;i < argc; i++) {
			delete argv[i];
		}

		free (argv);
	}


}

void GliderVarioMain::startup () {
    int rc = 0;

    rc = readOptions (argc,argv);

#if defined HAVE_LOG4CXX_H

	log4cxx::BasicConfigurator::configure();

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


}

} /* namespace openEV */
