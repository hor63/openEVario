/*
 * GliderVarioMainPriv.h
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

#ifndef MAIN_GLIDERVARIOMAINPRIV_H_
#define MAIN_GLIDERVARIOMAINPRIV_H_

#include "OEVCommon.h"
#include "util/GliderVarioExceptionBase.h"

#include "Properties4CXX/Properties.h"

#define defaultConfigFileName "./openEVario.properties"
#define defaultLoggerConfigFileName "./openEVario.logger.properties"


namespace openEV {

/** \brief Internal main class of the openEVario which implements the functions and holds the variables of openEVario
 *
 *  \ref GliderVarioMain is a shell class which implements the interface to the outside world but does not expose any internal information to
 *  the outside world. All that is in this class here.
 *
 */
class GliderVarioMainPriv {
public:

	struct ProgramOptions{
	    std::string configFile       = defaultConfigFileName;
	    std::string loggerConfigFile = defaultLoggerConfigFileName;
	    /// logger level can be
	    /// 0: Quiet. No output at all
	    /// 1: Errors are reported
	    /// 2: Info. Major events and activities
	    /// 3: Debug. Be really chatty
	    int defaultLoggerLevel = 2; // Default level info.
	} ;


	/** \brief Constructor accepting command line options compatible with main()
	 *
	 * Constructor accepting command line options compatible with main().
	 * The command line options can be constructed by the caller. They do not have to be the original
	 * argc und argv parameters of the main() function
	 *
	 * @param argc Number of command line arguments in argv
	 * @param argv Array of strings with the command line options. The arguments list is copied into the object.
	 *   The caller may free the list between this constructor and calling \ref startup().
	 *   Changes of the caller's arguments list between this constructor and calling \ref startup() will have therefore no effect.
	 */
	GliderVarioMainPriv(int argc, const char *argv[]);

	/** \brief Destructor. Shutdown the program. Release resources.
	 *
	 * Shutdown everything:
	 * - Shutdown and terminate the main loop
	 * - Close all connections.
	 * - Release and unload the drivers.
	 * - Release all allocated memory
	 *
	 */
	virtual ~GliderVarioMainPriv();

	/** \brief Initialize and startup the program
	 *
	 * Initialize the program, i.e. primarily read options, and load the drivers
	 *
	 */
	void startup ();

	/** \brief Run the program
	 *
	 * Run the main loop of the program. Run the predictions, read sensor values, and apply them to the model.
	 *
	 */
	void runMainLoop ();

	/** \brief Suspend the program
	 *
	 * Suspend the main loop of the program temporarily. Can be resumed with resumeMainLoop().
	 *
	 */
	void suspendMainLoop ();

	/** \brief Resume the program
	 *
	 * Resume the main loop after it was suspended by suspendMainLoop().
	 *
	 */
	void resumeMainLoop ();

	ProgramOptions const &getProgramOptions() const {
		return programOptions;
	}

private:

	int argc;
	char **argv;

	ProgramOptions programOptions;

	Properties4CXX::Properties configuration;



	/** \brief Open the driver libraries and initialize them
	 *
	 * Tries to open the drivers as shared libraries.
	 *
	 * The list of driver libs is in the configuration variable "driverSharedLibs".
	 *
	 */
	void readDriverLibs();

};

} /* namespace openEV */

#endif /* MAIN_GLIDERVARIOMAINPRIV_H_ */
