/*
 * GliderVarioMain.h
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

#ifndef MAIN_GLIDERVARIOMAIN_H_
#define MAIN_GLIDERVARIOMAIN_H_

#include "OEVCommon.h"
#include "util/GliderVarioExceptionBase.h"

namespace openEV {

/** \brief Main class of the Open Electronic Vario (openEV)
 *
 * This class encapsulates the main program to make it as easy as possible to link openEV into another program.
 *
 */
class OEV_MAIN_PUBLIC GliderVarioMain {
public:

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
	GliderVarioMain(int argc, const char *argv[]);

	/** \brief Destructor. Shutdown the program. Release resources.
	 *
	 * Shutdown everything:
	 * - Shutdown and terminate the main loop
	 * - Close all connections.
	 * - Release and unload the drivers.
	 * - Release all allocated memory
	 *
	 */
	virtual ~GliderVarioMain();

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

private:

	int argc;
	char **argv;

};

} /* namespace openEV */

#endif /* MAIN_GLIDERVARIOMAIN_H_ */
