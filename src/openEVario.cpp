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


#if HAVE_LOG4CXX_H == 1
#	include "log4cxx/helpers/exception.h"
#	include "log4cxx/logger.h"
#	include "log4cxx/basicconfigurator.h"
#endif

namespace openEV {

}


using namespace std;
using namespace openEV;





/**
 * \brief The one and only main() function
 * Startup and initialization. Demonization if required. Entry into the main processing loop.
 * @param argc
 * @param argv
 * @return Return code of the program. 0 means without error.
 */
int main (int argc, char *argv[]) {
    int rc = 0;

#if defined HAVE_LOG4CXX_H

	log4cxx::BasicConfigurator::configure();

    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("openEV");

#endif /* defined HAVE_LOG4CXX_H */

    return rc;
}
