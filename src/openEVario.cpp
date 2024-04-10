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
#  include "config.h"
#endif

#include "GliderVarioMain.h"
#include <chrono>
#include <thread>

#include "gettext.h"

using namespace openEV;
using namespace std::chrono;
using namespace std::chrono_literals;

/**
 * \brief The one and only main() function
 * Startup and initialization. Demonization if required. Entry into the main processing loop.
 * @param argc
 * @param argv
 * @return Return code of the program. 0 means without error.
 */
int main (int argc, char *argv[]) {


#if defined ENABLE_NLS && ENABLE_NLS
	setlocale (LC_ALL, "");
	bindtextdomain ("OeV", "/usr/share/locale");
	textdomain ("OeV");
#endif // #if defined ENABLE_NLS && ENABLE_NLS

    GliderVarioMain oevMain (argc, (const char**)( argv));
    system_clock::duration waitTime = 10s;

    oevMain.startup();
    oevMain.runMainLoop();

    /// todo: Do something useful here. Otherwise the program ends right here :)
    while (1) {

    	std::this_thread::sleep_for(waitTime);

    }

    return 0;
}
