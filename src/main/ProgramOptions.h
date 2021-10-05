/*
 * ProgramOptions.h
 *
 *  Created on: Aug 23, 2018
 *      Author: hor
 *
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

#ifndef MAIN_PROGRAMOPTIONS_H_
#define MAIN_PROGRAMOPTIONS_H_

#include <string> 
#include <chrono>
 

#define defaultConfigFileName "./openEVario.properties"
#define defaultLoggerConfigFileName "./openEVario.logger.properties"

namespace openEV {

	struct ProgramOptions{
	    std::string configFile       = defaultConfigFileName;
	    std::string loggerConfigFile = defaultLoggerConfigFileName;
	    /**
	     * logger level can be
	     * 0: Quiet. No output at all
	     * 1: Errors are reported
	     * 2: Info. Major events and activities
	     * 3: Debug. Be really chatty
	     */
	    int defaultLoggerLevel = 2; // Default level info.
	    bool terminateOnDriverLoadError = true;
	    /// The idle loop cycle, i.e. the base cycle of the Kalman filter as system clock duration
	    OEVDuration idlePredictionCycle = std::chrono::milliseconds(20);
	    /// Idle loop cycle as double in milliseconds
	    /// \see \ref idlePredictionCycle
	    double idlePredictionCycleMilliSec = 20.0;
	    OEVDuration maxTimeBetweenPredictionAndMeasurementUpdate = std::chrono::milliseconds(15);
	    /** \brief Run the idle thread in the main program to perform Kalman prediction cycles at least every \ref idlePredictionCycle.
	     *
	     * This will be set false only when a sensor driver has the capability \ref drivers::GliderVarioDriverBase::RUN_IDLE_LOOP. In this case the idle loop is executed
	     * by the driver within for debugging and testing purposes.
	     */
	    bool runIdleLoop = true;
	} ;
 
} // namespace openEV

#endif /* MAIN_PROGRAMOPTIONS_H_ */
