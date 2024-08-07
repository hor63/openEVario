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

#include <list>
#include <string>
#include <mutex>
#include <thread>

#include "CommonDefs.h"
#include "Properties4CXX/Properties.h"

#include "main/ProgramOptions.h"
#include "util/GliderVarioExceptionBase.h"
#include "main/GliderVarioDriverList.h"
#include "kalman/GliderVarioStatus.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementVector.h"


namespace openEV {

/** \brief Internal main class of the openEVario which implements the functions and holds the variables of openEVario
 *
 *  \ref GliderVarioMain is a shell class which implements the interface to the outside world but does not expose any internal information to
 *  the outside world. All that is in this class here.
 *
 */
class GliderVarioMainPriv {

public:

	/** \brief Convenience class which implements a synchronized access to GliderVarioMainPriv::CurrentStatus
	 *
	 * Use instead of GliderVarioMainPriv::getCurrentStatusAndLock() and GliderVarioMainPriv::releaseCurrentStatus() whenever possible
	 * The current status will be locked as long as the object exists but automatically released when the object is deleted.
	 * Therefore it is used best directly declared in a code block with limited scope.
	 */
	class LockedCurrentStatus {
	public:

		LockedCurrentStatus (GliderVarioMainPriv & gliderVario)
		: gliderVario {gliderVario}
		{

			currentStatus = gliderVario.getCurrentStatusAndLock(measurementVector);

		}

		~LockedCurrentStatus() {
			gliderVario.releaseCurrentStatus();
		}

		GliderVarioStatus *getCurrentStatus () {
			return currentStatus;
		}

		GliderVarioStatus * operator ->() {
			return currentStatus;
		}

		GliderVarioMeasurementVector *getMeasurementVector () {
			return measurementVector;
		}

	private:

		GliderVarioMainPriv & gliderVario;
		GliderVarioStatus *currentStatus;
		GliderVarioMeasurementVector *measurementVector;

	};

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
	 * Start measurement capturing by all sensor drivers.
	 *
	 * Start the idle thread which forces a prediction cycle every \ref ProgramOptions::idlePredictionCycle milliseconds unless measurement updates are updating the Kalman status faster.
	 *
	 * This function returns immediately because the driver loops and the idle loop run as independent threads.
	 *
	 */
	void startMainLoop ();

	/** \brief Stop the program
	 *
	 * Stops measurement capturing by all sensor drivers.
	 *
	 * Stops the idle thread.
	 *
	 * This function may return with delay because it actively waits until all drivers have actually stopped.
	 *
	 */
	void stopMainLoop ();

	/** \brief Suspend the program
	 *
	 * Suspend the main loop and measurement capturing of the program temporarily. Can be resumed with \ref resumeMainLoop().
	 *
	 */
	void suspendMainLoop ();

	/** \brief Resume the program
	 *
	 * Resume the main loop and measurement capturing after it was suspended by \ref suspendMainLoop().
	 *
	 */
	void resumeMainLoop ();

	ProgramOptions const &getProgramOptions() const {
		return programOptions;
	}

	/** \brief Return the pointer to the current status and lock it against concurrent access
	 *
	 * The function return a pointer to the current status, and requests a mutex to prevent concurrent access to the current status.
	 *
	 * The intended use of this function is the measurement update from sensors. If ProgramOptions::maxTimeBetweenPredictionAndMeasurementUpdate
	 * is exceeded a prediction cycle is run and the status instances are swapped.
	 *
	 * After using the status it *must* be released ASAP by *the same* thread with \ref releaseCurrentStatus()
	 *
	 * @param[out] measurementVector Returns a pointer to the mostly useless :) measurement vector of the Kalman filter.
	 * @return Pointer to the current status
	 */
	GliderVarioStatus * getCurrentStatusAndLock(GliderVarioMeasurementVector* & measurementVector) OEV_MAIN_PUBLIC;

	/** \brief Release the mutex of the current status obtained from calling \ref getCurrentStatusAndLock
	 *
	 */
	void releaseCurrentStatus () OEV_MAIN_PUBLIC ;

	/** \brief Access function to obtain all Kalman filter components for a driver running the complete main loop single threaded for debugging and testing purposes only.
	 *
	 * Do not forget to call \ref releaseCurrentStatus() .
	 *
	 * @param currentStatus Reference to the address of the pointer of the current status.
	 * @param nextStatus Reference to the address of the pointer of the current status.
	 * @param transitionMatrix Reference to the pointer to the transition matrix
	 * @param measurementVector Refrence to the pointer to the (mostly useless :) measurement vector
	 */
	void getAndLockInternalStatusForDebug (
			GliderVarioStatus **&currentStatus,
			GliderVarioStatus **&nextStatus,
			GliderVarioTransitionMatrix *&transitionMatrix,
			GliderVarioMeasurementVector *&measurementVector
			) OEV_MAIN_PUBLIC ;

	OEVClock::time_point getLastPredictionUpdate() {
		return lastPredictionUpdate;
	}

private:

	int argc;
	char **argv;

	ProgramOptions programOptions;

	Properties4CXX::Properties configuration;

	GliderVarioDriverList driverList;

	/// The alternating status variables. These are never used directly but the pointers \ref currentStatus and \ref nextStatus which are exchanged after each
	/// prediction step.
	GliderVarioStatus stat1,stat2;

	/// The transition matrix to predict the next status and the co-variance
	GliderVarioTransitionMatrix transitionMatrix;

	/// Needed for the Kalman filter interface, not much use otherwise at the moment.
	GliderVarioMeasurementVector measurementVector;

	GliderVarioStatus *currentStatus = &stat1;
	GliderVarioStatus *nextStatus = &stat2;

	/// \brief synchronizes access and updates to \ref stat1 and \ref stat2
	std::mutex currentStatusLock;

	OEVClock::time_point lastPredictionUpdate;

    /// \brief The idle loop thread
    std::thread idleLoopThread;

    bool idleLoopRunning = false;

    /** \brief Register the communication drivers
     *
     * Register the communication drivers during startup.
     */
    void registerPortDrivers();

	/// \brief Read the configuration file, and extract the base configuration values into \ref programOptions
	void readConfiguration ();

	/** \brief Performs a prediction from the current to the next status and swaps statuses.
	 *
	 * This function must only be called under protection of \ref currentStatusLock
	 */
	void predictAndSwapStatus();

	/** \brief Let all drivers initialize the status from initial readings.
	 *
	 * After letting all drivers initialize the status initialize the un-initialized components with default values.
	 */
	void intializeStatus ();

	/** \brief Set the Kalman system error increments based on the sensor capabilities
	 *
	 * The system error increment determine the characteristics of the filter during runtime.
	 * The error increments determine the equilibrium status of the system error together with
	 * the decrease of system errors by measurement updates.
	 *
	 * Depending on available sensors the emphasis differs. For instance a derived values will get a higher
	 * system error than the integrated value. For example when I only have the GPS position I will update
	 * position and speed roughly equally. When I have the TAS by difference pressure available I will increment the
	 * speed system error much faster than the position error.
	 */
	void setKalmanErrorIncrements ();

	/*** \brief The idle loop will force an prediction cycle of the Kalman status when no measurement updates did it in the mean time.
	 *
	 *
	 */
	void idleLoop ();

	static void idleLoopTreadEntry (GliderVarioMainPriv *vario);


};

} /* namespace openEV */

#endif /* MAIN_GLIDERVARIOMAINPRIV_H_ */
