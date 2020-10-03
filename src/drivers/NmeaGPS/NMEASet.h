/*
 * NMEASet.h
 *
 *  Created on: 24.09.2020
 *      Author: kai_horstmann
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2020  Kai Horstmann
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

#ifndef DRIVERS_NMEAGPS_NMEASET_H_
#define DRIVERS_NMEAGPS_NMEASET_H_

#include <chrono>
#include <string>

#include "OEVCommon.h"
#include "drivers/NmeaGPS/NmeaGPSDriver.h"
#include "drivers/NmeaGPS/NMEA0813.h"


namespace openEV {

/** \brief Collector of NMEA data from a GNSS receiver for one update cycle
 *
 * This class collects NMEA sentences from a GNSS (vulgo GPS) receiver for one update cycle of the receiver.
 * One cycle is defined by a set of NMEA sentences with the same timestamp (if there is one) which belong together, and report different
 * aspects of the status.
 * Here I am only interested in the position, altitude (MSL), the status (3D or 3D differential, or invalid otherwise),
 * and an error estimate of the fix.
 *
 * To optimize the cycle time at the start I am running a 'teach-in' phase where the object is scanning incoming sentences,
 * and analyzes which sentences are being sent per update cycle. It stores the required sentences and their sequence for later use.
 * In operation per update cycle it only reads the earlier determined sentences, and ignores all sentences which come after even
 * when they belong to the same update cycle. Thus I can optimize the lag between the start of the cycle and the last required sentence
 * received. I simply ignore everything after until the next cycle starts.
 */
class NMEASet {
public:

	// DTM: Datum reference not implemented. Does not carry anything useful

	/** \brief Indexes for NMEA0813::NMEASentence::fields for GBS sentences
	 *
	 * GBS: GPS Satellite Fault Detection
	 *
	 */
	enum GBSIndex {
		GBS_TIME = 0, ///< hhmmss.sss; UTC time to which this RAIM sentence belongs.
		GBS_ERR_LAT = 1, ///< double; Expected error (1-sigma) of latitude in m
		GBS_ERR_LON = 2, ///< double; Expected error (1-sigma) of longitude in m
		GBS_ERR_ALT = 3, ///< double; Expected error (1-sigma) of atitude in m
		GBS_SVID = 4, ///< int; Satellite ID of most likely failed satellite
		GBS_PROB = 5, ///< double; Probability of missed detection for most likely failed satellite
		GBS_BIAS = 6, ///< double; Estimate of bias in meters on most likely failed satellite
		GBS_STDDEV = 7, ///< double; Standard deviation of estimated bias
		GBS_SYSTEM_ID = 8, ///< int; NMEA-defined GNSS system ID (only available in NMEA 4.10 and later)
		GBS_SIGNAL_ID = 9, ///< int; NMEA-defined GNSS signal ID (only available in NMEA 4.10 and later)
	};

	/** \brief Indexes for NMEA0813::NMEASentence::fields for GGA sentences
	 *
	 * GGA: Global positioning system fix data sentence
	 *
	 */
	enum GGAIndex {
		GGA_TIME = 0, ///< hhmmss.ss; UTC time
		GGA_LAT = 1, ///< ddmm.mmmmm; Latitude (degrees and minutes)
		GGA_NS = 2, ///< N/S ; North/South indicator; North: Positive latitude
		GGA_LON = 3, ///< dddmm.mmmmm; Longitude (degrees and minutes)
		GGA_EW = 4, ///< E/W ; East/West indicator; East: Positive longitude
		GGA_QUALITY = 5, ///< digit; Quality indicator for position fix:
		 ///< - 0 - fix not available,
		 ///< - 1 - GPS fix,
		 ///< - 2 - Differential GPS fix (values above 2 are 2.3 features)
		 ///< - 3 = PPS fix
		 ///< - 4 = Real Time Kinematic
		 ///< - 5 = Float RTK
		 ///< - 6 = estimated (dead reckoning)
		 ///< - 7 = Manual input mode
		 ///< - 8 = Simulation mode
		GGA_NUM_SV = 6, ///< int; Number of satellites used (range: 0-12)
		GGA_HDOP = 7, ///< double; Horizontal Dilution of Precision
		GGA_ALT = 8, ///< double; Altitude above mean sea level
		GGA_ALT_UNIT = 9, ///< 'M'; Altitude units: M (meters, fixed field)
		GGA_GEOID_SEP = 10, ///< double; Geoid separation: difference between ellipsoid and mean sea level
		GGA_GEOID_SEP_UNIT = 11, ///< 'M'; Unit of geoid separation: M (meters, fixed field)
		GGA_DIFF_AGE = 12, ///< double; Age of differential corrections in seconds (null when DGPS is not used)
		GGA_DIFF_STATION = 13 ///< int; ID of station providing differential corrections (null when DGPS is not used)
	};

	/** \brief Indexes for NMEA0813::NMEASentence::fields for GLL sentences
	 *
	 * GLL: Latitude and longitude, with time of position fix and status
	 */
	enum GLLIndex {
		GLL_LAT = 0, ///< ddmm.mmmmm; Latitude (degrees and minutes)
		GLL_NS = 1, ///< N/S ; North/South indicator; North: Positive latitude
		GLL_LON = 2, ///< dddmm.mmmmm; Longitude (degrees and minutes)
		GLL_EW = 3, ///< E/W ; East/West indicator; East: Positive longitude
		GLL_TIME = 4, ///< hhmmss.ss; UTC time
		GLL_STATUS = 5, ///< A/V; Data validity status: V = Invalid, A = valid
		GLL_POS_MODE = 6, ///< character; Positioning mode (only available in NMEA 2.3 and later):
		 ///< - N = No fix
		 ///< - E = Estimated/Dead reckoning fix
		 ///< - A = Autonomous GNSS fix (OK)
		 ///< - D = Differential GNSS fix (OK)
		 ///< - P = Precise (OK)
		 ///< - F = RTK float
		 ///< - R = RTK fixed
		 ///<
		 ///< Only A, D, and P are usable fixes for me
	};

	/** \brief Indexes for NMEA0813::NMEASentence::fields for GNS sentences
	 *
	 * GNS: GNSS fix data
	 */
	enum GNSIndex {
		GNS_TIME = 0, ///< hhmmss.ss; UTC time
		GNS_LAT = 1, ///< ddmm.mmmmm; Latitude (degrees and minutes)
		GNS_NS = 2, ///< N/S ; North/South indicator; North: Positive latitude
		GNS_LON = 3, ///< dddmm.mmmmm; Longitude (degrees and minutes)
		GNS_EW = 4, ///< E/W ; East/West indicator; East: Positive longitude
		GNS_POS_MODE = 5, ///< String (4 char); Positioning mode (only available in NMEA 2.3 and later): \n
		 ///< 1st Char GPS, 2nd Char GLONASS, 3rd Char Galileo, 4th Char BeiDou \n
		 ///< - N = No fix
		 ///< - E = Estimated/Dead reckoning fix
		 ///< - A = Autonomous GNSS fix (OK)
		 ///< - D = Differential GNSS fix (OK)
		 ///< - P = Precise (OK)
		 ///< - F = RTK float
		 ///< - R = RTK fixed
		 ///<
		 ///< Only A, D, and P are usable fixes for me
		GNS_NUM_SV = 6, ///< int; Number of satellites used (range: 0-12)
		GNS_HDOP = 7, ///< double; Horizontal Dilution of Precision
		GNS_ALT = 8, ///< double; Altitude above mean sea level
		GNS_GEOID_SEP = 10, ///< double; Geoid separation: difference between ellipsoid and mean sea level
		GNS_DIFF_AGE = 12, ///< double; Age of differential corrections in seconds (null when DGPS is not used)
		GNS_DIFF_STATION = 13, ///< int; ID of station providing differential corrections (null when DGPS is not used)
	};


	// GRS: GNSS range residuals: Not useful

	/** \brief Indexes for NMEA0813::NMEASentence::fields for GSA sentences
	 *
	 * GSA: GNSS DOP and active satellites
	 */
	enum GSAIndex {
		GSA_OP_MODE = 0, ///< character; Operation mode:
		 ///< - M = Manually set to operate in 2D or 3D mode
		 ///< - A = Automatically switching between 2D or 3D mode
		GSA_NAV_MODE = 1, ///< digit; Kind of fix:
		 ///< - 1 = No fix
		 ///< - 2 = 2D fix
		 ///< - 3 = 3D fix
		GSA_SV_ID_1 = 2, ///< 1st satellite number
		GSA_SV_ID_2 = 3, ///< 1st satellite number
		GSA_SV_ID_3 = 4, ///< 1st satellite number
		GSA_SV_ID_4 = 5, ///< 1st satellite number
		GSA_SV_ID_5 = 6, ///< 1st satellite number
		GSA_SV_ID_6 = 7, ///< 1st satellite number
		GSA_SV_ID_7 = 8, ///< 1st satellite number
		GSA_SV_ID_8 = 9, ///< 1st satellite number
		GSA_SV_ID_9 = 10, ///< 1st satellite number
		GSA_SV_ID_10 = 11, ///< 1st satellite number
		GSA_SV_ID_11 = 12, ///< 1st satellite number
		GSA_SV_ID_12 = 13, ///< 1st satellite number
		GSA_PDOP = 14, ///< double; Position Dilution of Precision
		GSA_HDOP = 15, ///< double; Horizontal Dilution of Precision
		GSA_VDOP = 16, ///< double; Vertical Dilution of Precision
		GSA_SYSTEM_ID = 17, ///< int; NMEA-defined GNSS system ID, (only available in NMEA 4.10 and later):
		 ///< - 1: GPS
		 ///< - 2: GLONASS
		 ///< - 3: Galileo
		 ///< - 4: BeiDou
	};

	// GSV: Satellites in view: Not useful


	// RMA, RMB: Never seen by a GNSS receiver

	/** \brief Indexes for NMEA0813::NMEASentence::fields for RMC sentences
	 *
	 * RMC: Recommended minimum data
	 */
	enum RMCIndex {
		RMC_TIME = 0, ///< hhmmss.ss; UTC time
		RMC_STATUS = 1, ///< A/V; Data validity status: V = Invalid, A = valid
		RMC_LAT = 2, ///< ddmm.mmmmm; Latitude (degrees and minutes)
		RMC_NS = 3, ///< N/S ; North/South indicator; North: Positive latitude
		RMC_LON = 4, ///< dddmm.mmmmm; Longitude (degrees and minutes)
		RMC_EW = 5, ///< E/W ; East/West indicator; East: Positive longitude
		RMC_SPEED = 6, ///< double; Speed over ground
		RMC_COG = 7, ///< double; Course over ground
		RMC_DATE = 8, ///< ddmmyy; Day/Month/Year
		RMV_MV = 9, ///< double; Magnetic variation. Only supported in ADR 4.10 and later
		RMC_MV_EW = 10, ///< E/W; East/West. Only supported in ADR 4.10 and later
		RMC_POS_MODE = 11, ///< character; FAA mode indicator (NMEA 2.3 and later)
		 ///< - N = No fix
		 ///< - E = Estimated/Dead reckoning fix
		 ///< - A = Autonomous GNSS fix (OK)
		 ///< - D = Differential GNSS fix (OK)
		 ///< - P = Precise (OK)
		 ///< - F = RTK float
		 ///< - R = RTK fixed
		 ///<
		 ///< Only A, D, and P are usable fixes for me
		RMC_NAV_STATUS = 12, ///< character; Navigational status indicator (only available in NMEA 4.10 and later) U-Blox sets fixed 'V'
	};

	// TXT, VTG, and ZDA are useless for me.

	/** \brief Hard-coded number of distinct types of sentences per cycle
	 *
	 * If there are more sentences types they are going to be cut off, and not worth looking at :)
	 * Usually there should not be more than 2-4 sentences per cycle which I am interested in.
	 */
	static constexpr uint32_t numExpectedSentencesPerCycle = 16;
	// Number of teach-in cycles to run.
	static constexpr uint32_t numTeachInCycles = 10;


	/** \brief Records of the teach-in/learning phase.
	 *
	 * Each record depicts a sentence type, and the attributes of interest to me
	 *
	 */
	struct TeachInRecord {
		/// The type of the sentence like "RMC", "GGA", "GLL" etc.
		std::string recordType;
		/// The delay of the sentence from the start of a cycle
		std::chrono::system_clock::duration timeAfterCycleStart;
		/// Defines longitude and latitude
		bool definesPosition = false;
		/// Defines altitude MSL. Altitude above geoid is pretty useless to me.
		bool definesMSL = false;
		/// Defines the level of fix qualitiy (no fix, 2D fix, 3D fix)
		bool definesQualitiyLevel = false;
		/// Defines differential mode indicator
		bool definesDifferentialMode = false;
		/// Defines overall positional Dilation of Precision
		bool definesPDop = false;
		/// Defines horizontal Dilation of Precision
		bool definesHDop = false;
		/// Defines vertical Dilation of Precision (altitude)
		bool definesVDop = false;
		/// Defines absolute error margins for latitude, longitude, and altitude.
		bool definesAbsErr = false;
	};

	struct TeachInCollection {
		/// Number of valid records in \p records
		uint32_t numInCollection = 0;
		/// The array of records. The number of actually defined records is stored in \p numInCollection.
		TeachInRecord records [numExpectedSentencesPerCycle];
	};

	NMEASet();
	virtual ~NMEASet();

	/** \brief calculate the hhmmss.sss string of a NMEA timestamp to milliseconds
	 *
	 * @param timestampStr Timestamp in the NMEA timestamp format hhmmss
	 * @return
	 */
	static uint32_t NMEATimeStampToMS(uint8_t const *timestampStr);

	/** \brief Process a new NMEA sentence which was recieved, parsed, and verified.
	 *
	 * @param newSentence parsed and verified sentence
	 */
	void processSentence(NMEA0813::NMEASentence const& newSentence) {
		(this->*processSentenceFunction)(newSentence);
	}

private:


	/** \brief Indicator if teach-in can start
	 *
	 * This indicator is set when the first received sentence contains a valid defined timestamp.
	 * Otherwise I cannot distinguish between different fixes which are sent as a set of NMEA messages.
	 *
	 */
	bool teachInStarted = false;

	/** \brief Collection of records during the teach-in phase
	 *
	 * It is created with the constructor of the object and freed at the end of the teach-in phase.
	 */
	TeachInCollection *teachInRecords;

	/** Number of teach-in cycles have been completed.
	uint32_t numTeachInCycles = 0;

	/** \brief Last timestamp of an GNSS fix and message sequence
	 *
	 * This is milliseconds after midnight today, taken from the GPS sentences, not std::chrono::
	 * The initial value can never be reached because the timestamp resets every day.
	 */
	uint32_t currSequenceTimestampMS = 0xFFFFFFFFU;

	/** \brief System timestamp when the first sentence of the current sequence was received.
	 *
	 */

	std::chrono::system_clock::time_point currSequenctStart;

	/** \brief Process a new NMEA sentence in teach-in mode.
	 *
	 * In teach-in mode the system uses 10 cycles to figure out in which order sentences are being sent,
	 * and which ones are going to be used in operation mode.
	 *
	 * I am interested only in position, altitude MSL, and error estimates.
	 * The goal is to obtain these sentences as early in a cycle to minimize the latency between real time and availability of a
	 * GNSS update.
	 *
	 * @param newSentence parsed and verified NMEA sentence
	 */
	void processSentenceTeachIn(NMEA0813::NMEASentence const& newSentence);

	/** \brief Process a new NMEA sentence in operations mode.
	 *
	 * In teach-in mode the system uses 10 cycles to figure out in which order sentences are being sent,
	 * and which ones are going to be used in operation mode.
	 *
	 * I am interested only in position, altitude MSL, and error estimates.
	 * The goal is to obtain these sentences as early in a cycle to minimize the latency between real time and availability of a
	 * GNSS update.
	 *
	 * @param newSentence parsed and verified NMEA sentence
	 */
	void processSentenceOperation(NMEA0813::NMEASentence const& newSentence);

	/** \brief Function pointer to the sentence processor
	 *
	 * Initially it is set to \ref processSentenceTeachIn(). This method will switch to \ref processSentenceOperation()
	 * when the teach-in phase is finished.
	 *
	 * @param newSentence parsed and verified NMEA sentence
	 */
	void (NMEASet::* processSentenceFunction) (NMEA0813::NMEASentence const& newSentence) = &processSentenceTeachIn;



};

} /* namespace openEV */

#endif /* DRIVERS_NMEAGPS_NMEASET_H_ */
