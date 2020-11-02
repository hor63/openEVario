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
#include <list>

#include "OEVCommon.h"
#include "NmeaGPSDriver.h"
#include "NMEASentence.h"
#include "NMEA0813Protocol.h"


namespace openEV::drivers::NMEA0813 {

class NmeaGPSDriver;

/** \brief Exception thrown by string conversion function of class \ref NMEASet.
 *
 */
class NMEASetParseException: public GliderVarioExceptionBase {
public:

	NMEASetParseException (
			char const *source,
			int line,
			char const *description)
	: GliderVarioExceptionBase{source,line,description}
		{}

};

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

	// Now the enumerations of field indexes of the various NMEA sentence type which are of interest for me

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

	/** \brief Indexes for NMEA0813::NMEASentence::fields for GST sentences
	 *
	 * GST: GNSS pseudorange error statistics
	 */
	enum GSTIndex {
		GST_TIME = 0, ///< hhmmss.ss; UTC time
		GST_RANGE_RMS = 1, ///< double; Total RMS standard deviation of ranges inputs to the navigation solution
		GST_STD_MAJOR = 2, ///< double; Standard deviation (meters) of semi-major axis of error ellipse
		GST_STD_MINOR = 3, ///< double; Standard deviation (meters) of semi-minor axis of error ellipse
		GST_ORIENT = 4, ///< double; Orientation of semi-major axis of error ellipse (true north degrees)
		GST_STD_LAT = 5, ///< double; Standard deviation (meters) of latitude error
		GST_STD_LON = 6, ///< double; Standard deviation (meters) of longitude error
		GST_STD_ALT = 7, ///< double; Standard deviation (meters) of altitude error
	};

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

	// Number of teach-in cycles to run.
	static constexpr uint32_t numTeachInCycles = 10;

	/** \brief The initial value of \ref currSequenceTimestampMS.
	 *
	 * The initial value is used to distinguish if the initial set of values is
	 */

#if defined UINT32_MAX | defined DOXYGEN
	static constexpr uint32_t NMEATimeStampUndef = UINT32_MAX;
#else
	static constexpr uint32_t NMEATimeStampUndef = 0xFFFFFFFFU;
#endif // defined UINT32_MAX

	/** \brief Records of the teach-in/learning phase.
	 *
	 * Each record depicts a sentence type, and the attributes of interest to me
	 *
	 */
	struct TeachInRecord {
		/// Record number to keep track of the position in the list.
		/// Since I am dealing with iterators only I would be loosing track of the position in a
		/// \ref TeachInRecordList
		uint32_t recordNo = 0U;
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

	typedef std::list<TeachInRecord> TeachInRecordList;
	typedef TeachInRecordList::iterator TeachInRecordIter;

	/// \brief Set of NMEA message type which are received during one GNSS fix cycle, i.e. with the same GNSS timstamp.
	struct TeachInCollection {
		/// Record number to keep track of the number of reach-in cycles.
		/// Since I am dealing with iterators only I would be loosing track of the number of teach-in cycles in a
		/// \ref TeachInCollectionList
		uint32_t recordNo = 0U;
		/// Local timestamp of receipt of the first sentence of the current cycle
		std::chrono::system_clock::time_point cycleStart;
		/// The array of records. The number of actually defined records is stored in \p numInCollection.
		TeachInRecordList records;
	};


	/// \brief List of \ref TeachInCollection objects which comprise the teach-in phase
	typedef std::list<TeachInCollection> TeachInCollectionList;
	typedef TeachInCollectionList::iterator TeachInCollectionIter;

	typedef TeachInCollectionList::pointer TeachInCollectionPtr;

	struct CommonRecordItem {
		/// Position in a \ref CommonRecordItemList
		/// Since I am dealing with iterators only I would be loosing track of the number
		uint32_t recordNo;
		TeachInCollectionPtr teachInCollectionPtr;
	};

	/// \brief Store the pointers to common sentence sequences, i.e. equal \ref TeachInCollection.
	typedef std::list<CommonRecordItem> CommonRecordItemList;
	typedef CommonRecordItemList::iterator CommonRecordItemIter;

	/// \brief List item of the list of common cycles
	struct CommonCycle {
		/// \brief Item number in the list
		uint32_t recordNo = 0;
		/// \brief List of common cycles, i.e. series of MNEA sentences with an identical sequence of NMEA message types.
		CommonRecordItemList commonRecordItems;
	};

	typedef std::list<CommonCycle> CommonCycleList;
	typedef CommonCycleList::iterator CommonCycleListIter;

	/// \brief Collection class for storing the sequence of expected NMEA sentences
	///
	/// This is just a list of strings containing the NMEA sentence type.
	typedef std::list<std::string> UsedNMEASentenceTypes;
	/// \brief Constant iterator through an \ref UsedNMEASentenceTypes object
	typedef UsedNMEASentenceTypes::const_iterator UsedNMEASentenceTypesCIter;
	/// \brief Iterator through an \ref UsedNMEASentenceTypes object.

	struct GnssRecord {
		double latitude;		///< Latitude in degrees, North of the equator is positive
		double longitude;		///< Longitude in degrees. East of Greenwich is positive
		bool posDefined;		///< true when \ref latitude and \ref longitude are defined.
		double altMSL;			///< Altitude MSL in meter.
		bool altMslDefined;		///< true when \ref altMSL is defined.
		float latDeviation;		///< Standard deviation of \ref latitude in meters (not degrees!)
		float longDeviation;	///< Standard deviation of \ref longitude in meters
		float altDeviation;		///< Standard deviation of \ref altMSL
		bool pDoPDefined; 		///< Position Dilution of Precision; one DoP value for all dimensions. Lowest priority, and least specific
		bool hDoPDefined;		///< Horizontal Dilution of Precision; one DoP value for \ref latitude and \ref longitude
		bool vDoPDefined;		///< Vertical Dilution of Precision; one DoP value for \ref altMSL
		bool devDirectDefined;	///< Expected errors (standard deviations) for \ref latitude, \ref longitude, and \ref altMSL are explicitly defined.
		 ///< Takes precedence over all DoP values.

		std::chrono::system_clock::time_point recordStart;
		uint32_t gnssTimeStamp;

		/** \brief Initialize the record for a new GNSS fix cycle
		 *
		 * It only resets the ...defined flags.
		 */
		void initialize() {
			posDefined = false;
			altMslDefined = false;
			pDoPDefined = false;
			hDoPDefined = false;
			vDoPDefined = false;
			devDirectDefined = false;
			// Reset to epoch start
			recordStart = std::chrono::system_clock::time_point();
			// And set the GNSS timestamp to an implausible value
			gnssTimeStamp = NMEATimeStampUndef;
		}
		GnssRecord() {
			initialize();
		}
	};

	NMEASet(NmeaGPSDriver& gpsDriver);
	virtual ~NMEASet();

	/** \brief Specialized and fast string to double conversion function
	 *
	 * \p str is a 0-terminated string and must consist of digits only and one optional decimal separator, and an optional leading '-' or '+' character.
	 * The string must be 0-terminated.
	 * Any invalid character in the string leads to a NMEASetParseException exception.\n
	 * The decimal separator is fixed '.'. Locales are not evaluated. \n
	 * Base is always 10. Leading 0[xX] is not evaluated. Leading 0s are just ignored.
	 * Leading space characters are allowed
	 *
	 * As I said: Specialized, but about as fast as I can make it while retaining readability, thorough checking, and not delving into assembler.
	 *
	 * @param str The string
	 * @return double value of the string
	 * @throws NMEASetParseException
	 */
	static double strToD (uint8_t const *str);

	/** \brief Specialized and fast string to double conversion function for a fxied length, not 0-terminated string
	 *
	 * \p str is a string of which only the first \p maxLen characters are being considered. It does therefore not have to be 0-terminated. \n
	 * \p str must consist of digits only and one optional decimal separator, and an optional leading '-' or '+' character.
	 * Any invalid character in the string leads to a NMEASetParseException exception.\n
	 * The decimal separator is fixed '.'. Locales are not evaluated. \n
	 * Base is always 10. Leading 0[xX] is not evaluated. Leading 0s are just ignored.
	 * Leading space characters are allowed
	 *
	 *
	 * @param str The string containing the number
	 * @param maxLen Number of characters up to \p str is valid.
	 * @return double value of the string
	 * @throws NMEASetParseException
	 */
	static double strToD (uint8_t const *str,uint32_t maxLen);

	/** \brief Convert a coordinate string of a NMEA 0813 sentence into decimal degrees
	 *
	 * In NMEA 0813 sentences coordinates are presented as degrees and decimal minutes. \n
	 * There is no separator between the degrees and minutes. The assumption is that the two digits
	 * left of the decimal separator are the minutes. Anything before are the integer digits of the degrees.
	 * The two common forms are:
	 * - Longitude with 3-digit degrees: dddmm.mmm, e.g. 01032.2221 which is 10° 32.2221'
	 * - Latitude with 2-digit degrees: ddmm.mmmm, e.g. 5002.12 is 50° 2.12'
	 *
	 * The number of decimal digits varies for different GNSS receivers (I tested U-Blox Antaris 4, U-Blox 6, and a SiRF 3 or 4.)
	 * Also it is not guaranteed that the number of digits of degrees are always filled up with leading 0s.
	 *
	 * @param str The string containing an coordinate angle in NMEA format.
	 * @return Angle in decimal degrees
	 * @throws NMEASetParseException
	 *
	 */
	static double nmeaCoordToD(uint8_t const * str);


	/** \brief Convert the timestamp from a NMEA sentence into milliseconds since 00:00 UTC of the day.
	 *
	 * The existence of second decimals is optional. \n
	 * The number of second decimals is cut off after max. three decimals
	 *
	 * @param timestampStr String containing the timestamp in the format hhmiss.ssss
	 * @return
	 * @throw NMEASetParseException when non-digit characters except the decimal separator as 7th character occur
	 */
	static uint32_t NMEATimeStampToMS(uint8_t const *timestampStr);

	/** \brief Process a new NMEA sentence which was recieved, parsed, and verified.
	 *
	 * @param newSentence parsed and verified sentence
	 */
	void processSentence(NMEASentence const& newSentence) {
		(this->*processSentenceFunction)(newSentence);
	}

private:

	/// Reference to the owning \ref NmeaGPSDriver object
	NmeaGPSDriver& gpsDriver;

	/** \brief Indicator if teach-in can start
	 *
	 * This indicator is set when the first received sentence contains a valid defined timestamp.
	 * Otherwise I cannot distinguish between different fixes which are sent as a set of NMEA messages.
	 *
	 */
	bool teachInStarted = false;

	/** \brief Collection of records during the teach-in phase
	 *
	 * It is cleared at the end of the teach-in phase.
	 */
	TeachInCollectionList teachInRecords;
	TeachInCollectionIter currTeachInRecord;

	/** \brief Last timestamp of an GNSS fix and message sequence
	 *
	 * This is milliseconds after midnight today, taken from the GPS sentences, not std::chrono::
	 * The initial value can never be reached because the timestamp resets every day.
	 */
	uint32_t currSequenceTimestampMS = NMEATimeStampUndef;

	/// \brief The list of expected NMEA sentences during one GNSS fix cycle
	UsedNMEASentenceTypes usedNMEASentenceTypes;
	/// \brief Current position, and currently expected NMEA sentence in the current GNSS update cycle.
	///
	/// The iterator points into \ref usedNMEASentenceTypes
	UsedNMEASentenceTypesCIter currExpectedSentenceType;

	/** \brief Is the Kalman filter set to the initial position and altitude?
	 *
	 * GNSS processing will wait until a sufficiently precise position and altitude is available.
	 * Then it will set the initial position, and altitude. \n
	 * If altimeter (static pressure) readings are already available the QFF is also initially calculated
	 * to bring altimeter and GPS altitude in synch. \n
	 * (GNSS is usually the latest sensor to come online. Even if the receiver was online before and is
	 * delivering valid data immediately the teach-in phase takes 10 GNSS fix cycles which is between 1 and 10 seconds,
	 * depending on the receiver rate).
	 *
	 * When the initial position and altitude are set \p initialPositionSet is set true.
	 * From that moment the GNSS readings are being used to update the Kalman filter.
	 *
	 */
	bool initialPositionSet = false;

	/** \brief Data of the current GNSS fix cycle
	 *
	 * The various attributes of one GNSS fix are spread over various NMEA sentences.
	 * This record collects the data.
	 */
	GnssRecord currGnssRecord;

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
	void processSentenceTeachIn(NMEASentence const& newSentence);

	/** \brief Finish the teach-in cycle, and switch to regular NMEA sentence processing.
	 *
	 * Analyze the type of sentences which were received during the teach-in phase.
	 * Figure out the minimum set of sentences which I will use for an update of the Kalman filter.
	 * This optimizes the delay between the actual time of the fix, and the receipt and processing of the NMEA sententes of the fix.
	 * Since some receivers only communicate only with 9600 or even 4800 baud the delay is pretty significant, and any optimization here
	 * is effective.
	 */
	void finishTeachIn();

	/** \brief Use the data collected in \ref finishTeachIn() to determine the required set of messages for a Kalman update cycle
	 *
	 * @param commonCycles List of identical collections of NMEA sentence types
	 */
	void determineNMEASet(CommonCycleList& commonCycles);

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
	void processSentenceOperation(NMEASentence const& newSentence);

	/** \brief Determine the timestamp in the received NMEA sentence in milliseconds
	 *
	 * Determine the timestamp in ms since midnight UTC.
	 * If the sentence type does not bear a timestamp by design return the time of the previous sentence.
	 * If the timestamp string is empty, i.e. the GNSS receiver has no defined time base return \ref NMEATimeStampUndef.
	 *
	 * @param newSentence The received and parsed NMEA sentence
	 * @return The timestamp of the sentence in ms. Return \ref NMEATimeStampUndef when the GNSS receiver has no time base.
	 * @throws NMEASetParseException when the timestamp does apply to the convention of the timestamp string.
	 */
	uint32_t getNewSentenceTimestampMS(NMEASentence const& newSentence);

	/** \brief Function pointer to the sentence processor
	 *
	 * Initially it is set to \ref processSentenceTeachIn(). This method will switch to \ref processSentenceOperation()
	 * when the teach-in phase is finished.
	 *
	 * @param newSentence parsed and verified NMEA sentence
	 */
	void (NMEASet::* processSentenceFunction) (NMEASentence const& newSentence) = &processSentenceTeachIn;

};

} /* namespace openEV */

#endif /* DRIVERS_NMEAGPS_NMEASET_H_ */
