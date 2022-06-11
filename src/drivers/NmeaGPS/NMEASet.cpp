/*
 * NMEASet.cpp
 *
 *  Created on: 24.09.2020
 *      Author: kai_horstmann
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

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <sstream>

#include "NMEA0813Protocol.h"
#include "NMEASet.h"
#include "NmeaGPSDriver.h"
#include "kalman/GliderVarioTransitionMatrix.h"
#include "kalman/GliderVarioMeasurementUpdater.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.Drivers.NmeaGPS.NMEASet");
	}
}

#endif

namespace openEV::drivers::NMEA0813 {

NMEASet::NMEASet(NmeaGPSDriver& gpsDriver)
		:gpsDriver {gpsDriver},
		teachInRecords{},
		currTeachInRecord {teachInRecords.end()}
		{

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif

}

NMEASet::~NMEASet() {

}

/** \brief My inline and faster isdigit variant
 *
 * @param c character to check
 * @return true if \p c is between '0' and '9' ASCII.
 */
inline bool isDigit (int c) {
	return (c>='0' && c<='9');
}

double NMEASet::strToD (uint8_t const *str) {
	double rc = 0.0;
	double sign = 1.0;
	double factor = 1.0;
	uint8_t const *lStr = str;

	// Eat up leading space
	while (*lStr == ' ') lStr++;

	if (*lStr == '+') {
		// A leading + is allowed, but does not change a thing
		lStr ++;
	} else {
		if (*lStr == '-') {
			sign = -1.0;
		}
	}
	for (; isDigit(*lStr);lStr ++) {
		rc = rc * 10.0 + double(*lStr - '0');
	}

	if (*lStr == '.') {
		// Eat up the decimal separator
		lStr ++;

		// now come the decimal places when there are any.
		for (; isDigit(*lStr);lStr ++) {
			factor /= 10.0;
			rc += double(*lStr - '0') * factor;
		}
	}

	// Check if I reached the end of the string. Else there is invalid trailing stuff
	// This is not allowed for this function.
	if (*lStr != 0) {
		std::ostringstream s;
		s << "strToD: String \"" << str << "\" is not a valid double value";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	LOG4CXX_TRACE (logger,"strToD: String \"" << str << "\" converted to " << (rc * sign));
	return rc * sign;
}

double NMEASet::strToD (uint8_t const *str,uint32_t maxLen) {
	uint8_t * strippedStr = new uint8_t[maxLen + 1];
	double rc;

	// Copy the designated part of the string
	memcpy(strippedStr,str,maxLen);
	// Terminate it
	strippedStr[maxLen] = 0;

	try {
		rc = strToD(strippedStr);
	}
	catch (...) {
		delete strippedStr;
		throw;
	}

	delete strippedStr;
	return rc;
}

double NMEASet::nmeaCoordToD(uint8_t const * str) {

	double degrees;
	double minutes;
	double rc;
	uint32_t i;
	uint8_t const *lStr;

	// Look for the decimal.
	for (lStr = str;*lStr != 0; lStr ++) {
		if (*lStr == '.') break;
	}

	if (*lStr == 0) {
		std::ostringstream s;
		s << "nmeaCoordToD: String \"" << str << "\" does not contain a decimal '.'";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	// Where is the dot in the String?
	i = lStr - str;
	if (i < 3) {
		std::ostringstream s;
		s << "nmeaCoordToD: String \"" << str << "\" does not have space for degrees '.'";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	// Get the degrees
	degrees = strToD(str,i-2);
	minutes = strToD (str + (i-2));
	rc = degrees + minutes / 60.0;

	LOG4CXX_TRACE(logger,"nmeaCoordToD: \"" << str << "\" converted to " << degrees << " Degrees, " << minutes << " minutes = "
			<< rc);

	return rc;
}

uint32_t NMEASet::NMEATimeStampToMS(uint8_t const *timestampStr) {
	uint32_t rc = 0xFFFFFFFFU;
	uint32_t i = 0;
	char const *lStr = (char const *)(timestampStr);

	// First two characters are hours
	if (isDigit(*lStr)) {
		rc = (*lStr - '0') * (1000*3600*10);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000*3600);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}


	// Next two characters are minutes
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000*60*10);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000*60);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}

	// Next two characters are seconds
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000*10);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}
	if (isDigit(*lStr)) {
		rc += (*lStr - '0') * (1000);
		lStr ++;
	} else {
		// Not a digit. This is therefore not a valid timestamp string
		std::ostringstream str;
		str << "NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" is not a valid NMEA timestamp string.";
		LOG4CXX_WARN(logger,str.str());
		throw NMEASetParseException(__FILE__,__LINE__,str.str().c_str());
	}

	if (*lStr == '.') {
		lStr ++;
		for (i = 100; i >= 1; i /= 10) {
			if (isDigit(*lStr)) {
				rc += (*lStr - '0') * i;
				lStr ++;
			} else {
				break;
			}
		}
	}

	LOG4CXX_TRACE(logger,"NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" converted to " << rc);

	return rc;
}

void NMEASet::processSentenceTeachIn(
		const NMEASentence &newSentence) {

	auto currTime = OEVClock::now();

	// Some sentences do no carry a GPS timestmap. Therefore by default assume the same cycle.
	auto thisGPSTimeStampMS = currSequenceTimestampMS;
	TeachInRecord locRecord;
	bool useLocRecord = false;

	locRecord.recordTypeString = (char const*)(newSentence.sentenceTypeString);
	locRecord.recordType = newSentence.sentenceType;

	try {

		LOG4CXX_DEBUG(logger,"processSentenceTeachIn: Process message type " << newSentence.sentenceTypeString);

		switch (newSentence.sentenceType) {
		case NMEASentence::NMEA_RMC:
			// If the timestamp is undefined the GNSS receiver is totally in the dark,
			// and does not even have a battery backed RTC.
			// Do not use teach-in now.
			if (*(newSentence.fields[RMC_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[RMC_TIME]);
				useLocRecord = true;
				locRecord.definesDifferentialMode = true;
				locRecord.definesPosition = true;
			}
			break;

		case NMEASentence::NMEA_GGA:
			// If the timestamp is undefined the GNSS receiver is totally in the dark,
			// and does not even have a battery backed RTC.
			// Do not use teach-in now.
			if (*(newSentence.fields[GGA_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GGA_TIME]);
				useLocRecord = true;
				locRecord.definesDifferentialMode = true;
				locRecord.definesHDop = true;
				locRecord.definesMSL = true;
				locRecord.definesPosition = true;
			}
			break;

		case NMEASentence::NMEA_GLL:
			if (*(newSentence.fields[GLL_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GLL_TIME]);
				useLocRecord = true;
				locRecord.definesPosition = true;
				locRecord.definesDifferentialMode = true;
				locRecord.definesHDop = true;
			}
			break;

		case NMEASentence::NMEA_GNS:
			if (*(newSentence.fields[GNS_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GNS_TIME]);
				useLocRecord = true;
				locRecord.definesPosition = true;
				locRecord.definesHDop = true;
				locRecord.definesDifferentialMode = true;
				locRecord.definesMSL = true;
			}
			break;

		case NMEASentence::NMEA_GST:
			// If the timestamp is undefined the GNSS receiver is totally in the dark,
			// and does not even have a battery backed RTC.
			// Do not use teach-in now.
			if (*(newSentence.fields[GST_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GST_TIME]);
				useLocRecord = true;
				locRecord.definesAbsErr = true;
			}
			break;

		case NMEASentence::NMEA_GSA:
			// GSA is a bit unique because it does not have an own timestamp field.
			// It kind of swims in the fix update set with other NMEA sentences.
			// For simplicity, and practical experience with GNSS receivers I assume
			// the timestamp is the same as the previous record.
			useLocRecord = true;
			// Because I do not have a timestamp I cannot decide if teach-in started.
			// Teach-in starts when the timestamp changes compared to the previous cycle,
			// or to the initial unplausible timestamp.
			locRecord.definesHDop = true;
			locRecord.definesPDop = true;
			locRecord.definesVDop = true;
			locRecord.definesQualitiyLevel = true;
			break;

		case NMEASentence::NMEA_GBS:
			// If the timestamp is undefined the GNSS receiver is totally in the dark,
			// and does not even have a battery backed RTC.
			// Do not use teach-in now.
			if (*(newSentence.fields[GBS_TIME]) != 0) {
				thisGPSTimeStampMS = NMEATimeStampToMS(newSentence.fields[GBS_TIME]);
				useLocRecord = true;
				locRecord.definesAbsErr = true;
			}
			break;

		default:
			LOG4CXX_DEBUG(logger,"processSentenceTeachIn: Throw out un-supported sentence with type " << newSentence.sentenceTypeString);
		}

	}
	catch (NMEASetParseException const& e) {
		LOG4CXX_WARN(logger,"processSentenceTeachIn: Sentence " << newSentence.sentenceTypeString << " failed. Reason: " << e.what());
	}

	if (useLocRecord) {

		if (currSequenceTimestampMS != thisGPSTimeStampMS) {
			// If the last timestamp is at the initial value throw away messages until
			// the next timestamp change occurs.
			// Only this way I can be sure to catch the first start of a new cycle properly.
			if (currSequenceTimestampMS != NMEATimeStampUndef) {
				if (teachInRecords.size() == numTeachInCycles) {
					finishTeachIn();

					// Initialize the iterator for operational mode.
					currExpectedSentenceType = usedNMEASentenceTypes.cbegin();

					// Switch to the operational message processor
					// Without that this method would be called forever and be stuck here.
					processSentenceFunction =
							&openEV::drivers::NMEA0813::NMEASet::processSentenceOperation;

					// Cleanout the collection to free up memory
					teachInRecords.clear();
					currTeachInRecord = teachInRecords.end();

					return;
				}

				// Append an initialized record at the end
				currTeachInRecord = teachInRecords.emplace(teachInRecords.end());

				currTeachInRecord->cycleStart = currTime;
				currTeachInRecord->recordNo = teachInRecords.size() - 1;

				// I am progressing from a previous real timestamp to the next one.
				// This means this is really the start of a new cycle
				teachInStarted = true;
				LOG4CXX_TRACE(logger,"processSentenceTeachIn: Last timestamp = " << currSequenceTimestampMS
						<< " New timestamp = " << thisGPSTimeStampMS
						<< ". Teach-in in progress. New numTeachInCyclesExecuted = " << teachInRecords.size());
			}
			// this is the start of a new cycle
			currSequenceTimestampMS = thisGPSTimeStampMS;
			// Store the local timestamp as start of the new cycle

		}

		if (teachInStarted) {
		TeachInCollection& currCollection = *currTeachInRecord;

			LOG4CXX_DEBUG(logger, "processSentenceTeachIn: Add message " << newSentence.sentenceTypeString
					<< " to collection #"  << teachInRecords.size());

			locRecord.timeAfterCycleStart = currTime - currCollection.cycleStart;
			currCollection.records.insert(currCollection.records.end(),locRecord);
		}
	}
}

inline void NMEASet::finishTeachIn() {

	CommonCycleList commonCycles;

	CommonRecordItem commonRecord;
	TeachInCollectionIter currTeachInCollectionIter =  teachInRecords.begin();

	// If there are no recoded records the behavior would be undefined.
	if (currTeachInCollectionIter == teachInRecords.end()) {
		return;
	}

	commonRecord.recordNo = 0;
	commonRecord.teachInCollectionPtr = &(*currTeachInCollectionIter);
	// The first record is by definition always the first template.
	commonCycles.emplace_front();
	commonCycles.begin()->commonRecordItems.insert(commonCycles.begin()->commonRecordItems.cend(),commonRecord);


	for (currTeachInCollectionIter++; currTeachInCollectionIter != teachInRecords.end(); currTeachInCollectionIter++) {
		bool foundRecord = false;
		TeachInCollection *newRecord = nullptr;

		// Compare the cycles. Run through the previous collected common sets.
		for (CommonCycleListIter k = commonCycles.begin(); k != commonCycles.end(); k++) {
			newRecord = &(*currTeachInCollectionIter);
			TeachInCollection *refRecord = k->commonRecordItems.begin()->teachInCollectionPtr;
			if (newRecord->records.size() == refRecord->records.size()) {
				TeachInRecordIter newSentenceTypeIter = newRecord->records.begin();
				TeachInRecordIter refSentenceTypeIter = refRecord->records.begin();
				foundRecord = true;
				for (;newSentenceTypeIter != newRecord->records.end() && refSentenceTypeIter != refRecord->records.end();
						newSentenceTypeIter++,refSentenceTypeIter++) {
					if (newSentenceTypeIter->recordType != refSentenceTypeIter->recordType) {
						foundRecord = false;
						break;
					}
				} // for (uint32_t l = 0; l < newRecord.numInCollection; l++)
			} // if (newRecord.numInCollection == refRecord.numInCollection)

			if (foundRecord) {
				LOG4CXX_DEBUG(logger,
						"finishTeachIn: Collection #" << newRecord->recordNo << " joins common record #" << k->recordNo
						<< " as member #" << k->commonRecordItems.size());

				// Store the pointer to the current collection
				CommonRecordItemIter newCommonRecord = k->commonRecordItems.emplace(k->commonRecordItems.cend());
				newCommonRecord->recordNo = k->commonRecordItems.size() -1;
				newCommonRecord->teachInCollectionPtr = newRecord;

				// And finish the search for the current record because I had a hit.
				break;
			}

		} // for (uint32_t k = 0; commonRecords[k].numEqualRecords != 0; k++)

		if (!foundRecord && newRecord != nullptr) {
			// This collection defines a new template.
			CommonCycleListIter newCommonCycle = commonCycles.emplace(commonCycles.cend());
			newCommonCycle->recordNo = commonCycles.size() - 1;
			CommonRecordItemIter newCommonRecordItem = newCommonCycle->commonRecordItems.emplace(newCommonCycle->commonRecordItems.cend());
			newCommonRecordItem->recordNo = 0;
			newCommonRecordItem->teachInCollectionPtr = newRecord;
			LOG4CXX_DEBUG(logger,"finishTeachIn: Add collection #" << newRecord->recordNo
					<< " as new template #" << newCommonCycle->recordNo);
		}
	} // for (uint32_t i = 1; i < numTeachInCycles; i++)

	// Finally determine the dominant set of MNEA sentence types which the GNSS receiver sends per update cycle.
	NMEASet::determineNMEASet(commonCycles);
}


void NMEASet::determineNMEASet(CommonCycleList& commonCycles) {

	CommonCycleListIter usedCommonSequence = commonCycles.begin();
	// First determine the one set with more then 50% of occurrences.
	for (; usedCommonSequence != commonCycles.end();usedCommonSequence++){
		if (usedCommonSequence->commonRecordItems.size() > (numTeachInCycles/2)) {
			// This is the one. There can not be one with more entries
			// Because this one has already more than half.
			LOG4CXX_DEBUG(logger,"determineNMEASet: Found dominant common sequence # " << usedCommonSequence->recordNo
					<< " with " << usedCommonSequence->commonRecordItems.size() << " equal sets.");
			break;
		}
	}

	// If I cannot determine the absolute winner try to determine the relative winner
	if (usedCommonSequence != commonCycles.end()) {
		usedCommonSequence = commonCycles.begin();
		for (CommonCycleListIter i = commonCycles.begin(); i != commonCycles.end();i++){
			if (usedCommonSequence->commonRecordItems.size() < i->commonRecordItems.size()) {
				usedCommonSequence = i;
			}
		LOG4CXX_DEBUG(logger,"determineNMEASet: The major common sequence is #" << usedCommonSequence->recordNo
				<< ". It has " << usedCommonSequence->commonRecordItems.size() << " equal sets.");
		}
	}

	if (usedCommonSequence->commonRecordItems.size() < 3) {
		LOG4CXX_WARN(logger,"determineNMEASet: Could not find a common sequence shared by at least 3 cycles. Go by individual cycle analysis.");
	} else {
		// Use a teach-in record to collect which flags have been set so far by NMEA sentences.
		TeachInRecord collectiveFlags;

		auto usedTeachInCollection = usedCommonSequence->commonRecordItems.begin()->teachInCollectionPtr;

		for (auto i = usedTeachInCollection->records.begin();i != usedTeachInCollection->records.end(); i++) {
			TeachInRecord &currTeachInRecord = *i;
			// Check if the current record brings any new attributes.
			if (
					(currTeachInRecord.definesDifferentialMode && !collectiveFlags.definesDifferentialMode) ||
					(currTeachInRecord.definesMSL && !collectiveFlags.definesMSL) ||
					(currTeachInRecord.definesPosition && !collectiveFlags.definesPosition) ||
					(currTeachInRecord.definesQualitiyLevel && !collectiveFlags.definesQualitiyLevel) ||

					// Here some priorization: When I have HDOP and VDOP or AbsErr I do not care about PDOP any more
					( !collectiveFlags.definesAbsErr &&
						(!collectiveFlags.definesHDop || !collectiveFlags.definesVDop) &&
						currTeachInRecord.definesPDop && !collectiveFlags.definesPDop
					) ||

					// Some either-ors: When I have HDOP and VDOP I do not need AbsErr
					( (!collectiveFlags.definesHDop || !collectiveFlags.definesVDop) &&
						(currTeachInRecord.definesAbsErr && !collectiveFlags.definesAbsErr)
					) ||
					// If I have AbsErr I not not need HDOP and PDOP any more.
					( !collectiveFlags.definesAbsErr &&
						((currTeachInRecord.definesHDop && !collectiveFlags.definesHDop) ||
						 (currTeachInRecord.definesVDop && !collectiveFlags.definesVDop)
						)
					)

				) {

#if defined HAVE_LOG4CXX_H
				if (logger->isDebugEnabled()) {
					std::ostringstream str;
					str << "determineNMEASet: Add " << currTeachInRecord.recordTypeString << ". It defines";

					if (!collectiveFlags.definesAbsErr && currTeachInRecord.definesAbsErr){
						str << " AbsErr,";
					}
					if (!collectiveFlags.definesDifferentialMode && currTeachInRecord.definesDifferentialMode){
						str << " DifferentialMode,";
					}
					if (!collectiveFlags.definesHDop && currTeachInRecord.definesHDop){
						str << " HDop,";
					}
					if (!collectiveFlags.definesVDop && currTeachInRecord.definesVDop){
						str << " VDop,";
					}
					if (!collectiveFlags.definesPDop && currTeachInRecord.definesPDop){
						str << " PDop,";
					}
					if (!collectiveFlags.definesMSL && currTeachInRecord.definesMSL){
						str << " MSL,";
					}
					if (!collectiveFlags.definesPosition && currTeachInRecord.definesPosition){
						str << " Position,";
					}
					if (!collectiveFlags.definesQualitiyLevel && currTeachInRecord.definesQualitiyLevel){
						str << " QualityLevel";
					}
					LOG4CXX_DEBUG(logger,str.str());
				} // if (logger->isDebugEnabled())

#endif // #if defined HAVE_LOG4CXX_H

				// Set any new flags in the collective attribute set
				collectiveFlags.definesAbsErr 			|= currTeachInRecord.definesAbsErr;
				collectiveFlags.definesDifferentialMode	|= currTeachInRecord.definesDifferentialMode;
				collectiveFlags.definesHDop 			|= currTeachInRecord.definesHDop;
				collectiveFlags.definesMSL 				|= currTeachInRecord.definesMSL;
				collectiveFlags.definesPDop 			|= currTeachInRecord.definesPDop;
				collectiveFlags.definesPosition 		|= currTeachInRecord.definesPosition;
				collectiveFlags.definesQualitiyLevel 	|= currTeachInRecord.definesQualitiyLevel;
				collectiveFlags.definesVDop 			|= currTeachInRecord.definesVDop;

				// This record brings a new aspect into my life. Store it.
				usedNMEASentenceTypes.insert(usedNMEASentenceTypes.cend(),currTeachInRecord.recordType);
			} // if ( relevant attributes are defined which were not defined before.
		} // for (uint32_t i = 0;i < usedTeachInCollection.numInCollection; i++)
	} // if (commonRecords[usedCommonSequence].numEqualRecords < 3) {...} else {

}

uint32_t NMEASet::getNewSentenceTimestampMS(NMEASentence const& newSentence) {

	uint32_t thisGPSTimeStampMS = NMEATimeStampUndef;
	uint8_t const * timestampString = nullptr;

	LOG4CXX_TRACE(logger,"processSentenceOperation: Message " << newSentence.talkerID << ' ' << newSentence.sentenceTypeString
			<< ", internal type " << newSentence.sentenceType);

	LOG4CXX_TRACE(logger,"processSentenceTeachIn: Process message type " << newSentence.sentenceTypeString);

	switch (newSentence.sentenceType) {
	case NMEASentence::NMEA_RMC:
		timestampString = newSentence.fields[RMC_TIME];
		break;

	case NMEASentence::NMEA_GGA:
		timestampString = newSentence.fields[GGA_TIME];
		break;

	case NMEASentence::NMEA_GLL:
		timestampString = newSentence.fields[GLL_TIME];
		break;

	case NMEASentence::NMEA_GNS:
		timestampString = newSentence.fields[GNS_TIME];
		break;

	case NMEASentence::NMEA_GST:
		timestampString = newSentence.fields[GST_TIME];
		break;

	case NMEASentence::NMEA_GSA:
		// This case is special: The record does not have a timestamp field.
		// Therefore just assume the same timestamp as the record before.
		thisGPSTimeStampMS = currGnssRecord.gnssTimeStamp;
		break;

	case NMEASentence::NMEA_GBS:
		timestampString = newSentence.fields[GBS_TIME];
		break;

	default:
		// Why am I here? Un-supported sentence types should have been thrown out by the caller,
		// i.e. NMEA0813Protocol::parseSentence()
		LOG4CXX_WARN(logger,"NMEASet::processSentenceOperation: Un-supported sentence type " << newSentence.sentenceType
				<< " from talker/type" << newSentence.talkerID << '/' << newSentence.sentenceTypeString << " is discarded.");
	}

	if (timestampString != nullptr) {
		if (*timestampString != 0) {
			LOG4CXX_TRACE(logger, "getNewSentenceTimestampMS: Timestamp string = " << timestampString);
			thisGPSTimeStampMS = NMEATimeStampToMS(timestampString);
		}
	}

	LOG4CXX_TRACE(logger, "getNewSentenceTimestampMS: Return = " << thisGPSTimeStampMS);
	return thisGPSTimeStampMS;
}

void NMEASet::processSentenceOperation(
		const NMEASentence &newSentence) {

	try {

		uint32_t thisGPSTimeStampMS = getNewSentenceTimestampMS(newSentence);

		if (thisGPSTimeStampMS != NMEATimeStampUndef &&
				thisGPSTimeStampMS != currGnssRecord.gnssTimeStamp	) {
			// The record contains a valid timestamp, and it is different from the last cycle.

			// Close out the previous cycle when it was not finished before
			if (!currGnssRecord.recordProcessed) {
				updateKalmanFilter (/*endOfCycle*/true);
			}

			// Therefore start a new cycle.
			LOG4CXX_TRACE(logger,"Start new cycle. Old GNSS timestamp = " << currGnssRecord.gnssTimeStamp
					<< ", new GNSS timestamp = " << thisGPSTimeStampMS);

			currExpectedSentenceType = usedNMEASentenceTypes.cbegin();
			currGnssRecord.initialize();
			currGnssRecord.gnssTimeStamp = thisGPSTimeStampMS;
			currGnssRecord.recordStart = OEVClock::now();
		}

		// Check if the current cycle was complete and use to update the Kalman filter already
		// and if the sentence type is the one which is currently expected,
		// or if the expected type list is empty, and we are in promiscuous mode
		LOG4CXX_TRACE(logger,"Check if message must be processed: currGnssRecord.recordProcessed = " << currGnssRecord.recordProcessed
				<< ", usedNMEASentenceTypes.size() = " << usedNMEASentenceTypes.size()
				<< ", currExpectedSentenceType != usedNMEASentenceTypes.cend() = " << (currExpectedSentenceType != usedNMEASentenceTypes.cend())
				<< ", *currExpectedSentenceType = " << *currExpectedSentenceType
				<< ", newSentence.sentenceType = " << newSentence.sentenceType);

		/* !!!Test +/
		currGnssRecord.recordProcessed = false;
		currGnssRecord.initialize();
		/+ */

		/* !!!Test */
		if (!currGnssRecord.recordProcessed &&
				(usedNMEASentenceTypes.size() == 0 ||
				 (currExpectedSentenceType != usedNMEASentenceTypes.cend() &&
				  *currExpectedSentenceType == newSentence.sentenceType)))
		/* */ {
			// Check if you got all required messages in targeted mode
			if (usedNMEASentenceTypes.size() > 0) {
				// Advance to the next expected sentence type
				// Do that before processing. Otherwise the current position gets stuck when
				// an exception is thrown in extractDataFromSentence below.
				/* !!!Test */
				currExpectedSentenceType ++;
				/* */

				// Process the message
				extractDataFromSentence(newSentence);

				if (currExpectedSentenceType == usedNMEASentenceTypes.cend()) {
					// I received all required sentences.
					// Check if all required data are present, and if so pass them on to the Kalman filter
					currGnssRecord.recordProcessed = true;
					updateKalmanFilter (/*endOfCycle*/true);
				}
			} else {
				// Process the message
				extractDataFromSentence(newSentence);

				// In promiscuous mode check if all required data are present, and if so pass them on to the Kalman filter
				// Do this for each received message.
				updateKalmanFilter (/*endOfCycle*/false);
			}
		}

	}
	catch (NMEASetParseException const& e) {
		LOG4CXX_WARN(logger,"processSentenceTeachIn: Sentence " << newSentence.sentenceTypeString << " failed. Reason: " << e.what());
		return;
	}

}

void NMEASet::extractDataFromSentence(NMEASentence const& newSentence) {

	if (currGnssRecord.recordProcessed) {
		LOG4CXX_WARN(logger,"extractDataFromSentence called for a processed GNSS record again.! Sentence type = " << newSentence.sentenceType);
		return;
	}

	LOG4CXX_TRACE(logger,"extractDataFromSentence called with a " << newSentence.sentenceType << " sentence.");
	switch (newSentence.sentenceType) {
	case NMEASentence::NMEA_RMC:
		LOG4CXX_TRACE(logger,"Extract Longitude \"" << newSentence.fields[RMC_LON] << newSentence.fields[RMC_EW]
							<< "\", Latitude \"" << newSentence.fields[RMC_LAT] << newSentence.fields[RMC_NS]
							<< "\", Status '" << newSentence.fields[RMC_STATUS]
							<< "', Mode indicator (FAA Indicator) '" << newSentence.fields[RMC_POS_MODE] << '\'');

		// Check the FAA indicator and the status.
		// Valid values for the FAA indicator are A (Autonomous), D (Differential mode), R (RTK Mode), F (RTK Floar) or P (Precise).
		// OR, when the FAA indicator is empty: Valid value for the Status is A (no idea what that means)
		if ((newSentence.fields[RMC_STATUS][0] == 'A'
				&& newSentence.fields[RMC_POS_MODE][0] == 0)	// The Nav status (FAA indicator) is defined for NMEA 2.3 and higher.
																// If it is empty only look at the status
					|| ( (newSentence.fields[RMC_POS_MODE][0] == 'A' || newSentence.fields[RMC_POS_MODE][0] == 'D'
						|| newSentence.fields[RMC_POS_MODE][0] == 'P'
						|| newSentence.fields[RMC_POS_MODE][0] == 'R' || newSentence.fields[RMC_POS_MODE][0] == 'F'
						)) ) {

			extractCoordinatesFromSentence(
					newSentence,RMC_LON,RMC_EW,RMC_LAT,RMC_NS);

		} else {
			// Status is invalid
			LOG4CXX_DEBUG(logger,"Status or Nav-Status invalid. Cancel this entire cycle");

			// Mark the current measurement record as processed to indicate that this cycle is shot, and prevent wasting
			// time and cycles for processing more messages for this cycle.
			currGnssRecord.recordProcessed = true;
		}

		break;

	case NMEASentence::NMEA_GGA:
		LOG4CXX_TRACE(logger,"Extract Longitude \"" << newSentence.fields[GGA_LON] << newSentence.fields[GGA_EW]
							<< "\", Latitude \"" << newSentence.fields[GGA_LAT] << newSentence.fields[GGA_NS]
							<< "\", Quality Indicator '" << newSentence.fields[GGA_QUALITY]
							<< "', hDoP \"" << newSentence.fields[GGA_HDOP]
							<< "\", Alt MSL \"" << newSentence.fields[GGA_ALT] << newSentence.fields[GGA_ALT_UNIT]
							<< "\", Number of satellites in use \"" << newSentence.fields[GGA_NUM_SV] << "\"");

		// Check the Quality indicator.
		// Valid values 1 (Fix (unspecified)), 2 (Differential mode), 3 (PPS, precise),
		// 4 (RTK), 5 (Float RTK)
		if (*newSentence.fields[GGA_QUALITY] >= '1' && *newSentence.fields[GGA_QUALITY] <= '5' ) {

			extractCoordinatesFromSentence(
					newSentence,GGA_LON,GGA_EW,GGA_LAT,GGA_NS);

			extractAltMSLFromSentence(newSentence,GGA_ALT,GGA_ALT_UNIT);

			extractHDoPFromSentence(newSentence,GGA_HDOP);

		} else {
			// Status is invalid
			LOG4CXX_DEBUG(logger,"Quality indicator invalid. Cancel this entire cycle");

			// Mark the current measurement record as processed to indicate that this cycle is shot, and prevent wasting
			// time and cycles for processing more messages for this cycle.
			currGnssRecord.recordProcessed = true;
		}
		break;

	case NMEASentence::NMEA_GLL:
		LOG4CXX_TRACE(logger,"Extract Longitude \"" << newSentence.fields[GLL_LON] << newSentence.fields[GLL_EW]
							<< "\", Latitude \"" << newSentence.fields[GLL_LAT] << newSentence.fields[GLL_NS]
							<< "\", Status '" << newSentence.fields[GLL_STATUS]
							<< "', Nav Status (FAA Indicator) '" << newSentence.fields[GLL_POS_MODE] << '\'');

		// Check the FAA indicator and the status.
		// Valid values for the FAA indicator are A (Autonomous), D (Differential mode), R (RTK Mode), F (RTK Float) or P (Precise).
		// OR, when the FAA indicator is empty: Valid value for the Status is A (no idea what that means)
		if ((newSentence.fields[GLL_STATUS][0] == 'A'
				&& newSentence.fields[GLL_POS_MODE][0] == 0)	// The Nav status (FAA indicator) is defined for NMEA 2.3 and higher.
																// If it is empty only look at the status
					|| ( (newSentence.fields[GLL_POS_MODE][0] == 'A' || newSentence.fields[GLL_POS_MODE][0] == 'D'
						|| newSentence.fields[GLL_POS_MODE][0] == 'P'
						|| newSentence.fields[GLL_POS_MODE][0] == 'R' || newSentence.fields[GLL_POS_MODE][0] == 'F'
						)) ) {

			extractCoordinatesFromSentence(
					newSentence,GLL_LON,GLL_EW,GLL_LAT,GLL_NS);

		} else {
			// Status is invalid
			LOG4CXX_DEBUG(logger,"Status or Nav-Status invalid. Cancel this entire cycle");

			// Mark the current measurement record as processed to indicate that this cycle is shot, and prevent wasting
			// time and cycles for processing more messages for this cycle.
			currGnssRecord.recordProcessed = true;
		}

		break;

	case NMEASentence::NMEA_GNS:
		LOG4CXX_TRACE(logger,"Extract Longitude \"" << newSentence.fields[GNS_LON] << newSentence.fields[GNS_EW]
							<< "\", Latitude \"" << newSentence.fields[GNS_LAT] << newSentence.fields[GNS_NS]
							<< "', Nav Mode (by GNSS system) \"" << newSentence.fields[GNS_POS_MODE] << '"');

		{
			bool validStatus = false;

					// The mode indicator is here a string of multiple characters, each representing
			// the status of one GNSS system: GPS, GLONASS, Galileo, Baidu...
			// If any of these has the values A, D, P, R, or F the measurement is deemed valid.
			for (auto i = newSentence.fields[GNS_POS_MODE];*i != 0; i++) {
				if ( *i == 'A' || *i == 'D' || *i == 'P' || *i == 'R' || *i == 'F')  {
					validStatus = true;
					break;
				}
			}

			if (validStatus) {

				extractAltMSLFromSentence(newSentence,GNS_ALT,-1);
				extractCoordinatesFromSentence(newSentence,GNS_LON,GNS_EW,GNS_LAT,GNS_NS);
				extractHDoPFromSentence(newSentence,GNS_HDOP);

			} else {
				// Status is invalid
				LOG4CXX_DEBUG(logger,"Quality indicator invalid. Cancel this entire cycle");

				// Mark the current measurement record as processed to indicate that this cycle is shot, and prevent wasting
				// time and cycles for processing more messages for this cycle.
				currGnssRecord.recordProcessed = true;
			}
		}
		break;

	case NMEASentence::NMEA_GST:
		LOG4CXX_TRACE(logger,"Extract Std.Dev. Latitude \"" << newSentence.fields[GST_STD_LAT]
						<< "\", Std.Dev. Longitude \"" << newSentence.fields[GST_STD_LON]
						<< "\", Std.Dev. Altitude \"" << newSentence.fields[GST_STD_ALT] << "\""
						);

		extractDeviationsFromSentence(newSentence,GST_STD_LAT,GST_STD_LON,GST_STD_ALT);
		break;

	case NMEASentence::NMEA_GSA:
		LOG4CXX_TRACE(logger,"Extract hDoP \"" << newSentence.fields[GSA_HDOP]
						<< "\", vDoP \"" << newSentence.fields[GSA_VDOP]
						<< "\", pDoP \"" << newSentence.fields[GSA_PDOP]
						<< "\", Nav Mode \"" << newSentence.fields[GSA_NAV_MODE]
						<< "\", Operation mode \"" << newSentence.fields[GSA_OP_MODE]
						<< "\", System ID \"" << newSentence.fields[GSA_SYSTEM_ID] << "\""
		);

		if (*newSentence.fields[GSA_NAV_MODE] == '3') { // I am accepting only 3D-Mode
			extractVDoPFromSentence(newSentence,GSA_VDOP);
			extractHDoPFromSentence(newSentence,GSA_HDOP);
			extractPDoPFromSentence(newSentence,GSA_PDOP);
		} else {
			// Status is invalid
			LOG4CXX_DEBUG(logger,"Navigation mode is not 3D. Cancel this entire cycle");

			// Mark the current measurement record as processed to indicate that this cycle is shot, and prevent wasting
			// time and cycles for processing more messages for this cycle.
			currGnssRecord.recordProcessed = true;
		}


		break;

	case NMEASentence::NMEA_GBS:
		LOG4CXX_TRACE(logger,"Extract Std.Dev. Latitude \"" << newSentence.fields[GBS_ERR_LAT]
						<< "\", Std.Dev. Longitude \"" << newSentence.fields[GBS_ERR_LON]
						<< "\", Std.Dev. Altitude \"" << newSentence.fields[GBS_ERR_ALT] << "\""
						);

		extractDeviationsFromSentence(newSentence,GBS_ERR_LAT,GBS_ERR_LON,GBS_ERR_ALT);

		break;

	default:
		// Not recognized. Skip the message
		LOG4CXX_DEBUG(logger,"Not supported message type. Skipped.");

		break;

	}

}

void NMEASet::extractCoordinatesFromSentence(
		NMEASentence const& newSentence,
		int lonIndex,int ewIndex,int latIndex, int nsIndex) {

	// Extract coordinates when they have not yet been extracted,
	// and when longitude and latitude are not empty in the sentence.
	if (!currGnssRecord.posDefined
			&& *newSentence.fields[lonIndex] != 0
			&& *newSentence.fields[latIndex] != 0) {
		double longitude;
		double latitude;
		// Data seem valid. Go for it.
		switch (*newSentence.fields[ewIndex]) {
		case 'E':
			longitude = 1.0;
			break;

		case 'W':
			longitude = -1.0;
			break;

		default:
			std::ostringstream s;
			s << "extractCoordinatesFromSentence: Invalid East-West indicator in " << newSentence.sentenceTypeString << " sentence = " << newSentence.fields[ewIndex];
			throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
		}
		longitude *= nmeaCoordToD(newSentence.fields[lonIndex]);

		switch (*newSentence.fields[nsIndex]) {
		case 'N':
			latitude = 1.0;
			break;

		case 'S':
			latitude = -1.0;
			break;

		default:
			std::ostringstream s;
			s << "extractCoordinatesFromSentence: Invalid North-South indicator in " << newSentence.sentenceTypeString << " sentence = " << newSentence.fields[nsIndex];
			throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
		}
		latitude *= nmeaCoordToD(newSentence.fields[latIndex]);

		LOG4CXX_TRACE (logger," Longitude = " << longitude << ", latitude = "<< latitude);
		currGnssRecord.longitude = longitude;
		currGnssRecord.latitude = latitude;
		currGnssRecord.posDefined = true;
	} // if (!currGnssRecord.posDefined)
}

void NMEASet::extractAltMSLFromSentence(NMEASentence const& newSentence,int altMSLIndex,int altMSLUoMIndex) {


	if (!currGnssRecord.altMslDefined
			&& *newSentence.fields[altMSLIndex] != 0) {

		// Some sentences like GNS do not carry the UoM because it is *always* in meters.
		if (altMSLUoMIndex > 0 &&
				*newSentence.fields[altMSLUoMIndex] != 'M') {
			LOG4CXX_TRACE (logger,"extractAltMSLFromSentence: Unit of altitude is not 'M' but " << newSentence.fields[altMSLUoMIndex]);
			return;
		}
		currGnssRecord.altMSL = strToD(newSentence.fields[altMSLIndex]);
		currGnssRecord.altMslDefined = true;
		LOG4CXX_TRACE (logger, "extractAltMSLFromSentence: altMSL = " << currGnssRecord.altMSL);
	}

}

void NMEASet::extractHDoPFromSentence(NMEASentence const& newSentence,int hDoPIndex) {

	// Direct definition of deviations overrides DoP values
	if (!currGnssRecord.devDirectDefined && !currGnssRecord.hDoPDefined
			&& *newSentence.fields[hDoPIndex] != 0) {
		double hDoP = strToD(newSentence.fields[hDoPIndex]);

		currGnssRecord.lonDeviation = currGnssRecord.latDeviation = gpsDriver.getCEP() * hDoP;
		currGnssRecord.hDoPDefined = true;
		LOG4CXX_TRACE (logger, "extractHDoPFromSentence: hDop = " << hDoP << ", Lat/Long-Deviation = " << currGnssRecord.lonDeviation);


		if (!currGnssRecord.vDoPDefined && ! currGnssRecord.pDoPDefined) {
			// Estimate the vDoP as ~1.7 hDoP
			// Thus estimate the vertical deviation
			currGnssRecord.altDeviation = gpsDriver.getAltStdDev() * hDoP * 1.7;
			LOG4CXX_TRACE (logger, "	Estimated vDop = " << hDoP * 1.7 << ", Altitude Deviation = " << currGnssRecord.altDeviation);
		}

	}

}

void NMEASet::extractVDoPFromSentence(NMEASentence const& newSentence,int vDoPIndex) {

	if (!currGnssRecord.devDirectDefined && !currGnssRecord.vDoPDefined
			&& *newSentence.fields[vDoPIndex] != 0) {
		double vDoP = strToD(newSentence.fields[vDoPIndex]);

		currGnssRecord.altDeviation = gpsDriver.getAltStdDev() * vDoP;
		currGnssRecord.vDoPDefined = true;
		LOG4CXX_TRACE (logger, "extractVDoPFromSentence: vDop = " << vDoP << ", Altitude Deviation = " << currGnssRecord.altDeviation);

	}

}

void NMEASet::extractPDoPFromSentence(NMEASentence const& newSentence,int pDoPIndex) {

	if (!currGnssRecord.devDirectDefined && !currGnssRecord.pDoPDefined
			&& *newSentence.fields[pDoPIndex] != 0) {

		// Retrieve pDoP only when it really makes sense, and calculate hDoP and vDoP on demand only.
		if (!currGnssRecord.hDoPDefined) {
			double pDoP = strToD(newSentence.fields[pDoPIndex]);
			// Use the float variant for speed.
			double hDoP = sqrtf(pDoP*pDoP / (3.89 /*1 + 1.7*1.7*/) );

			currGnssRecord.lonDeviation = currGnssRecord.latDeviation = gpsDriver.getCEP() * hDoP;
			LOG4CXX_TRACE (logger, "extractPDoPFromSentence: pDop = " << pDoP << ", calculated hDoP = " << hDoP
					<<", Lon/Lat Deviation = " << currGnssRecord.latDeviation);

			// Continue with the calculation of vDoP here because thedirect calculation from pDoP
			// would be as expensive as the calculation of hDoP above, whereas I can calculate
			// vDoP easly from hDoP
			if (!currGnssRecord.vDoPDefined) {
				double vDoP = hDoP * 1.7;

				currGnssRecord.altDeviation = gpsDriver.getAltStdDev() * vDoP;
				LOG4CXX_TRACE (logger, "extractPDoPFromSentence: Calculated vDoP = " << vDoP
						<< ", Altitude Deviation = " << currGnssRecord.altDeviation);

			} // if (!currGnssRecord.vDoPDefined)

		} else { // if (!currGnssRecord.hDoPDefined)
			if (!currGnssRecord.vDoPDefined) {
				double pDoP = strToD(newSentence.fields[pDoPIndex]);
				// Use the float variant for speed.
				double vDoP = sqrtf(pDoP*pDoP / (3.89 /* 1 + 1.7*1.7 */) ) * 1.7;

				currGnssRecord.altDeviation = gpsDriver.getAltStdDev() * vDoP;
				LOG4CXX_TRACE (logger, "extractPDoPFromSentence: pDop = " << pDoP << ", calculated vDoP = " << vDoP
						<< ", Altitude Deviation = " << currGnssRecord.altDeviation);

			} // if (!currGnssRecord.vDoPDefined)

		} // if (!currGnssRecord.hDoPDefined)

		// Put this line at the bottom. If an exception is thrown during conversion the flag is not being set.
		currGnssRecord.pDoPDefined = true;

	} //	if (!currGnssRecord.devDirectDefined && !currGnssRecord.pDoPDefined
	  //		&& *newSentence.fields[pDoPIndex] != 0)

}

void NMEASet::extractDeviationsFromSentence(NMEASentence const& newSentence,int latDevIndex, int lonDevIndex, int altDevIndex) {

	if (!currGnssRecord.devDirectDefined && !currGnssRecord.pDoPDefined) {
		double latDeviation = 0.0;
		double lonDeviation = 0.0;
		double altDeviation = 0.0;

		if (*newSentence.fields[latDevIndex] != 0) {
			latDeviation = strToD(newSentence.fields[latDevIndex]);
			currGnssRecord.devDirectDefined = true;
		}

		if (*newSentence.fields[lonDevIndex] != 0) {
			lonDeviation = strToD(newSentence.fields[lonDevIndex]);
			currGnssRecord.devDirectDefined = true;
		}

		if (*newSentence.fields[altDevIndex] != 0) {
			altDeviation = strToD(newSentence.fields[altDevIndex]);
			currGnssRecord.devDirectDefined = true;
		}

		// No Exception? Then move on.
		currGnssRecord.latDeviation = latDeviation;
		currGnssRecord.lonDeviation = lonDeviation;
		currGnssRecord.altDeviation = altDeviation;

		LOG4CXX_TRACE(logger,"extractDeviationsFromSentence: latDeviation = " << currGnssRecord.latDeviation
				<< ", lonDeviation = " << currGnssRecord.lonDeviation
				<< ", altDeviation = " << currGnssRecord.altDeviation
				);
	}

}

#define SQUARE(x) ((x)*(x))

void NMEASet::updateKalmanFilter (bool endOfCycle) {

	LOG4CXX_DEBUG(logger,"updateKalmanFilter with endOfCycle = " << endOfCycle << " and:");
	if (currGnssRecord.altMslDefined && (currGnssRecord.vDoPDefined || currGnssRecord.pDoPDefined || currGnssRecord.devDirectDefined)) {
		LOG4CXX_DEBUG(logger,"	Alt MSL = "  << currGnssRecord.altMSL << ", altitude deviation = " << currGnssRecord.altDeviation);
	}
	if (currGnssRecord.posDefined && (currGnssRecord.hDoPDefined || currGnssRecord.pDoPDefined || currGnssRecord.devDirectDefined)) {
		LOG4CXX_DEBUG(logger,"	Latitude = "  << currGnssRecord.latitude << ", latitude deviation = " << currGnssRecord.latDeviation);
		LOG4CXX_DEBUG(logger,"	Longitude = "  << currGnssRecord.longitude << ", longitude deviation = " << currGnssRecord.lonDeviation);
	}

	GliderVarioMainPriv::LockedCurrentStatus currStat (*varioMain );

	if (currGnssRecord.posDefined &&
			currGnssRecord.latDeviation > 0.0 && currGnssRecord.lonDeviation > 0.0 &&
			(currGnssRecord.devDirectDefined || currGnssRecord.hDoPDefined|| // Rather specific deviation information
					(currGnssRecord.pDoPDefined && endOfCycle))) {	// Just minimal deviation information,
																	// but I will not get anything better in this cycle
		if (initialPositionSet ) {
			updatePosition(endOfCycle,currStat);
		} else { // if (initialPositionSet )
			initializePosition(endOfCycle,currStat);
		} // if (initialPositionSet )
	} // if (currGnssRecord.posDefined && ...

	if (currGnssRecord.altMslDefined &&
			currGnssRecord.altDeviation > 0.0 &&
			(currGnssRecord.devDirectDefined || currGnssRecord.vDoPDefined || // Rather specific deviation information
					(currGnssRecord.pDoPDefined && endOfCycle))) {	// Just minimal deviation information,
																	// but I will not get anything better in this cycle
		if (initialAltitudeSet) {
			updateAltitude(endOfCycle,currStat);
		} else { // if (initialAltitudeSet)
			initializeAltitude(endOfCycle,currStat);
		} // if (initialAltitudeSet)
	} // if (currGnssRecord.altMslDefined && ...
}

void NMEASet::initializePosition (
		bool endOfCycle,
		GliderVarioMainPriv::LockedCurrentStatus &currStat
	) {

	GliderVarioStatus& currVarioStatus = *currStat.getCurrentStatus();

	// Use the position only when the precision is sufficient.
	// Else wait until the position is sufficiently precisely determined.
	if (currGnssRecord.latDeviation < gpsDriver.getMaxStdDeviationPositionInitialization() &&
			currGnssRecord.lonDeviation < gpsDriver.getMaxStdDeviationPositionInitialization()) {
		GliderVarioStatus::StatusCoVarianceType &errorCov = currVarioStatus.getErrorCovariance_P();

		// Set latitude and initial error variance unconditionally
		currVarioStatus.latitude(currGnssRecord.latitude);
		errorCov.coeffRef(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_LATITUDE_OFFS)
				= SQUARE(currGnssRecord.latDeviation) * 10.0;

		// Set longitude and initial error variance unconditionally
		currVarioStatus.longitude(currGnssRecord.longitude);
		errorCov.coeffRef(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS)
				= SQUARE(currGnssRecord.lonDeviation)  * 10.0;

		// Initialize the ground speed. Assume you are starting the thing on the ground
		// For cases when starting in the air make the error large enough to quickly adjust.
		/// \todo Read out the speed and direction from the GPS device too, and initialize the
		/// values accordingly instead naively to 0.
		if (UnInitVal == currStat->groundSpeedNorth) {
			currStat->groundSpeedNorth = 0.0f;
		}
		if (UnInitVal == errorCov.coeffRef(currStat->STATUS_IND_SPEED_GROUND_N,currStat->STATUS_IND_SPEED_GROUND_N)) {
			errorCov.coeffRef(currStat->STATUS_IND_SPEED_GROUND_N,currStat->STATUS_IND_SPEED_GROUND_N) = 1000.0f;
		}

		if (UnInitVal == currStat->groundSpeedEast) {
			currStat->groundSpeedEast = 0.0f;
		}
		if (UnInitVal == errorCov.coeffRef(currStat->STATUS_IND_SPEED_GROUND_E,currStat->STATUS_IND_SPEED_GROUND_E)) {
			errorCov.coeffRef(currStat->STATUS_IND_SPEED_GROUND_E,currStat->STATUS_IND_SPEED_GROUND_E) = 1000.0f;
		}

		// Initialize wind only with positional sensors.
		if (UnInitVal == currStat->windSpeedNorth) {
			currStat->windSpeedNorth = 0.0f;
		}
		if (UnInitVal == errorCov.coeffRef(currStat->STATUS_IND_WIND_SPEED_N,currStat->STATUS_IND_WIND_SPEED_N)) {
			errorCov.coeffRef(currStat->STATUS_IND_WIND_SPEED_N,currStat->STATUS_IND_WIND_SPEED_N) = 100.0f;
		}

		if (UnInitVal == currStat->windSpeedEast) {
			currStat->windSpeedEast = 0.0f;
		}
		if (UnInitVal == errorCov.coeffRef(currStat->STATUS_IND_WIND_SPEED_E,currStat->STATUS_IND_WIND_SPEED_E)) {
			errorCov.coeffRef(currStat->STATUS_IND_WIND_SPEED_E,currStat->STATUS_IND_WIND_SPEED_E) = 100.0f;
		}

		// Initialize the heading. If possible extract it directly from the GPS if supported.
		// When stationary direction accuracy is more or less random because it is driven by GPS noise, and
		// not by real positional changes.
		if (UnInitVal == currStat->heading) {
			currStat->heading = 45.0f;
		}
		if (UnInitVal == errorCov.coeffRef(currStat->STATUS_IND_HEADING,currStat->STATUS_IND_HEADING)) {
			errorCov.coeffRef(currStat->STATUS_IND_HEADING,currStat->STATUS_IND_HEADING) = 10.0f;
		}

		LOG4CXX_DEBUG (logger,"Initialize latitude and longitude.");
		LOG4CXX_DEBUG (logger,"	latitude = " << currVarioStatus.latitude()
				<< ", initial variance = "
				<< errorCov.coeff(GliderVarioStatus::STATUS_IND_LATITUDE_OFFS,GliderVarioStatus::STATUS_IND_LATITUDE_OFFS)
				<< ", variance increment = ");
		LOG4CXX_DEBUG (logger,"	longitude = " << currVarioStatus.longitude()
				<< ", initial variance = "
				<< errorCov.coeff(GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS,GliderVarioStatus::STATUS_IND_LONGITUDE_OFFS)
				<< ", variance increment = ");

		initialPositionSet = true;
	}

}

void NMEASet::updatePosition (
		bool endOfCycle,
		GliderVarioMainPriv::LockedCurrentStatus &currStat
	) {

	GliderVarioStatus& currVarioStatus = *currStat.getCurrentStatus();

	if (currGnssRecord.latDeviation < gpsDriver.getMaxStdDeviationPositionUpdate() &&
			currGnssRecord.lonDeviation < gpsDriver.getMaxStdDeviationPositionUpdate()) {
		LOG4CXX_DEBUG (logger,"Update longitude and latitude.");

		// Assess the additional uncertainty due to turning.
		// Use the amount of predicted veering off a straight course at the current speed and current rate of turn
		// within the next second.

		FloatType addDeviation = 0.0f;

		if (currVarioStatus.yawRateZ != 0.0f) {
			// turn radius calculated by speed (m/s) * time for a full circle / 2Pi
			FloatType turnRadius = currVarioStatus.trueAirSpeed *
					360.0f / currVarioStatus.yawRateZ
					/ (2.0f * float(M_PI));

			addDeviation = fabsf(turnRadius * FastMath::fastSin(currVarioStatus.yawRateZ));
			LOG4CXX_DEBUG (logger,"	TrueAirSpeed = " << currVarioStatus.trueAirSpeed
					<< ", turn rate (deg/s) = " << currVarioStatus.yawRateZ
					<< ", turnRadius = " << turnRadius
					<< " addDeviation by turning = " << addDeviation
					);
		}

		addDeviation += fabsf(currVarioStatus.accelHeading);
		LOG4CXX_DEBUG (logger,"	addDeviation incl. accel = " << addDeviation);

		LOG4CXX_DEBUG(logger,"	Latitude = " << currGnssRecord.latitude
				<< ", variance = " << SQUARE(currGnssRecord.latDeviation + addDeviation));
		LOG4CXX_DEBUG(logger,"	Longitude = " << currGnssRecord.longitude
				<< ", variance = " << SQUARE(currGnssRecord.latDeviation + addDeviation));

		GliderVarioMeasurementUpdater::GPSPositionUpd(
				currGnssRecord.latitude,
				currGnssRecord.longitude,
				SQUARE(currGnssRecord.latDeviation + addDeviation),
				SQUARE(currGnssRecord.lonDeviation + addDeviation),
				*currStat.getMeasurementVector(),
				*currStat.getCurrentStatus());

		currStat.getCurrentStatus()->normalizeStatus();
	}

}

void NMEASet::initializeAltitude (
		bool endOfCycle,
		GliderVarioMainPriv::LockedCurrentStatus &currStat
	) {

	GliderVarioStatus& currVarioStatus = *currStat.getCurrentStatus();

	if (currGnssRecord.altDeviation < gpsDriver.getMaxStdDeviationAltitudeInitialization()) {
		LOG4CXX_DEBUG (logger,"Initialize altitude.");
		GliderVarioStatus::StatusCoVarianceType &systemNoiseCov = currVarioStatus.getSystemNoiseCovariance_Q();
		GliderVarioStatus::StatusCoVarianceType &errorCov = currVarioStatus.getErrorCovariance_P();
		double baseIntervalSec = varioMain->getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

		currVarioStatus.altMSL = currGnssRecord.altMSL;
		errorCov.coeffRef(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL)
				= SQUARE(currGnssRecord.altDeviation) * 20.0;
		systemNoiseCov.coeffRef(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL) =
				SQUARE(4.0) * baseIntervalSec;

		LOG4CXX_DEBUG (logger,"	altitude = " << currVarioStatus.altMSL
				<< ", initial variance = "
				<< errorCov.coeff(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL)
				<< ", variance increment = "
				<< systemNoiseCov.coeff(GliderVarioStatus::STATUS_IND_ALT_MSL,GliderVarioStatus::STATUS_IND_ALT_MSL)
				<< " / " << baseIntervalSec << "s");
		if (!std::isnan(currStat.getMeasurementVector()->staticPressure)) {
			initQFF(currStat);
		}

		initialAltitudeSet = true;
	}

}

void NMEASet::updateAltitude (
		bool endOfCycle,
		GliderVarioMainPriv::LockedCurrentStatus &currStat
	) {

	GliderVarioStatus& currVarioStatus = *currStat.getCurrentStatus();

	if (currGnssRecord.altDeviation < gpsDriver.getMaxStdDeviationAltitudeUpdate()) {
		LOG4CXX_DEBUG (logger,"Update altitude.");

		// Assess the additional uncertainty due to acceleration in thermals and turbulence.
		// Use acceleration based on one second prediction time
		// to assess the additional uncertainty.
		FloatType addDeviation = fabsf(currVarioStatus.accelVertical);
		FloatType altVariance = SQUARE(currGnssRecord.altDeviation + addDeviation);

		LOG4CXX_DEBUG (logger,"	addDeviation = " << addDeviation);

		LOG4CXX_DEBUG(logger,"	Altitude = " << currGnssRecord.altMSL
				<< ", variance = " << altVariance);

		GliderVarioMeasurementUpdater::GPSAltitudeUpd(currGnssRecord.altMSL,
				altVariance,
				*currStat.getMeasurementVector(),
				*currStat.getCurrentStatus());

	} // if (currGnssRecord.altDeviation < gpsDriver.getMaxStdDeviationAltitudeUpdate())
}

void NMEASet::initQFF(GliderVarioMainPriv::LockedCurrentStatus &currStat) {
	GliderVarioStatus& currVarioStatus = *currStat.getCurrentStatus();
	GliderVarioStatus::StatusCoVarianceType &systemNoiseCov = currVarioStatus.getSystemNoiseCovariance_Q();
	GliderVarioStatus::StatusCoVarianceType &errorCov = currVarioStatus.getErrorCovariance_P();
	GliderVarioMeasurementVector& measurementVector = *currStat.getMeasurementVector();
	double baseIntervalSec = varioMain->getProgramOptions().idlePredictionCycleMilliSec / 1000.0;

	FloatType pressureFactor = calcBarometricFactor(
    		currVarioStatus.altMSL,
			measurementVector.tempLocalC
			);
	LOG4CXX_DEBUG (logger,"NMEASet::initQFF pressureFactor = " << pressureFactor);

	currVarioStatus.qff = measurementVector.staticPressure / pressureFactor;

	// Assume the same variance of qff pressure as the initial altitude variance
	errorCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) = 1.0f;
	systemNoiseCov.coeffRef(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) =
			SQUARE(0.0001) * baseIntervalSec;

	LOG4CXX_DEBUG (logger,"	QFF = " << currVarioStatus.qff
			<< ", initial variance = "
			<< errorCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF)
			<< ", variance increment = "
			<< systemNoiseCov.coeff(GliderVarioStatus::STATUS_IND_QFF,GliderVarioStatus::STATUS_IND_QFF) / baseIntervalSec
			<< " / " << "s");


}

} /* namespace openEV::drivers::NMEA0813 */
