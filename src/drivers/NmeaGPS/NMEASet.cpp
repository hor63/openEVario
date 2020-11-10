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
#  include <config.h>
#endif

#include <sstream>

#include "NMEA0813Protocol.h"
#include "NMEASet.h"
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
		if (*lStr == '-') sign = -1.0;
	}
	for (; isDigit(*lStr);lStr ++) {
		rc = rc * 10.0 + double(*lStr - '0');
	}

	if (*lStr == '.') {
		// Eat up the decimal separator
		lStr ++;
	} else {
		std::ostringstream s;
		s << "strToD: String \"" << str << "\" is not a valid double value";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	// now come the decimal places when there are any.
	for (; isDigit(*lStr);lStr ++) {
		factor /= 10.0;
		rc += double(*lStr - '0') * factor;
		lStr ++;
	}

	// Check if I reached the end of the string. Else there is invalid trailing stuff
	// This is not allowed for this function.
	if (lStr != 0) {
		std::ostringstream s;
		s << "strToD: String \"" << str << "\" is not a valid double value";
		LOG4CXX_WARN(logger,s.str());
		throw NMEASetParseException(__FILE__,__LINE__,s.str().c_str());
	}

	LOG4CXX_DEBUG (logger,"strToD: String \"" << str << "\" converted to " << (rc * sign));
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
	uint32_t i;
	uint8_t const *lStr;

	// Look for the decimal.
	for (lStr = str;*lStr != 0; lStr ++) {
		if (*str == '.') break;
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

	LOG4CXX_DEBUG(logger,"nmeaCoordToD: \"" << str << "\" converted to " << degrees << " Degrees, " << minutes << " minutes = "
			<< " = " << (degrees + minutes / 60.0));

	return degrees + minutes / 60.0;
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

	LOG4CXX_DEBUG(logger,"NMEASet::NMEATimeStampToMS: \"" << timestampStr << "\" converted to " << rc);

	return rc;
}

void NMEASet::processSentenceTeachIn(
		const NMEASentence &newSentence) {

	auto currTime = std::chrono::system_clock::now();

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
			LOG4CXX_DEBUG(logger,"processSentenceTeachIn: Throw out un-supported sentenct with type " << newSentence.sentenceTypeString);
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
					processSentenceFunction = &processSentenceOperation;

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
				LOG4CXX_DEBUG(logger,"processSentenceTeachIn: Last timestamp = " << currSequenceTimestampMS
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

	LOG4CXX_DEBUG(logger,"processSentenceOperation: Message " << newSentence.talkerID << ' ' << newSentence.sentenceTypeString
			<< ", internal type" << newSentence.sentenceType);

	LOG4CXX_DEBUG(logger,"processSentenceTeachIn: Process message type " << newSentence.sentenceTypeString);

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
			LOG4CXX_DEBUG(logger, "getNewSentenceTimestampMS: Timestamp string = " << timestampString);
			thisGPSTimeStampMS = NMEATimeStampToMS(timestampString);
		}
	}

	LOG4CXX_DEBUG(logger, "getNewSentenceTimestampMS: Return = " << thisGPSTimeStampMS);
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
			LOG4CXX_DEBUG(logger,"Start new cycle. Old GNSS timestamp = " << currGnssRecord.gnssTimeStamp
					<< ", new GNSS timestamp = " << thisGPSTimeStampMS);

			currExpectedSentenceType = usedNMEASentenceTypes.cbegin();
			currGnssRecord.initialize();
			currGnssRecord.gnssTimeStamp = thisGPSTimeStampMS;
			currGnssRecord.recordStart = std::chrono::system_clock::now();
		}

		// Check if the current cycle was complete and use to update the Kalman filter already
		// and if the sentence type is the one which is currently expected,
		// or if the expected type list is empty, and we are in promiscuous mode
		if (!currGnssRecord.recordProcessed &&
				(usedNMEASentenceTypes.size() == 0 ||
				 (currExpectedSentenceType != usedNMEASentenceTypes.cend() &&
				  *currExpectedSentenceType != newSentence.sentenceType))) {
			// Check if you got all required messages in targeted mode
			if (usedNMEASentenceTypes.size() > 0) {
				// Advance to the next expected sentence type
				// Do that before processing. Otherwise the current position gets stuck when
				// an exception is thrown in extractDataFromSentence below.
				currExpectedSentenceType ++;

				// Process the message
				extractDataFromSentence(newSentence);

				if (currExpectedSentenceType == usedNMEASentenceTypes.cend()) {
					// I received all required sentences.
					// Check if all required data are present, and if so pass them on to the Kalman filter
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

	LOG4CXX_DEBUG(logger,"extractDataFromSentence called with a " << newSentence.sentenceType << " sentence.");
	switch (newSentence.sentenceType) {
	case NMEASentence::NMEA_RMC:
		LOG4CXX_DEBUG(logger,"Extract Longitude \"" << newSentence.fields[RMC_LON] << newSentence.fields[RMC_EW]
							<< "\", Latitude \"" << newSentence.fields[RMC_LAT] << newSentence.fields[RMC_NS]
							<< "\", Status '" << newSentence.fields[RMC_STATUS]
							<< "', Nav Status (FAA Indicator) '" << newSentence.fields[RMC_NAV_STATUS] << '\'');

		// Check the FAA indicator and the status.
		// Valid values for the FAA indicator are A (Autonomous), D (Differential mode), or P (Precise).
		// Valid value for the Status is A (no idea what that means)
		if (newSentence.fields[RMC_STATUS][0] == 'A'
				&& (newSentence.fields[RMC_NAV_STATUS][0] == 0	// The Nav status (FAA indicator) is defined for NMEA 2.3 and higher.
																// If it is empty only look at the status
					|| (newSentence.fields[RMC_NAV_STATUS][0] == 'A' || newSentence.fields[RMC_NAV_STATUS][0] == 'D'
						|| newSentence.fields[RMC_NAV_STATUS][0] == 'P')) ) {

			extractCoordinatesFromSentence(
					newSentence,RMC_LON,RMC_EW,RMC_LAT,RMC_NS);

		} else {
			// Status is invalid
			LOG4CXX_DEBUG(logger,"Status or Nav-Status invaid. Cancel this entire cycle");

			// Mark the current measurement record as processed to indicate that this cycle is shot, and prevent wasting
			// time and cycles for processing more messages for this cycle.
			currGnssRecord.recordProcessed = true;
		}

		break;

	case NMEASentence::NMEA_GGA:
		LOG4CXX_DEBUG(logger,"Extract Longitude \"" << newSentence.fields[GGA_LON] << newSentence.fields[GGA_EW]
							<< "\", Latitude \"" << newSentence.fields[GGA_LAT] << newSentence.fields[GGA_NS]
							<< "\", Quality Indicator '" << newSentence.fields[GGA_QUALITY]
							<< "', hDoP '" << newSentence.fields[GGA_HDOP]
							<< "\", Alt MSL \"" << newSentence.fields[GGA_ALT] << newSentence.fields[GGA_ALT_UNIT]
							<< ", Number of satellites in use " << newSentence.fields[GGA_NUM_SV]);

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
		break;

	case NMEASentence::NMEA_GNS:
		break;

	case NMEASentence::NMEA_GST:
		break;

	case NMEASentence::NMEA_GSA:
		break;

	case NMEASentence::NMEA_GBS:
		break;

	default:
		// Not recognized. Skip the message
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
		latitude *= nmeaCoordToD(newSentence.fields[lonIndex]);

		LOG4CXX_DEBUG (logger," Longitude = " << longitude << ", latitude = "<< latitude);
		currGnssRecord.longitude = longitude;
		currGnssRecord.latitude = latitude;
		currGnssRecord.posDefined = true;
	} // if (!currGnssRecord.posDefined)
}

void NMEASet::extractAltMSLFromSentence(NMEASentence const& newSentence,int altMSLIndex,int altMSLUoMIndex) {


	if (!currGnssRecord.altMslDefined
			&& *newSentence.fields[altMSLIndex] != 0) {

		if (*newSentence.fields[altMSLUoMIndex] != 'M') {
			LOG4CXX_DEBUG (logger,"extractAltMSLFromSentence: Unit of altitude is not 'M' but " << newSentence.fields[altMSLUoMIndex]);
			return;
		}
		currGnssRecord.altMSL = strToD(newSentence.fields[altMSLIndex]);
		LOG4CXX_DEBUG (logger, "extractAltMSLFromSentence: altMSL = " << currGnssRecord.altMSL);
	}

}

void NMEASet::extractHDoPFromSentence(NMEASentence const& newSentence,int hDoPIndex) {

	// Direct definition of deviations overrides DoP values
	if (!currGnssRecord.devDirectDefined && !currGnssRecord.hDoPDefined
			&& *newSentence.fields[hDoPIndex] != 0) {
		double hDoP = strToD(newSentence.fields[hDoPIndex]);

		currGnssRecord.longDeviation = currGnssRecord.latDeviation = gpsDriver.getCEP() * hDoP;
		currGnssRecord.hDoPDefined = true;
		LOG4CXX_DEBUG (logger, "extractHDoPFromSentence: hDop = " << hDoP << ", Lat/Long-Deviation = " << currGnssRecord.longDeviation);


		if (!currGnssRecord.vDoPDefined && ! currGnssRecord.pDoPDefined) {
			// Estimate the vDoP as ~1.7 hDoP
			// Thus estimate the vertical deviation
			currGnssRecord.altDeviation = gpsDriver.getAltStdDev() * hDoP * 1.7;
			LOG4CXX_DEBUG (logger, "	Estimated vDop = " << hDoP * 1.7 << ", Altitude Deviation = " << currGnssRecord.altDeviation);
		}

	}

}

void NMEASet::extractVDoPFromSentence(NMEASentence const& newSentence,int vDoPIndex) {

	if (!currGnssRecord.devDirectDefined && !currGnssRecord.vDoPDefined
			&& *newSentence.fields[vDoPIndex] != 0) {
		double vDoP = strToD(newSentence.fields[vDoPIndex]);

		currGnssRecord.altDeviation = gpsDriver.getAltStdDev() * vDoP;
		currGnssRecord.vDoPDefined = true;
		LOG4CXX_DEBUG (logger, "extractVDoPFromSentence: vDop = " << vDoP << ", Altitude Deviation = " << currGnssRecord.altDeviation);

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

			currGnssRecord.pDoPDefined = true;

			currGnssRecord.longDeviation = currGnssRecord.latDeviation = gpsDriver.getCEP() * hDoP;
			LOG4CXX_DEBUG (logger, "extractPDoPFromSentence: pDop = " << pDoP << ", calculated hDoP = " << hDoP
					<<", Lon/Lat Deviation = " << currGnssRecord.latDeviation);

			// Continue with the calculation of vDoP here because thedirect calculation from pDoP
			// would be as expensive as the calculation of hDoP above, whereas I can calculate
			// vDoP easly from hDoP
			if (!currGnssRecord.vDoPDefined) {
				double vDoP = hDoP * 1.7;

				currGnssRecord.altDeviation = gpsDriver.getAltStdDev() * vDoP;
				LOG4CXX_DEBUG (logger, "extractPDoPFromSentence: Calculated vDoP = " << vDoP
						<< ", Altitude Deviation = " << currGnssRecord.altDeviation);

			}

		} else {
			if (!currGnssRecord.vDoPDefined) {
				double pDoP = strToD(newSentence.fields[pDoPIndex]);
				// Use the float variant for speed.
				double vDoP = sqrtf(pDoP*pDoP / (3.89 /*1 + 1.7*1.7*/) ) * 1.7;

				currGnssRecord.altDeviation = gpsDriver.getAltStdDev() * vDoP;
				LOG4CXX_DEBUG (logger, "extractPDoPFromSentence: pDop = " << pDoP << ", calculated vDoP = " << vDoP
						<< ", Altitude Deviation = " << currGnssRecord.altDeviation);

			}

		}
	}

}

void NMEASet::extractDeviationsFromSentence(NMEASentence const& newSentence,int latDevIndex, int lonDevIndex, int altDevIndex) {

	if (!currGnssRecord.devDirectDefined && !currGnssRecord.pDoPDefined) {
		double latDeviation = 0.0;
		double lonDeviation = 0.0;
		double altDeviation = 0.0;

		if (*newSentence.fields[latDevIndex] != 0) {
			latDeviation = strToD(newSentence.fields[latDevIndex]);
		}

		if (*newSentence.fields[lonDevIndex] != 0) {
			lonDeviation = strToD(newSentence.fields[lonDevIndex]);
		}

		if (*newSentence.fields[altDevIndex] != 0) {
			altDeviation = strToD(newSentence.fields[altDevIndex]);
		}

		// No Exception? Then move on.
		currGnssRecord.latDeviation = latDevIndex;
		currGnssRecord.longDeviation = lonDevIndex;
		currGnssRecord.altDeviation = altDevIndex;
		currGnssRecord.devDirectDefined = true;
	}

}

void NMEASet::updateKalmanFilter (bool endOfCycle) {

}

} /* namespace openEV::drivers::NMEA0813 */
