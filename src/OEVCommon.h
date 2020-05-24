/*
 * OEVCommon.h
 *
 *  Created on: Sep 29, 2017
 *      Author: hor
 *
 *  Common definitions for building shared libraries
 *  and other helpers
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

#ifndef OEVCOMMON_H_
#define OEVCOMMON_H_

#if defined HAVE_CONFIG_H
#	include "config.h"
#endif

#include <ostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <unordered_map>
#include <stdlib.h>
#include <cstdlib>
#include <cstdint>
#include <chrono>

#if HAVE_TIME_H == 1
#	include <time.h>
#endif

#if HAVE_SYS_TIME_H == 1
#	include <sys/time.h>
#endif

#if defined HAVE_LOG4CXX_H
#	include <log4cxx/logger.h>
#	include "log4cxx/basicconfigurator.h"
#	include "log4cxx/propertyconfigurator.h"
#else /* #if defined HAVE_LOG4CXX_H */
// These are all the logger macros from log4cxx/logger.h. Here redefined empty when no logger is used.
#define LOG4CXX_LOG(logger, level, message)  do {;} while (0)
#define LOG4CXX_LOGLS(logger, level, message)  do {;} while (0)
#define LOG4CXX_DEBUG(logger, message)  do {;} while (0)
#define LOG4CXX_TRACE(logger, message)  do {;} while (0)
#define LOG4CXX_INFO(logger, message)  do {;} while (0)
#define LOG4CXX_WARN(logger, message)  do {;} while (0)
#define LOG4CXX_ERROR(logger, message)  do {;} while (0)
#define LOG4CXX_ASSERT(logger, condition, message)  do {;} while (0)
#define LOG4CXX_FATAL(logger, message)  do {;} while (0)
#define LOG4CXX_L7DLOG(logger, level, key)  do {;} while (0)
#define LOG4CXX_L7DLOG1(logger, level, key, p1)  do {;} while (0)
#define LOG4CXX_L7DLOG1(logger, level, key, p1)  do {;} while (0)
#define LOG4CXX_L7DLOG2(logger, level, key, p1, p2)  do {;} while (0)
#define LOG4CXX_L7DLOG3(logger, level, key, p1, p2, p3)  do {;} while (0)
#endif /* #if defined HAVE_LOG4CXX_H */

/**
 * Define OV_DLL_IMPORT, OV_DLL_EXPORT, and OV_DLL_LOCAL for Windows and Linux (ELF) ports of gcc and non-gcc compilers
 *
 * The macro definitions are highly inspired from the <a href="https://gcc.gnu.org/wiki/Visibility">GCC Wiki: Visibility</a>
 */
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
      #define OV_DLL_EXPORT __attribute__ ((dllexport))
      #define OV_DLL_IMPORT __attribute__ ((dllimport))
    #else
      #define OV_DLL_EXPORT __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
      #define OV_DLL_IMPORT __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
    #endif
    #ifdef __GNUC__
    #else
    #endif
  #define OV_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define OV_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define OV_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define OV_DLL_EXPORT
    #define OV_DLL_LOCAL
  #endif
  #define OV_DLL_IMPORT
#endif

#if defined (BUILDING_OEV_KALMAN)
  #define OEV_PUBLIC OV_DLL_EXPORT
  #define OEV_LOCAL  OV_DLL_LOCAL
#else /* BUILDING_OEV_KALMAN */
  #define OEV_PUBLIC OV_DLL_IMPORT
  #define OEV_LOCAL  OV_DLL_LOCAL
#endif /* BUILDING_OEV_KALMAN */


#if defined (BUILDING_OEV_UTILS)
  #define OEV_UTILS_PUBLIC OV_DLL_EXPORT
  #define OEV_UTILS_LOCAL  OV_DLL_LOCAL
#else /* BUILDING_OEV_UTILS */
  #define OEV_UTILS_PUBLIC OV_DLL_IMPORT
  #define OEV_UTILS_LOCAL  OV_DLL_LOCAL
#endif /* BUILDING_OEV_UTILS */

#if defined (BUILDING_OEV_DRIVER)
  #define OEV_DRIVER_PUBLIC OV_DLL_EXPORT
  #define OEV_DRIVER_LOCAL  OV_DLL_LOCAL
#else /* BUILDING_OEV_UTILS */
  #define OEV_DRIVER_PUBLIC OV_DLL_IMPORT
  #define OEV_DRIVER_LOCAL  OV_DLL_LOCAL
#endif /* BUILDING_OEV_UTILS */


#if defined (BUILDING_OEV_MAIN)
  #define OEV_MAIN_PUBLIC OV_DLL_EXPORT
  #define OEV_MAIN_LOCAL  OV_DLL_LOCAL
#else /* BUILDING_OEV_UTILS */
  #define OEV_MAIN_PUBLIC OV_DLL_IMPORT
  #define OEV_MAIN_LOCAL  OV_DLL_LOCAL
#endif /* BUILDING_OEV_UTILS */


namespace openEV {

/**
 * \brief The global float type. Change this one to double, and the entire system will run in double.
 * For optimal performance this should be *float*. Eigen can use the NEON unit for vectorized arithmetic on ARMV7 processors if available.
 */
	typedef float FloatType;

	/**
	 * \brief Use the system clock duration definition throughout
	 */
	typedef std::chrono::system_clock::duration OEVDuration;

}

/** \brief Macro to define enums, with a facility to directly stream the Enum name, or to retrieve a string
 *
 * The macro requires:
 *
 *       #include <ostream>
 *       #include <string>
 *       #include <unordered_map>
 *       #include <stdlib.h>
 *       #include <sstream>
 *       #include <cstdlib>
 *
 * Use it as follows:
 *
 * Instead of
 *
 *       enum foo { bar, moose, clam};
 *
 * write
 *
 *       OEV_ENUM ( foo,  bar, moose, clam);
 *
 * Please note that this macro will also work for enumeration with valued enumerations.
 * something like
 *
 *       enum xx {a=2, b=4, c=5}
 *
 * can be re-written to
 *
 *       OEV_ENUM ( xx, a=2, b=4, c=5}
 *
 * will return the correct representation for values 2, 4, and 5. Any value in between (here 3) will be printed as unknown value.
 *
 * It implements the enum foo with its members,
 * and a function
 *       std::string getString (enumName en)
 * within the same visibility and class scope as the enum definition.
 *
 * You may use this function to implement an ostream "<<" output operator.
 *
 */

#define OEV_ENUM(enumName, ...) \
	enum enumName { __VA_ARGS__ }; \
	/* helper class in the same scope */ \
	class enumName##HelperClass { \
		std::unordered_map<int,std::string> enumStrings; \
	public: \
		enumName##HelperClass() { \
			int enumVal = 0; \
			std::string enumValStr; \
			std::string enumStr; \
			bool addToEnumStr = true; \
			char const* nameList = #__VA_ARGS__; \
			while (*nameList != '\0') { \
				char c= *nameList; \
				switch (c) { \
				case ' ': \
				case '\t': \
				case '\r': \
				case '\n': \
					/* Ignore whitespaces */ \
					break; \
				case '=': \
					addToEnumStr = false; \
					break; \
				case ',': /* The previous enumName is finished and will be stored. */ \
					if (enumStr.length() > 0) { \
						if (enumValStr.length() > 0) { \
							long tmp = strtol(enumValStr.c_str(),0,0); \
							if (tmp != LONG_MAX && tmp != LONG_MIN) { \
								enumVal = int(tmp); \
							} \
						} \
						std::pair<int,std::string> p (enumVal,enumStr); \
						enumStrings.insert(p); \
						enumStr.clear(); \
						enumValStr.clear(); \
						addToEnumStr = true; \
						enumVal++; \
					} \
					break; \
				default: \
					if (addToEnumStr) { \
						enumStr += c; \
					} else { \
						enumValStr += c; \
					} \
				}  \
				nameList ++; \
			}  \
			if (enumStr.length() > 0) { \
				if (enumValStr.length() > 0) { \
					long tmp = strtol(enumValStr.c_str(),0,0); \
					if (tmp != LONG_MAX && tmp != LONG_MIN) { \
						enumVal = int(tmp); \
					} \
				} \
				std::pair<int,std::string> p (enumVal,enumStr); \
				enumStrings.insert(p); \
			}  \
		} \
		 \
		std::string getString (enumName en) { \
			std::unordered_map<int,std::string>::iterator it =  enumStrings.find(int(en)); \
			if (it == enumStrings.end()) { \
				std::ostringstream os; \
				os << "<Unknown " #enumName " value <" << int(en) << ">"; \
				return os.str(); \
			} \
			 \
			return it->second; \
		} \
	};  \
	static enumName##HelperClass enumName##HelperObj;

#endif /* OEVCOMMON_H_ */
