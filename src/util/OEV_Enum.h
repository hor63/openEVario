/*
 * OEV_Enum.h
 *
 *  Created on: Jan 25, 2022
 *      Author: hor
 *
 *  Helper macro to define enums and generate an output operator <<()
 *  which prints enum values as their symbolic name.
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2016-2022  Kai Horstmann
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

#ifndef UTIL_OEV_ENUM_H_
#define UTIL_OEV_ENUM_H_

#include <unordered_map>

/** \brief Macro to define enums, with a facility to directly stream the Enum name, or to retrieve a string
 *
 * The macro requires:
 *
 *       #include <ostream>
 *       #include <string>
 *       #include <unordered_map>
 *       #include <cstdlib>
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

#if defined DOXYGEN
#define OEV_ENUM(enumName, ...) \
	enum enumName {__VA_ARGS__ };
#else
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

#endif // #if defined DOXYGEN

#endif /* UTIL_OEV_ENUM_H_ */
