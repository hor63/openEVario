/*
 * CommonDefs.h
 *
 *  Created on: Jan 23, 2022
 *      Author: hor
 *
 *	Commonly used definitions of nature constants,
 *	rename typedefs of commonly used types,
 *	and some conversion and helper routines for barometric formulas
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2022  Kai Horstmann
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

#ifndef COMMONDEFS_H_
#define COMMONDEFS_H_

#if defined HAVE_CONFIG_H
#	include "config.h"
#endif

#include <chrono>
#include <ctime>
#include <string>

#if HAVE_SYS_TIME_H == 1
#	include <sys/time.h>
#endif

#include "CommonLogger_ShLib.h"

#include "gettext.h"
#define _(str) dgettext(PACKAGE,str)
#define N_(str1,str2,N) dngettext(PACKAGE,str1,str2,N)

/**
 * Define OV_DLL_IMPORT, OV_DLL_EXPORT, and OV_DLL_LOCAL for Windows and Linux (ELF) ports of gcc and non-gcc compilers
 *
 * The macro definitions are highly inspired from the [GCC Wiki: Visibility](https://gcc.gnu.org/wiki/Visibility)
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
	 * \brief The global float type. Recommended *float*
	 *
	 * For optimal performance this should be *float*.
	 * Eigen can use the NEON unit for vectorized arithmetic on ARMV7 processors if available.
	 *
	 * Change this one to double, and the entire system will run in \p double.
	 */
	typedef float FloatType;

	/** \brief Indicator value for untouched variables
	 *
	 * Use this value to indicate that further initialization is required.
	 *
	 * I cannot use \p NAN and \p isnan() because compiler options
	 * -funsafe-math-optimizations or -ffast-math or -Ofast
	 * because isnan() to return \p false even it was initialized with \p NAN.
	 * This is because these options treat all binary patterns as finite float values.
	 * I still need these options for ARM architectures to let GCC generate code
	 * for the NEON vector unit to speed up the number crunching.
	 *
	 *
	 * The value is close enough to 0 that in most cases it causes no harm
	 * if the value is accidentally not further initialized.
	 */
	static constexpr FloatType UnInitVal = 1235e-25;

	/**
	 * \brief Use the system clock as my clock class throughout
	 */
	typedef std::chrono::system_clock OEVClock;

	/**
	 * \brief Use the system clock duration definition throughout
	 */
	typedef OEVClock::duration OEVDuration;

	template <typename TimePointType>
	std::string timePointToString (const TimePointType t) {
		char buf[64];
		std::time_t tTimeT = TimePointType::clock::to_time_t(t);

		std::strftime(buf, sizeof buf,  "%F %T %Z", std::localtime(&tTimeT));

		return std::string(buf);
	}

}

#endif /* COMMONDEFS_H_ */
