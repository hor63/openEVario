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

#if HAVE_SYS_TIME_H == 1
#	include <sys/time.h>
#endif

#include "CommonLogger_ShLib.h"

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
	 * cause isnan() to return \p false even it was initialized with \p NAN.
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


}

#endif /* COMMONDEFS_H_ */
