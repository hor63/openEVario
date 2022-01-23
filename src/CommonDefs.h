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

#include <ostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <unordered_map>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <ctime>

#if HAVE_SYS_TIME_H == 1
#	include <sys/time.h>
#endif

#include "CommonLogger_ShLib.h"

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


	/** \brief Constant of gravity acceleration.
	 *
	 * Here I am using the [conventional value of Earth gravity](https://en.wikipedia.org/wiki/Gravity_of_Earth#Conventional_value) as basis.
	 * This value should also be used for accelerometers which measure "g" instead m/s<SUP>2</SUP>, and you are asking yourself "What the heck is the absolute value of 1g?"
	 * Drivers can adjust the initial gravity value in the status matrix from the calibration file.
	 *
	 * Exact values for Germany can be obtained for the German gravity base mesh
	 * Deutsches Schweregrundnetz 1994 (DSGN 94) from
	 * [DSGN94-Punktbeschreibung](https://www.bkg.bund.de/SharedDocs/Downloads/BKG/DE/Downloads-DE-Flyer/BKG-DSGN94-DE.pdf?__blob=publicationFile)
	 *
	 * If you cannot find detailed gravity values in you county you can look up the
	 * [Absolute Gravity Database](http://agrav.bkg.bund.de/agrav-meta/) and look up the gravity nearest you.
	 *
	 * \sa [Wikipedia: Gravity of Earth](https://en.wikipedia.org/wiki/Gravity_of_Earth)
	 * \sa [Deutsches Schweregrundnetz 1994 (DSGN94)](https://www.bkg.bund.de/DE/Ueber-das-BKG/Geodaesie/Schwere/Schwere-Deutschland/schwere-deutsch.html#doc57358bodyText1)
	 * \sa [Online gravity calculator within Germany](http://gibs.bkg.bund.de/geoid/gscomp.php?p=s)
	 * \sa [AGrav: Absolute Gravity Database](http://agrav.bkg.bund.de/agrav-meta)
	 */
	static FloatType constexpr GRAVITY = 9.80665;

	/**
	 * difference between K(elvin) and C(elsius)
	 */
	static FloatType constexpr CtoK = 273.15f;

	/**
	 * Initialization value for magnetic inclination.
	 * Rough Average value for Germany is about 67 degrees downward (pitch angle is measured upward).
	 *
	 * \sa [Inklinationskarten für Deutschland von 1982 bis 2012](http://www.gfz-potsdam.de/sektion/erdmagnetfeld/infrastruktur/deutsche-saekularpunktmessungen/inklination)
	 * \sa To calculate inclination and declination at any point on earth use [IGRF-Deklinationsrechner](http://www.gfz-potsdam.de/deklinationsrechner)
	 * or [International Geomagnetic Reference Field](http://www.ngdc.noaa.gov/IAGA/vmod/igrf.html) and
	 * [NOAA Magnetic Field Calculators](http://www.ngdc.noaa.gov/geomag-web/?model=igrf)
	 */
	extern OEV_PUBLIC FloatType MAG_INCLINATION; // = -67.0f;

	/**
	 * Nautical mile to m
	 *
	 * \sa [Wikipedia: Nautical mile](https://en.wikipedia.org/wiki/Nautical_mile)
	 */
	static FloatType constexpr NM_TO_M = 1852.0f;

	/**
	 * The rough length of a arc second latitude in meter at 45deg North.
	 * \sa [Length of a degree of longitude](https://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude)
	 */
	double constexpr LEN_LAT_ARC_SEC = 111132.0 / 3600.0;

	/*
	 * Here a bunch of gas constants for dry air as an ideal gas
	 */

	/// Universal gas constant = 8.3144598 J/mol/K
	/// \sa [Universal Gas Constant](https://en.wikipedia.org/wiki/Gas_constant)
	static FloatType constexpr R         = 8.3144598f;
	/// Molar mass of dry air = 0.0289644 kg/mol
	static FloatType constexpr M         = 0.0289644f;
	/// Specific gas constant for dry air = R/M
	/// \sa [Specific Gas Constant for dry air](https://en.wikipedia.org/wiki/Gas_constant#Specific_gas_constant)
	static FloatType constexpr Rspec     = R/M;

	/**
	 * Standard sea level pressure according to ICAO standard atmosphere in hPascal
	 * \sa [ICAO Standard atmosphere](https://en.wikipedia.org/wiki/International_Standard_Atmosphere#ICAO_Standard_Atmosphere)
	 */
	static FloatType constexpr PressureStdMSL = 1013.25f;
	static FloatType constexpr TempLapseStd = -0.65f / 100.0f; ///< Temperature lapse of the standard ICAO atmosphere in K/m
	static FloatType constexpr TempLapseIndiffBoundLayer = -1.0f / 100.0f; ///< Temperature lapse of the the indifferent mixed boundary layer in K/m

	/**
	 * Exponent of the Barometric formula for a dry atmosphere with a temperature lapse for the mixed boundary laye
	 * [Wikipedia: Barometric formula](https://en.wikipedia.org/wiki/Barometric_formula). Use formula 1 which includes a temperature gradient
	 */
	static FloatType constexpr BarometricFormulaExponent = GRAVITY * M / R / TempLapseIndiffBoundLayer;

	/** \brief Calculate the pressure from the altitude above MSL with the Barometric formula at standard temperature 15C.
	 *
	 * Constants and barometric formula from [Barometric formula:Pressure equations](https://en.wikipedia.org/wiki/Barometric_formula#Pressure_equations)
	 * The simplified formula is taken from
	 * [Barometrische Höhenformel:Internationale Höhenformel](https://de.wikipedia.org/wiki/Barometrische_H%C3%B6henformel#Internationale_H%C3%B6henformel)
	 *
	 * @param altitude Altitude above MSL in m
	 * @param temperatureLapse Temperature lapse in K/m. Default is the standard atmosphere lapse of 0.65K/100m
	 * @return Pressure in hPascal
	 */
	static inline double altToPressureStdTemp (double altitude,double temperatureLapse = -0.65/100.0) {

		static double constexpr R  = 8.3144598	; // Universal gas constant: 8.3144598 J/mol/K
		static double constexpr g0 = 9.80665	; // Gravitational acceleration: 9.80665 m/s2
		static double constexpr M  = 0.0289644	; // Molar mass of Earth's air: 0.0289644 kg/mol
		// Exponent term
		double const exp = g0*M/(R*temperatureLapse); // Exponential term of the Barometric formula

		static double constexpr T0 = 15.0f + CtoK		; // 15C at MSL according to the standard atmosphere

		return  (PressureStdMSL * pow(1 - ((temperatureLapse * altitude ) / T0),exp));

	}

	/** Calculate the factor from pressure at sea level to the current altitude MSL
	 *
	 * The pressure at a certain altitude based on altitude, temperature at the altitude (local measurement), and an assumed
	 * temperature gradient is calculated by the
	 * [Barometric formula](https://en.wikipedia.org/wiki/Barometric_formula). Use formula 1 which includes a temperature gradient
	 *
	 * The temperature gradient which is used in this function is \ref TempLapseIndiffBoundLayer, i.e. -1K/100m
	 * This is the gradient in the boundary layer which is mixed by thermals, and essentially dry adiabatic indifferent.
	 *
	 * This function calculates the factor by which the base pressure at sea level P<SUB>base</SUB>
	 * is multiplied to get the altitude pressure P<SUB>alt</SUB>,
	 * or vice versa by which you divide your measured pressure P<SUB>alt</SUB>
	 * to obtain the pressured reduced to sea level P<SUB>base</SUB>.
	 *
	 * I am assuming a dry atmosphere, and I am ignoring the effects of water vapor.
	 *
	 * @param altMSL The altitude above mean sea level
	 * @param currTempC Temperature in Celsius at the altitude \p altMsl, i.e. your local measured temperature.
	 * @return Factor for calculating altitude pressure P<SUB>alt</SUB> from base pressure at sea level P<SUB>base</SUB>
	 */
	static inline FloatType calcBarometricFactor(
			FloatType altMSL,
			FloatType currTempC
			) {
	    FloatType pFactor;
	    // Temperature in Kelvin
	    FloatType currTempK = currTempC + CtoK;


	    pFactor = powf ((currTempK - (TempLapseIndiffBoundLayer * altMSL)) / currTempK,BarometricFormulaExponent);

		return pFactor;
	}


} // namespace openEV



#endif /* COMMONDEFS_H_ */
