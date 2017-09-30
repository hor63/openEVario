/*
 * dllCommon.h
 *
 *  Created on: Sep 29, 2017
 *      Author: hor
 *
 *  Common definitions for building shared libraries
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

#ifndef DLL_COMMON_H_
#define DLL_COMMON_H_

/**
 * Define DLL_IMPORT, DLL_EXPORT, and DLL_LOCAL for Windows and Linux (ELF) ports of gcc and non-gcc compilers
 *
 * The macro definitions are highly inspired from the <a href="https://gcc.gnu.org/wiki/Visibility">GCC Wiki: Visibility</a>
 */
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
      #define DLL_EXPORT __attribute__ ((dllexport))
      #define DLL_IMPORT __attribute__ ((dllimport))
    #else
      #define DLL_EXPORT __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
      #define DLL_IMPORT __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
    #endif
    #ifdef __GNUC__
    #else
    #endif
  #define DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define DLL_EXPORT __attribute__ ((visibility ("default")))
    #define DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define DLL_EXPORT
    #define DLL_LOCAL
  #endif
  #define DLL_IMPORT
#endif

#if defined (BUILDING_OEV_KALMAN)
  #define OEV_PUBLIC DLL_EXPORT
  #define OEV_LOCAL  DLL_LOCAL
#else /* BUILDING_OEV_KALMAN */
  #define OEV_PUBLIC DLL_IMPORT
  #define OEV_LOCAL  DLL_LOCAL
#endif /* BUILDING_OEV_KALMAN */

#endif /* DLL_COMMON_H_ */
