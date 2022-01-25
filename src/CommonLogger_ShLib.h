/*
 * CommonLogger_ShLib.h
 *
 *  Created on: Sep 29, 2017
 *      Author: hor
 *
 *  Common definitions for building shared libraries,
 *  Log4CXX, and some helpers
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

#ifndef COMMONLOGGER_SHLIB_H_
#define COMMONLOGGER_SHLIB_H_

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

#endif /* COMMONLOGGER_SHLIB_H_ */
