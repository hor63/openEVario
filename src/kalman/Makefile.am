#    This file is part of openEVario, an electronic variometer for glider planes
#    Copyright (C) 2016  Kai Horstmann
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License along
#    with this program; if not, write to the Free Software Foundation, Inc.,
#    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

if OS_IS_CYGWIN
# Under Cywin always build shared libs.
# Otherwise I have linking issues due to dllimport/dllexport discrepancies
pkglib_LTLIBRARIES=libopenEVKalman.la
else
# Build a static lib only to be linked to the main share lib.
noinst_LTLIBRARIES = libopenEVKalman.la
endif

libopenEVKalman_la_SOURCES = GliderVarioStatus.cpp GliderVarioTransitionMatrix.cpp MeasureMatrix.cpp \
 GliderVarioMeasurementVector.cpp GliderVarioMeasurementUpdater.cpp
 
libopenEVKalman_la_LDFLAGS = $(LD_NO_UNDEFINED_OPT) $(MAIN_LDFLAGS) $(LOG4CXX_LDFLAGS)
libopenEVKalman_la_LIBADD = ../util/libopenEVUtil.la $(LOG4CXX_LIBS) $(MAIN_LIBS) $(PTHREAD_LIBS)

libopenEVKalman_la_CXXFLAGS=$(AM_CXXFLAGS) -DBUILDING_OEV_KALMAN=1 $(DLL_VISIBLE_CFLAGS)

AM_CXXFLAGS = -I$(top_srcdir)/src -I$(top_srcdir)/3rdParty/eigen \
	$(LOG4CXX_CXXFLAGS) \
	$(PTHREAD_CFLAGS)

