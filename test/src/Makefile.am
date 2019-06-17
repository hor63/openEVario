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


EXTRA_LTLIBRARIES = libgoogletest.la
libgoogletest_la_SOURCES = gtest-all.cc

gtest-all.cc: $(top_srcdir)/3rdParty/googletest/googletest/src/gtest-all.cc
	ln -s $(top_srcdir)/3rdParty/googletest/googletest/src/gtest-all.cc gtest-all.cc

TESTS = ./testOpenEVario$(EXEEXT)
check_PROGRAMS = testOpenEVario$(EXEEXT)
testOpenEVario_SOURCES = testOpenEVario.cpp FastMath_test.cpp GliderVarioMeasurementUpdater_test.cpp GliderVarioMeasurementVector_test.cpp \
	GliderVarioStatus_test.cpp GliderVarioTransitionMatrix_test.cpp MeasureMatrix_test.cpp RotationMatrix_test.cpp

testOpenEVario_LDFLAGS = $(AM_LDFLAGS)

testOpenEVario_LDADD = ./libgoogletest.la \
	../../src/kalman/libopenEVKalman.la \
	../../src/util/libopenEVUtil.la \
	$(LOG4CXX_LIBS) \
	$(PTHREAD_LIBS)

../../src/libopenEVKalman.la:
	cd ../../src ; make libopenEVKalman.la

# Build compiler sources are not well supported by autotools. Back to manual
genSineTables_sources=../../src/genSineTables.cpp

testOpenEVario_CXXFLAGS = -isystem $(top_srcdir)/3rdParty/googletest/googletest/include \
	-I../.. \
	-I$(top_srcdir)/3rdParty/eigen \
	-I$(top_srcdir)/src \
	-DENABLE_UNIT_TESTS=1 \
	$(LOG4CXX_CXXFLAGS) \
	$(PTHREAD_CFLAGS)

# compiling googletest itself requires PTHRED options, and the googletest source directory as include path.
libgoogletest_la_CXXFLAGS = \
	-isystem $(top_srcdir)/3rdParty/googletest/googletest/include \
	-I$(top_srcdir)/3rdParty/googletest/googletest

AM_LDFLAGS = $(LOG4CXX_LDFLAGS)