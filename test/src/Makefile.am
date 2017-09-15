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

noinst_LIBRARIES = libgoogletest.a
libgoogletest_a_SOURCES = $(top_srcdir)/3rdParty/googletest/googletest/src/gtest-all.cc

bin_PROGRAMS = testOpenEVario$(EXEEXT)
testOpenEVario_SOURCES = testOpenEVario.cpp \
  $(top_srcdir)/src/GliderVarioStatus.cpp \
  $(top_srcdir)/src/GliderVarioTransitionMatrix.cpp \
  $(top_srcdir)/src/RotationMatrix.cpp \
  $(top_srcdir)/src/FastMath.cpp \
  $(top_srcdir)/src/FastMathSineTable.cpp \
  $(top_srcdir)/src/MeasureMatrix.cpp \
  $(top_srcdir)/src/GliderVarioMeasurementVector.cpp \
  $(top_srcdir)/src/GliderVarioMeasurementUpdater.cpp \
  FastMath_test.cpp \
  GliderVarioTransitionMatrix_test.cpp \
  GliderVarioMeasurementUpdater_test.cpp \
  GliderVarioMeasurementVector_test.cpp	\
  MeasureMatrix_test.cpp \
  GliderVarioStatus_test.cpp \
  RotationMatrix_test.cpp
  
testOpenEVario_LDADD = ./libgoogletest.a $(PTHREAD_LIBS)

# Build compiler sources are not well supported by autotools. Back to manual
genSineTables_sources=../../src/genSineTables.cpp

AM_CXXFLAGS = -isystem $(top_srcdir)/3rdParty/googletest/googletest/include \
	-I../.. \
	-I$(top_srcdir)/3rdParty/eigen \
	-I$(top_srcdir)/src \
	-DENABLE_UNIT_TESTS=1 \
	$(PTHREAD_CFLAGS)

# disable optimizer to make debugging easier
CXXFLAGS ?= -g -O0
CXXFLAGS_FOR_BUILD += -g -O0

# compiling googletest itself requires PTHRED options, and the googletest source directory as include path.
libgoogletest_a_CXXFLAGS = -isystem $(top_srcdir)/3rdParty/googletest/googletest/include -I$(top_srcdir)/3rdParty/googletest/googletest

AM_CXXFLAGS_FOR_BUILD="-I$(top_srcdir)/3rdParty/eigen/Eigen"

LINK_FOR_BUILD.cpp = $(CXX_FOR_BUILD) $(CXXFLAGS_FOR_BUILD) $(CPPFLAGS_FOR_BUILD) $(AM_CXXFLAGS_FOR_BUILD) $(LDFLAGS_FOR_BUILD) $(TARGET_ARCH_FOR_BUILD)

# compile and link in one step. Saves me at least mainaining object and source file lists.
genSineTables$(EXEEXT) : $(genSineTables_sources)
	$(LINK_FOR_BUILD.cpp) $^ $(LOADLIBES_FOR_BUILD) $(LDLIBS_FOR_BUILD) -o $@

# Generate the sinus and atan2 lookup tables for the FastMath class.
FastMathSineTable.cpp: genSineTables$(EXEEXT)
	./genSineTables$(EXEEXT) FastMathSineTable.cpp
	
