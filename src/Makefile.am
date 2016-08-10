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


bin_PROGRAMS=openEVario$(EXEEXT)
openEVario_SOURCES=openEVario.cpp GliderVarioStatus.cpp GliderVarioTransitionMatrix.cpp RotationMatrix.cpp FastMath.cpp FastMathSineTable.cpp MeasureMatrix.cpp \
 GliderVarioMeasurementVector.cpp GliderVarioMeasurementMatrix.cpp RotMatrixConversion.cpp

# Build compiler sources are not well supported by autotools. Back to manual
genSineTables_sources=genSineTables.cpp

AM_CXXFLAGS="-I$(top_srcdir)/3rdParty/eigen/Eigen"

AM_CXXFLAGS_FOR_BUILD="-I$(top_srcdir)/3rdParty/eigen/Eigen"

LINK_FOR_BUILD.cpp = $(CXX_FOR_BUILD) $(CXXFLAGS_FOR_BUILD) $(CPPFLAGS_FOR_BUILD) $(AM_CXXFLAGS_FOR_BUILD) $(LDFLAGS_FOR_BUILD) $(TARGET_ARCH_FOR_BUILD)

# compile and link in one step. Saves me at least mainaining object and source file lists.
genSineTables$(EXEEXT) : $(genSineTables_sources)
	$(LINK_FOR_BUILD.cpp) $^ $(LOADLIBES_FOR_BUILD) $(LDLIBS_FOR_BUILD) -o $@

# Generate the sinus and atan2 lookup tables for the FastMath class.
FastMathSineTable.cpp: genSineTables$(EXEEXT)
	./genSineTables$(EXEEXT) FastMathSineTable.cpp
	
