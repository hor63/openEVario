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


pkglib_LTLIBRARIES=libopenEVUtil.la

libopenEVUtil_la_SOURCES=RotationMatrix.cpp FastMath.cpp FastMathSineTable.cpp crc.cpp \
	GliderVarioDriverBase.cpp GliderVarioDriverLibBase.cpp \
	GliderVarioExceptionBase.cpp \
	io/PortBase.cpp io/StreamPort.cpp \
	io/DatagramPort.cpp

if BUILD_SERIAL_PORT_DRIVER
libopenEVUtil_la_SOURCES += io/SerialPort.cpp
endif

if BUILD_IP_PORT_DRIVERS
libopenEVUtil_la_SOURCES += io/TCPPort.cpp io/UDPPort.cpp
endif

if BUILD_I2C_PORT_DRIVER
libopenEVUtil_la_SOURCES += io/I2CPort.cpp
endif

libopenEVUtil_la_CXXFLAGS=$(AM_CXXFLAGS) -DBUILDING_OEV_UTILS=1 $(DLL_VISIBLE_CFLAGS)

libopenEVUtil_la_LDFLAGS = $(LD_NO_UNDEFINED_OPT) $(MAIN_LDFLAGS) $(LOG4CXX_LDFLAGS)
libopenEVUtil_la_LIBADD = $(LOG4CXX_LIBS) $(MAIN_LIBS) $(PTHREAD_LIBS)


# Build compiler sources are not well supported by autotools. Back to manual
genSineTables_sources=genSineTables.cpp

AM_CXXFLAGS = -I$(top_srcdir)/src -I$(top_srcdir)/3rdParty/eigen $(LOG4CXX_CXXFLAGS) \
	$(PTHREAD_CFLAGS)

AM_CXXFLAGS_FOR_BUILD=-I$(top_srcdir)/src -I$(top_srcdir)/3rdParty/eigen

LINK_FOR_BUILD.cpp=$(CXX_FOR_BUILD) $(CXXFLAGS_FOR_BUILD) $(CPPFLAGS_FOR_BUILD) $(AM_CXXFLAGS_FOR_BUILD) $(LDFLAGS_FOR_BUILD) $(TARGET_ARCH_FOR_BUILD)

# compile and link in one step. Saves me at least mainaining object and source file lists.
genSineTables$(EXEEXT) : $(genSineTables_sources)
	$(LINK_FOR_BUILD.cpp) $^ $(LOADLIBS_FOR_BUILD) $(LDLIBS_FOR_BUILD) -o $@

# Generate the sinus and atan2 lookup tables for the FastMath class.
FastMathSineTable.cpp: genSineTables$(EXEEXT)
	./genSineTables$(EXEEXT) FastMathSineTable.cpp
	
