#    This file is part of openEVario, an electronic variometer for glider planes
#    Copyright (C) 2020  Kai Horstmann
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


pkglib_LTLIBRARIES = openEVNmeaGPS.la

openEVNmeaGPS_la_SOURCES=NmeaGPSMain.cpp NmeaGPSLib.cpp NmeaGPSDriver.cpp NMEA0813Protocol.cpp NMEASet.cpp
 
openEVNmeaGPS_la_LDFLAGS=$(LD_NO_UNDEFINED_OPT) $(MAIN_LDFLAGS) $(LOG4CXX_LDFLAGS) -module
openEVNmeaGPS_la_LIBADD = ../../main/libopenEVMain.la ../../kalman/libopenEVKalman.la ../../util/libopenEVUtil.la $(LOG4CXX_LIBS) \
	 $(MAIN_LIBS) $(PTHREAD_LIBS)

AM_CXXFLAGS = -I../../.. -I$(top_srcdir)/src -isystem $(top_srcdir)/3rdParty/eigen -I$(top_srcdir)/3rdParty/fmt/include \
	$(LOG4CXX_CXXFLAGS) \
	$(PTHREAD_CFLAGS) \
	$(DLL_VISIBLE_CFLAGS) \
	-DBUILDING_OEV_DRIVER=1
	
libopenEVNmeaGPS_la_CXXFLAGS=$(AM_CXXFLAGS) -DBUILDING_OEV_DRIVER=1

	
