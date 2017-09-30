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


lib_LTLIBRARIES = libopenEVSimul.la

libopenEVSimul_la_SOURCES=simulMain.cpp
 
libopenEVSimul_la_LIBADD = $(PTHREAD_LIBS)

AM_CXXFLAGS = -I../../.. -I$(top_srcdir)/src -I$(top_srcdir)/3rdParty/eigen \
	$(PTHREAD_CFLAGS) \
	$(DLL_VISIBLE_CFLAGS)
	
libopenEVSimul_la_CXXFLAGS=$(AM_CXXFLAGS) -DBUILDING_SIMUL_DRIVER=1

	
