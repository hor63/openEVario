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

dnl Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)
AC_INIT(openEVario, 1.0)

AC_CONFIG_MACRO_DIRS([m4])

AC_CANONICAL_SYSTEM
AM_INIT_AUTOMAKE()


# AC_CONFIG_SUBDIRS(3rdParty/googletest/googletest)

AC_PROG_CXX
AX_PROG_CXX_FOR_BUILD
AX_CXX_COMPILE_STDCXX_11(noext, mandatory)
AX_CXX_COMPILE_BUILD_STDCXX_11(noext, mandatory)

AX_PTHREAD()

# get a suitable command like mkdir -p on LINUX systems use install if mkdir -p does not work.
AC_PROG_INSTALL
AC_PROG_MKDIR_P

DX_DOXYGEN_FEATURE(ON)
DX_DOT_FEATURE(ON)
DX_HTML_FEATURE(ON)
DX_PDF_FEATURE(ON)
DX_PS_FEATURE(OFF)
DX_INIT_DOXYGEN(openEVario, $(srcdir)/doxygen.cfg, doc/doxygen)

AC_CONFIG_FILES(Makefile src/Makefile)

AC_OUTPUT

# create the doc/doxygen directory right away. If it does not exist the doxygen rules will not work :(
echo "configure: creating directory doc/doxygen" 
$MKDIR_P doc/doxygen

