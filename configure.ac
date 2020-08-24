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

AC_PREREQ([2.69])
AC_INIT([openEVario],[0.1],[https://github.com/hor63/openEVario/issues],[openEVario],https://github.com/hor63/openEVario)
AC_CONFIG_HEADERS(config.h)

AC_CONFIG_MACRO_DIRS([m4])


AC_CANONICAL_TARGET
AM_INIT_AUTOMAKE()
AM_DEFAULT_VERBOSITY=0


AC_PROG_CXX
AX_PROG_CXX_FOR_BUILD
AX_CXX_COMPILE_STDCXX_14(noext, mandatory)
AX_CXX_COMPILE_BUILD_STDCXX_11(noext, mandatory)
AX_PTHREAD()


# get a suitable command like mkdir -p on LINUX systems use install if mkdir -p does not work.
AC_PROG_INSTALL
AC_PROG_MKDIR_P

# commands for the archiver
AM_PROG_AR
# obsoleted by LT_INIT
#AC_PROG_RANLIB

LT_INIT(win32-dll dlopen )
AC_SUBST([LIBTOOL_DEPS])

AC_LANG([C++])


# Explicitly define _GNU_SOURCE under CYGWIN.
# Some features are not included OOTB in Cygwin as of 16-Apr-2020 with gcc v9
# Particularly missing were fileno() and getaddrinfo() and its companions
if test "$host_os" = cygwin
then
    AC_DEFINE([_GNU_SOURCE],[1],[Define to 1 if you want to enable GNU and all other extensions.])
fi

# Enable specific library handling for Cygwin
AM_CONDITIONAL(OS_IS_CYGWIN, test "$host_os" = cygwin)


AC_CHECK_HEADER([linux/i2c-dev.h],[AC_DEFINE([HAVE_I2C_DEV_H],[1],[Define to 1 if you have the <linux/i2c-dev.h> header file.])],[])
AC_CHECK_HEADER([linux/i2c.h],[AC_DEFINE([HAVE_I2C_H],[1],[Define to 1 if you have the <linux/i2c.h> header file.])],[])

AC_CHECK_HEADER([argp.h],[AC_DEFINE([HAVE_ARGP_H],[1],[Define to 1 if you have the <argp.h> header file.])],[])
AC_CHECK_HEADER([getopt.h],[AC_DEFINE([HAVE_GETOPT_H],[1],[Define to 1 if you have the <getopt.h> header file.])],[])
AC_SEARCH_LIBS([argp_parse],[],[AC_DEFINE([HAVE_ARGP_PARSE],[1],[Define to 1 if the function argp_parse is available.])],[])
AC_SEARCH_LIBS([getopt_long],[],[AC_DEFINE([HAVE_GETOPT_LONG],[1],[Define to 1 if the function getopt_long is available.])],[])
AC_SEARCH_LIBS([getopt],[],[AC_DEFINE([HAVE_GETOPT],[1],[Define to 1 if the function getopt is available.])],[])

AC_STRUCT_TIMEZONE
AC_CHECK_HEADER([time.h],[AC_DEFINE([HAVE_TIME_H],[1],[Define to 1 if you have the <time.h> header file.])],[])
AC_SEARCH_LIBS([timespec_get],[],[AC_DEFINE([HAVE_TIMESPEC_GET],[1],[Define to 1 if the function timespec_get is available.])],[])
AC_CHECK_MEMBER([struct timespec.tv_nsec], 
							[AC_DEFINE([HAVE_TIMESPEC_STRUCT],[1],[Define to 1 if you have the timespec structure defined in time.h.])],[],
                          	[[#include <time.h>]])

AC_CHECK_HEADER([sys/time.h],[AC_DEFINE([HAVE_SYS_TIME_H],[1],[Define to 1 if you have the <sys/time.h> header file.])],[])
AC_SEARCH_LIBS([gettimeofday],[],[AC_DEFINE([HAVE_GETTIMEOFDAY],[1],[Define to 1 if the function gettimeofday is available.])],[])
AC_CHECK_MEMBER([struct timeval.tv_usec], 
							[AC_DEFINE([HAVE_TIMEVAL_STRUCT],[1],[Define to 1 if you have the timeval structure defined in sys/time.h.])],[],
                          	[[#include <sys/time.h>]])

AC_CHECK_HEADER([sys/types.h],[AC_DEFINE([HAVE_SYS_TYPES_H],[1],[Define to 1 if you have the <sys/types.h> header file.])],[])
AC_CHECK_HEADER([sys/socket.h],[AC_DEFINE([HAVE_SYS_SOCKET_H],[1],[Define to 1 if you have the <sys/socket.h> header file.])],[])
AC_CHECK_HEADER([netdb.h],[AC_DEFINE([HAVE_NETDB_H],[1],[Define to 1 if you have the <netdb.h> header file.])],[])
AC_SEARCH_LIBS([getaddrinfo],[],[AC_DEFINE([HAVE_GETADDRINFO],[1],[Define to 1 if the function getaddrinfo is available.])],[])
AC_SEARCH_LIBS([gethostbyname_r],[],[AC_DEFINE([HAVE_GETHOSTBYNAME_R],[1],[Define to 1 if the function gethostbyname_r is available.])],[])
AC_SEARCH_LIBS([gethostbyname],[],[AC_DEFINE([HAVE_GETHOSTBYNAME],[1],[Define to 1 if the function gethostbyname is available.])],[])

# If termios is not available skip building the SerialPort driver
AC_CHECK_HEADER([termios.h],[AC_SUBST([HAVE_TERMIOS_H],[1])],[])
AC_SEARCH_LIBS([tcsetattr],[],[AC_SUBST([HAVE_TCSETATTR],[1])],[])
AS_IF(
[test "x$HAVE_TERMIOS_H" = x1 -a "x$HAVE_TCSETATTR" = x1],
[AC_DEFINE([WITH_SERIAL_PORT_DRIVER],[1],[Define to 1 if the serial port driver is available.])]
)
AM_CONDITIONAL([BUILD_SERIAL_PORT_DRIVER],[test "x$HAVE_TERMIOS_H" = x1 -a "x$HAVE_TCSETATTR" = x1])

# If <netinet/in.h> or <netinet/tcp.h> are not available skip building the TCPPort and UDPPort driver
AC_CHECK_HEADER([netinet/in.h],[AC_SUBST([HAVE_NETINET_IN_H],[1])],[])
AC_CHECK_HEADER([netinet/tcp.h],[AC_SUBST([HAVE_NETINET_TCP_H],[1])],[])
AS_IF(
[test "x$HAVE_NETINET_IN_H" = x1 -a "x$HAVE_NETINET_TCP_H" = x1],
[AC_DEFINE([WITH_IP_PORT_DRIVER],[1],[Define to 1 if the TCP and UDP drivers are available.])]
)
AM_CONDITIONAL([BUILD_IP_PORT_DRIVERS],[test "x$HAVE_NETINET_IN_H" = x1 -a "x$HAVE_NETINET_TCP_H" = x1])

AX_CHECK_COMPILE_FLAG([-fvisibility=internal],[DLL_VISIBLE_CFLAGS="-fvisibility=internal"])
AC_SUBST([DLL_VISIBLE_CFLAGS])

AX_CHECK_LINK_FLAG([-Wl,--no-undefined], [LD_NO_UNDEFINED_OPT=-no-undefined])
AC_SUBST([LD_NO_UNDEFINED_OPT])

# Check for log4cxx
# options are
# --with-log4cxx=no			Do not include logging support in the program
# --without-log4cxx 		Same as --with-log4cxx=no
# --with-log4cxx=<pathname> Use provided log4cxx package in directory <pathname>, e.g. /usr/local
# --with-log4cxx=build 		Build apr, upr-util and log4cxx 
AC_ARG_WITH([log4cxx],
  [AS_HELP_STRING([--with-log4cxx],
[support logging with Apache loc4cxx @<:@default=auto@:>@
[Possible values are: | ]
[<pathname>	: Path to log4cxx installation | ] 
no			: build completely without logging | 
yes (or empty): use the provided log4cxx library. Fatal error if it cannot be used. | 
build		: build log4cxx from shipped sources | 
auto		: (default) when an installation is detected it is used, else build from shipped sources])],
[],
[])


# default is to use the embedded sources if an installation cannot be found
USE_EMBEDDED_LOG4CXX_SOURCES=yes
# By default use the logger
HAVE_LOG4CXX_H=1

# Check for options given
AS_IF(
[test "x$with_log4cxx" = xauto -o "x$with_log4cxx" = x ],
[
	LOG4CXX_PREFIX_LIST="none /opt/log4cxx /opt /usr/local/log4cxx /usr/local"
],

[test "x$with_log4cxx" = xno ],
[
	# The embedded sources are not used, the prefix list is empty, and the header is marked as unusable 
	USE_EMBEDDED_LOG4CXX_SOURCES=no
	AS_UNSET([HAVE_LOG4CXX_H])
],

[test "x$with_log4cxx" = xbuild ],
[
	# Do nothing. Leave the prefix list empty, and leave the embedded use on.
],

[test "x$with_log4cxx" = xyes ],
[
	# Look only in the system directories for includes and lib
	# but do not the embedded sources if not found
	LOG4CXX_PREFIX_LIST="none"
	USE_EMBEDDED_LOG4CXX_SOURCES=no
],

[
	# this must be the prefix to the installation of log4cxx
	LOG4CXX_PREFIX_LIST="$with_log4cxx"
]
)

# Save LIBS for link test
logger_test_save_LIBS="$LIBS"
logger_test_save_LDFLAGS="$LDFLAGS"
logger_test_save_CPPFLAGS="$CPPFLAGS"
LIBS="-llog4cxx $LIBS"

if test "x$LOG4CXX_PREFIX_LIST" != x
then
    # run through the list of potential installation prefixes
	for i in $LOG4CXX_PREFIX_LIST
	do
		if test x"$i" != xnone
		then
			LDFLAGS="$LDGLAGS -L$i/lib"
			CPPFLAGS="$CPPFLAGS -isystem $i/include"
			AC_MSG_CHECKING([log4cxx header and library usability in $i])
		else
			AC_MSG_CHECKING([log4cxx header and library usability])
		fi
		AC_LINK_IFELSE([AC_LANG_PROGRAM(
		[
		#include <log4cxx/logger.h>
		],
		[
		log4cxx::LoggerPtr  logger(log4cxx::Logger::getLogger("com.foo"));
		])],
		[
		USE_EMBEDDED_LOG4CXX_SOURCES=no
		HAVE_LOG4CXX_H=1
		AC_MSG_RESULT([yes])
		LOG4CXX_LIBS=-llog4cxx
		if test x"$i" != xnone
		then
			LOG4CXX_LDFLAGS=-L$i/lib
			LOG4CXX_CXXFLAGS="-isystem $i/include"
		fi
		break
		]
		,
		[
		AS_UNSET([HAVE_LOG4CXX_H])
		AC_MSG_RESULT([no])]
		)
		LDFLAGS="$logger_test_save_LDFLAGS"
	done # for i in $LOG4CXX_PREFIX_LIST
fi # if test "x$LOG4CXX_PREFIX_LIST" != x

# restore LIBS after test 
LIBS="$logger_test_save_LIBS"
LDFLAGS="$logger_test_save_LDFLAGS"
CPPFLAGS="$logger_test_save_CPPFLAGS"

if test x"$USE_EMBEDDED_LOG4CXX_SOURCES" = xyes
then
	if test "x$with_log4cxx" = xauto -o "x$with_log4cxx" = x -o "x$with_log4cxx" = xbuild
	then 
		HAVE_LOG4CXX_H=1
		
		LOG4CXX_LIBS=-llog4cxx
		LOG4CXX_LDFLAGS="-L\$(top_builddir)/3rdParty/apache/log4cxx/src/main/cpp"
		LOG4CXX_CXXFLAGS="-isystem \$(top_builddir)/3rdParty/apache/log4cxx/src/main/include -isystem \$(top_srcdir)/3rdParty/apache/log4cxx/src/main/include"
		
		AC_CONFIG_SUBDIRS([3rdParty/apache/apr 3rdParty/apache/apr-util 3rdParty/apache/log4cxx])
		AC_SUBST([LOG4CXX_MAKE_SUBDIRS],["apr apr-util log4cxx"])
		AC_MSG_NOTICE([Building and using  apr, apr-util, and log4cxx in 3rdParty/apache])
		
	fi # if test "x$with_log4cxx" = xauto -o "x$with_log4cxx" = x -o "x$with_log4cxx" = xbuild
else # if test x"$USE_EMBEDDED_LOG4CXX_SOURCES" = xyes
	if test x"$HAVE_LOG4CXX_H" != x1
	then 
		if test "x$with_log4cxx" = xyes
		then 
			AC_MSG_ERROR([Could not find log4cxx in default include and library locations],[1])
		elif  test "x$with_log4cxx" != xno
		then
			AC_MSG_ERROR([Could not find log4cxx in $with_log4cxx],[1])
		fi # if test "x$with_log4cxx" = xyes -o "x$with_log4cxx" = x
	fi # if test x"$HAVE_LOG4CXX_H" != x1
fi

if test x"$HAVE_LOG4CXX_H" = x1
then
	AC_DEFINE([HAVE_LOG4CXX_H], [1], [Determine if the log4cxx header is present and usable,
		and if log4cxx is to be used at all])
fi

AC_SUBST([LOG4CXX_LIBS])
AC_SUBST([LOG4CXX_LDFLAGS])
AC_SUBST([LOG4CXX_CXXFLAGS])

AC_CONFIG_SUBDIRS([3rdParty/Properties4CXX])
CPPFLAGS="$CPPFLAGS -I\$(top_srcdir)/3rdParty/Properties4CXX/include"

# Link libraries only to the main shared library, but not the partial link jobs in util and kalman
MAIN_LDFLAGS="-L\$(top_builddir)/3rdParty/Properties4CXX/src -lProperties4CXX"
MAIN_LIBS="-lProperties4CXX"
AC_SUBST([MAIN_LDFLAGS])
AC_SUBST([MAIN_LIBS])


DX_DOXYGEN_FEATURE(ON)
DX_DOT_FEATURE(ON)
DX_HTML_FEATURE(ON)
DX_PDF_FEATURE(ON)
DX_PS_FEATURE(OFF)
DX_INIT_DOXYGEN(openEVario, $(srcdir)/doxygen.cfg, doc/doxygen)

AC_CONFIG_FILES(Makefile
				3rdParty/apache/Makefile
                src/util/Makefile
                src/kalman/Makefile
                src/main/Makefile
                src/Makefile
                src/drivers/simul/Makefile
                src/drivers/IGCReader/Makefile
                src/drivers/BoschBMXSensorBoard/Makefile
                src/drivers/NmeaGPS/Makefile
                test/src/Makefile)

AC_OUTPUT

# create the doc/doxygen directory right away. If it does not exist the doxygen rules will not work :(
AS_ECHO(["configure: creating directory doc/doxygen"]) 
AS_MKDIR_P(doc/doxygen)
