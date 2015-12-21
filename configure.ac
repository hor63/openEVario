dnl Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)
AC_INIT(openEVario, 1.0)


AC_CANONICAL_SYSTEM
AM_INIT_AUTOMAKE()

AC_PROG_CXX
AX_CXX_COMPILE_STDCXX_11(noext, mandatory)

DX_DOXYGEN_FEATURE(ON)
DX_DOT_FEATURE(ON)
DX_HTML_FEATURE(ON)
DX_INIT_DOXYGEN(openEvario, Doxyfile, doc/doxygen)



AC_CONFIG_FILES(Makefile src/Makefile)
AC_OUTPUT

