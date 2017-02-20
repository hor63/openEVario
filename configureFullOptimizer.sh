#!/bin/bash

if [ -f Makefile ]
then
  make distclean
fi

CFLAGS="-Ofast -funsafe-loop-optimizations -funsafe-loop-optimizations -floop-interchange -floop-strip-mine -floop-block $CFLAGS" \
CXXFLAGS="$CFLAGS" \
LDFLAGS="$CFLAGS" \
./configure $*
