#!/bin/bash

compileHeaders() {

find $1 -name "*.h" | while read i
do
    echo "
#if defined HAVE_CONFIG_H
#	include \"config.h\"
#endif

#include \"$i\"

int main () {
  return 0;
}
" > testHdr.cpp

    echo "checking  $i"

    g++ -std=gnu++17 -DHAVE_CONFIG_H -I. -I../src/util -I../   -isystem ../3rdParty/Properties4CXX/include -I../src -isystem ../3rdParty/eigen \
        -pthread -fvisibility=internal -Og -g3 -march=native -mtune=native -Wall -Wextra -Wno-unused-parameter -c -o testHdr.o testHdr.cpp

    if test $? != 0
    then 
        return 1
    fi
    
done

}

while test -n "$1"
do
  echo " "
  echo "--------------- Test headers in directory $1 ----------------------"
  echo " "
  compileHeaders $1
  shift
done

