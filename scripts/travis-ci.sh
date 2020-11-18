#!/bin/bash

set -e

travis_nanoseconds() {
  python -c 'import time; print("{:d}".format(int(time.time()*1000000000)))'
}

travis_start() {
  travis_timer_id=`printf %08x $(( RANDOM * RANDOM ))`
  travis_start_time=`travis_nanoseconds`
  echo -e "travis_time:start:$travis_timer_id\r\033[0m$2"
  echo -e "travis_fold:start:$1\n$2"
}

travis_finish() {
  echo "travis_fold:end:$1"
  travis_end_time=`travis_nanoseconds`
  local duration=$(( $travis_end_time - $travis_start_time ))
  echo -en "\ntravis_time:end:$travis_timer_id:start=$travis_start_time,finish=$travis_end_time,duration=$duration\r\033[0m"
}

PARALLEL=-j2

# This should be set via .travis.yml depending on the OS/Distribution
# PARALLEL_CTEST=-j1

travis_start qmake "Building OpenSCAD using qmake"
export PATH="/usr/local/opt/gettext/bin:$PATH"
qmake CONFIG+=experimental CONFIG+=nogui && make $PARALLEL
travis_finish qmake

travis_start cmake "Building tests using cmake"

cd tests
cmake .
if [[ $? != 0 ]]; then
  echo "Error configuring test suite"
  exit 1
fi
make $PARALLEL
if [[ $? != 0 ]]; then
  echo "Error building test suite"
  exit 1
fi

travis_finish cmake

travis_start ctest "Running tests using ctest"

# Exclude tests known the cause issues on Travis
# opencsgtest_rotate_extrude-tests - Fails on Ubuntu 12.04 using Gallium 0.4 drivers
# *_text-font-direction-tests - Fails due to old freetype (issue #899)
# throwntogethertest_issue964 - Fails due to non-planar quad being tessellated slightly different
# opencsgtest_issue1165 - z buffer tearing

echo "*** Test run disabled ***"

if [[ $? != 0 ]]; then
  echo "Test failure"
  exit 1
fi

travis_finish ctest
