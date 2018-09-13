#!/bin/bash

set -eu

# These environment variables are provided by travis in CI
echo "Using env TRAVIS_BRANCH=${TRAVIS_BRANCH:=develop}"

# Install the binaries that we build on top of
gbz_name=$(getgbz -b ${TRAVIS_BRANCH} --dry-run)
ci_name=$(echo ${gbz_name} | sed 's/^gizmo/ci/g')
edu_name=$(echo ${gbz_name} | sed 's/^gizmo/edu/g')

echo "Downloading and installing EDU Binaries=${edu_name}"
gets3 "edu_bin/${edu_name}"
tar -xzC /opt -f edu-*.tar.gz
ln -s /opt/edu-* /opt/edu

echo "Downloading and installing CI Binaries=${ci_name}"
gets3 "ci_bin/${ci_name}"
tar -xzC /opt -f ci-*.tar.gz
ln -s /opt/ci-* /opt/ci

echo "Setting up build environment"
cd /root/kuri_edu
git clean -dxf  # Cleanup for local builds
set +u  # setup.bash expects some unset vars. . .
source /opt/edu/setup.bash
source /opt/ci/setup.bash --extend
set -u

echo -e "\nBuilding kuri_edu package"
catkin_make --source .

echo -e "\nRunning Tests"
set +u  # Ugh. . .
source ./devel/setup.bash
set -u
catkin_make run_tests --source .
catkin_test_results

echo -e "\nCoverage Report:"
# Finally, print out some final coverage numbers
# If ROS tests are added, their coverage files get dropped in the rostest folder
# combine that with the unit test coverage before generating a report
# coverage combine -a kuri_edu/rostest/.coverage
coverage report
coverage html  # For running locally - makes it easy to see what lines are hit
