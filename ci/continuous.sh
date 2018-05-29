#!/bin/bash

set -o errexit # exit imediately, if a pipeline command fails
set -o pipefail # returns the last command to exit with a non-zero status

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/common.sh"

echo "--------------------------------------------------------------------------------"
echo "Timestamp: setting up build environtment at $(date)"
echo "--------------------------------------------------------------------------------"

enable_git_access_for_service_accounts

install_system_dependencies

echo "--------------------------------------------------------------------------------"
echo "Timestamp: build started at $(date)"
echo "--------------------------------------------------------------------------------"

if [ -d git/cartographer-app ] ; then
  cd git/cartographer-app
fi

# it is important to build the tests only, otherwise, we will build the Qt5
# dependency which is currently not part of the third party repository
bazel test --build_tests_only ...

echo "--------------------------------------------------------------------------------"
echo "Timestamp: writing test logs at $(date)"
echo "--------------------------------------------------------------------------------"
prepare_testlogs
