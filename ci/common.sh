#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

function die {
  echo $1 >&2
  exit 1
}

function install_system_dependencies() {
  # install GCC
  currentver="$(gcc -dumpfullversion -dumpversion)"
  requiredver="7.0.0"
  if [ ! "$(printf '%s\n' "$requiredver" "$currentver" | sort -V | head -n1)" = "$requiredver" ]; then
    # install GCC 7
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test > /dev/null
    sudo apt-get update > /dev/null
    sudo apt-get install -y gcc-7 g++-7 > /dev/null
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 --slave /usr/bin/g++ g++ /usr/bin/g++-7 > /dev/null
  fi

  currentver="$(gcc -dumpfullversion -dumpversion)"
  if [ ! "$(printf '%s\n' "$requiredver" "$currentver" | sort -V | head -n1)" = "$requiredver" ]; then
    die "Failed to install required GCC version. Current version is $currentver."
  fi
}

function enable_git_access_for_service_accounts() {
  [[ -z "$KOKORO_ARTIFACTS_DIR" ]] && die "No artifacts environment variable specified."

  # this key is configured in
  # google3/production/security/keystore/config/robco_kokoro/keystore-prod.cfg
  # and updated as described in http://go/keystore-cookbook#root-keystore
  KEY_FILE=$KOKORO_ARTIFACTS_DIR/keystore/73196_service-account-key

  # the key file is used the oauth credentials retrieval below
  export GOOGLE_APPLICATION_CREDENTIALS=$KEY_FILE

  [[ ! -f $GOOGLE_APPLICATION_CREDENTIALS ]] && die "No application credentials specified."

  # this python script authenticates the service account, writes a cookie with
  # an authentication token to .git-credential-cache and configures git to use
  # these cookies for authentication during HTTPS calls
  ${DIR}/configure-git-cookie-auth

  # configure git to use HTTPS instead of SSO
  git config --global url.https://cloud-robotics.googlesource.com/.insteadOf sso://cloud-robotics.googlesource.com/
  git config --global --add url.https://cloud-robotics.googlesource.com/.insteadOf sso://cloud-robotics/
}

function prepare_testlogs() {
  # bazel loves symlinks, no one else does (b/79249732)
  # Bazel puts test results into to test.xml, organize in a tree to which it symlinks
  # 'bazel-testlogs'. Unfortunately kokoro's artifact uploaders don't handle symlinks with their
  # globbing.
  # As a workaround we create a shadow tree under 'testlogs' without symlinks. At the same time we
  # rename the test.xml files to sponge_log.xml, which is expected on the kokoro/sponge side.

  local WD=${PWD}
  sudo find -L bazel-testlogs -name 'test.xml' -exec bash -c 'mv $0 ${0/test.xml/sponge_log.xml}' {} \;
  cd bazel-testlogs
  find -L . -name "sponge_log.xml" | sudo tar cvf ${WD}/testlogs.tar -T -

  cd ${WD}
  mkdir -p testlogs
  cd testlogs
  sudo tar xvf ${WD}/testlogs.tar

  cd ${WD}
}
