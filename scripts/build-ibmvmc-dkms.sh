#!/usr/bin/env bash
# Copyright 2015, IBM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

## Shell Opts ----------------------------------------------------------------
set -e -u -o pipefail

## Vars ----------------------------------------------------------------------
install_deps=0
debian_build_deps="dkms debhelper linux-headers-$(uname -r)"
basedir="$(git rev-parse --show-toplevel)"
DKMS_SCRIPT=dkms.mkconf

## Exports ----------------------------------------------------------------------
export IBMVMC_MODULE_NAME=${IBMVMC_MODULE_NAME:-"ibmvmc"}
# Note - Do not use a hyphen ('-') as part of the version. See: https://bugs.launchpad.net/dkms/+bug/599983
export IBMVMC_VERSION=${IBMVMC_VERSION:-"$(git describe --tags --long | sed '1s/^.//' | sed 's/-/./g')"}
export IBMVMC_SOURCE_LOCATION=${IBMVMC_SOURCE_LOCATION:-$basedir}
export IBMVMC_SOURCE_DESTINATION=${IBMVMC_SOURCE_DESTINATION:-"/usr/src/${IBMVMC_MODULE_NAME}-${IBMVMC_VERSION}"}
export DKMS_CONF_LOCATION=${DKMS_CONF_LOCATION:-"$IBMVMC_SOURCE_DESTINATION/dkms.conf"}
export IBMVMC_STRIP_DEBUG=${IBMVMC_STRIP_DEBUG:-"yes"}

## Functions ----------------------------------------------------------------------

function usage {
  echo "Usage: $0 [OPTION]..."
  echo ""
  echo "  -i, --install-deps   Install build dependencies (Ubuntu only)"
  echo "  -h, --help           Print this message"
  echo ""
  exit
}

function process_option {
  case "$1" in
    -h|--help) usage;;
    -i|--install_deps) install_deps=1;;
    *) usage;;
  esac
}

## Main ----------------------------------------------------------------------

# Process arguments
for arg in "$@"; do
    process_option $arg
done

echo "Building ibmvmc-dkms package"

# Try to install the dependency packages if specified
if [ $install_deps -eq 1 ]; then
    os_type=$(awk -F= '/^NAME/{print $2}' /etc/os-release | tr -d '"')
    if [ $os_type == "Ubuntu" ]; then
        apt-get update
        apt-get -y install $debian_build_deps
    else
        echo "Installing dependencies on a non-Ubuntu environment not currently supported" && exit 1
    fi
fi

# If the source directory exists remove it
if [ -d "${IBMVMC_SOURCE_DESTINATION}" ];then
    rm -rf "${IBMVMC_SOURCE_DESTINATION}"
fi

# Clean up any old existing DKMS installs from the same version
if [ -d "/var/lib/dkms/${IBMVMC_MODULE_NAME}/${IBMVMC_VERSION}" ]; then
    rm -rf "/var/lib/dkms/${IBMVMC_MODULE_NAME}/${IBMVMC_VERSION}"
fi

# Place the ibmvmc-dkms source
cp -ra "${IBMVMC_SOURCE_LOCATION}" "${IBMVMC_SOURCE_DESTINATION}"

# Build and place the DKMS configuration file with the source
if [ -f "${basedir}/scripts/${DKMS_SCRIPT}" ]; then
    "${basedir}/scripts/${DKMS_SCRIPT}" "-n ${IBMVMC_MODULE_NAME}-dkms" "-v ${IBMVMC_VERSION}" "-f ${DKMS_CONF_LOCATION}" "-d ${IBMVMC_STRIP_DEBUG}"
else
    echo "Missing DKMS build script ${DKMS_SCRIPT}" && exit 1
fi

# Add the ibmvmc module to dkms
dkms add -m "${IBMVMC_MODULE_NAME}" -v "${IBMVMC_VERSION}"

# Build the ibmvmc module
dkms build -m "${IBMVMC_MODULE_NAME}" -v "${IBMVMC_VERSION}"

# Build the ibmvmc-dkms source package
dkms mkdsc -m "${IBMVMC_MODULE_NAME}" -v "${IBMVMC_VERSION}" --source-only

# Build the ibmvmc-dkms debian package
IBMVMC_PACKAGE_LOCATION=$(dkms mkdeb -m "${IBMVMC_MODULE_NAME}" -v "${IBMVMC_VERSION}" --source-only | awk '/built/')

echo "${IBMVMC_PACKAGE_LOCATION}"
echo "The ibmvmc-dkms package was built successfully"
