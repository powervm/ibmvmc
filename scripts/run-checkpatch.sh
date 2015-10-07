#!/bin/bash

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
#
# (c) 2015, Adam Reznechek <adreznec@us.ibm.com>

## Shell Opts ----------------------------------------------------------------
set -e -u -o pipefail

## Vars ----------------------------------------------------------------------
download_checkpatch=1
install_deps=0
debian_deps="perl curl"
basedir="$(git rev-parse --show-toplevel)"
checkpatch_loc="https://raw.githubusercontent.com/torvalds/linux/master/scripts/checkpatch.pl"
spelling_loc="https://raw.githubusercontent.com/torvalds/linux/master/scripts/spelling.txt"
checkpatch_file="${basedir}/scripts/checkpatch.pl"
spelling_file="${basedir}/scripts/spelling.txt"
module_source="${basedir}/ibmvmc"

## Functions ----------------------------------------------------------------------

function usage {
  echo "Usage: $0 [OPTION]..."
  echo ""
  echo "  -i, --install-deps   Install build dependencies (Ubuntu only, requires sudo)"
  echo "  -n, --no-download    Do not download checkpatch from github, using local copies"
  echo "  -h, --help           Print this message"
  echo ""
  exit
}

function process_option {
  case "$1" in
    -h|--help) usage;;
    -i|--install_deps) install_deps=1;;
    -n|--no-fetch) download_checkpatch=0;;
    *) usage;;
  esac
}

## Main ----------------------------------------------------------------------

# Process arguments
for arg in "$@"; do
    process_option $arg
done

echo "Running checkpatch"

# Try to install the dependency packages if specified
if [ ${install_deps} -eq 1 ]; then
    os_type=$(awk -F= '/^NAME/{print $2}' /etc/os-release | tr -d '"')
    if [ $os_type == "Ubuntu" ]; then
        apt-get update
        apt-get -y install ${debian_deps}
    else
        echo "Installing dependencies on a non-Ubuntu environment not currently supported" && exit 1
    fi
fi

if [ ${download_checkpatch} -eq 1 ]; then
    # If the checkpatch files exist, remove them
    if [ -d "${checkpatch_file}" ];then
        rm "${checkpatch_file}"
    fi
    
    if [ -d "${spelling_file}" ];then
        rm "${spelling_file}"
    fi
    
    # Fetch the latest checkpatch and spelling files
    curl -o "${checkpatch_file}" ${checkpatch_loc}
    curl -o "${spelling_file}" ${spelling_loc}
fi

# Make sure checkpatch is executable
chmod +x "${checkpatch_file}"

# Run checkpatch against the ibmvmc source
for filename in ${module_source}/*.c; do
    perl ${checkpatch_file} --no-tree -f "${filename}"
done

for filename in $module_source/*.h; do
    perl ${checkpatch_file} --no-tree -f "${filename}"
done

echo "Checkpatch run finished"
