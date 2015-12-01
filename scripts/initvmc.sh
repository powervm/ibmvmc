#!/bin/sh

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

DEVICE=ibmvmc

# Load module
modprobe $DEVICE
if [ $? -ne 0 ]; then
    echo "Error loading $DEVICE module\n"
    exit 1
fi

# Create device
if [ ! -c /dev/$DEVICE ]; then
    MAJOR=`sed -n 's/\([0-9]\+\) ibmvmc/\1/p' /proc/devices`
    if [ -z "$MAJOR" ]; then
	echo "Error getting $DEVICE major number\n"
	rmmod $DEVICE
	exit 1
    fi

    # TODO what should be permissions and ownership be?
    mknod -m 0660 /dev/$DEVICE c $MAJOR 0
    if [ $? -ne 0 ]; then
	echo "Error creating $DEVICE\n"
	rmmod $DEVICE
	exit 1
    fi
fi


