#!/bin/bash

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


