====================
ibmvmc kernel module
====================

NOTE
----
This kernel module currently uses DKMS until it can be merged upstream.
Support for DKMS can be used in one of two ways
- Manually triggering a DKMS build from this repository, as explained below
- Generating and installing the neo-vmc dkms package using the instructions
  below and the tooling contained in this repository


Overview
--------
The ibmvmc kernel module provides a device driver with interfaces supporting
interaction with the IBM PowerPC Virtual Communications Channel (VMC) device.


License
-------
The license can be found in the LICENSE file. It must be reviewed prior to use.


Manually Triggering DKMS
------------------------
To build and install this code using DKMS manually, do the following:
- Clone down this repository on the machine you want to build/install on
- On the same machine, copy the neo-vmc repo to /usr/src/ibmvmc-<version>
- Install prerequisite packages: 
  - dkms
  - debhelper
  - linux-headers-$(uname -r)
- Add the package to dkms using "sudo dkms add -m ibmvmc -v <version>"
- Build the package using dkms by running "sudo dkms build -m ibmvmc -v <version>"
- Install the package using dkms by running "sudo dkms install -m ibmvmc -v <version>"
- Modprobe the ibmvmc module


Building a DKMS Package
-----------------------
Instructions/Code Incoming
