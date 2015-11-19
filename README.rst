====================
ibmvmc kernel module
====================

NOTE
----
This kernel module currently uses DKMS until it can be merged upstream.
Support for DKMS can be used in one of two ways
- Manually triggering a DKMS build from this repository, as explained below
- Generating and installing the ibmvmc-dkms package using the instructions
  below and the tooling contained in this repository


Overview
--------
The ibmvmc kernel module provides a device driver with interfaces supporting
interaction with the IBM PowerPC Virtual Communications Channel (VMC) device.

Version
-------
1.0.0.1

License
-------
The license can be found in the LICENSE file. It must be reviewed prior to use.


Manually Building and Installing with DKMS
------------------------------------------
To build and install this code using DKMS manually, do the following:
- Clone down this repository on the machine you want to build/install on
- On the same machine, copy the neo-vmc repo to /usr/src/ibmvmc-<version>
- Install prerequisite packages:
  - dkms
  - debhelper
  - linux-headers-$(uname -r)
- Register the module with dkms using "sudo dkms add -m ibmvmc -v <version>"
- Build the module using dkms by running "sudo dkms build -m ibmvmc -v <version>"
- Install the package using dkms by running "sudo dkms install -m ibmvmc -v <version>"
- Modprobe the ibmvmc module


Building the ibmvmc DKMS Source Package
---------------------------------------
NOTE: The following dependencies are required when building the ibmvmc-dkms package
- dkms
- debhelper
- linux-headers-$(uname -r)

To build the DKMS package, run, with sudo/root authority:
- scripts/build-ibmvmc-dkms.sh [-i]
  - Note: Do not use hyphens in the version

Running this script will perform the following steps:
- If not installed, the dependencies listed above can be installed with the -i flag
- If not manually set, the ibmvmc version will be automatically generated using git tags
- The ibmvmc source code will be placed at /usr/src/ibmvmc-<version>
- The ibmvmc module will be added to DKMS
- The ibmvmc module will be built
- A debian source package will be built for ibmvmc-dkms
- A binary debian package containing this source will be built
- The build files will be cleaned up


Installing the DKMS Package
---------------------------
Install the package built in the previous step using:
- dpkg -i ibmvmc-dkms-<version>

As part of the ibmvmc-dkms package installation
- The ibmvmc module will be installed into the kernel module tree through DKMS
- The ibmvmc module will be loaded


Uninstalling the DKMS Package
-----------------------------
To uninstall the package, run:
- dpkg -r ibmvmc-dkms

Doing this will:
- Remove the ibmvmc module from the kernel module tree using DKMS
- Remove the ibmvmc source package from the system
