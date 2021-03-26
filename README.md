Linkam3
=======

EPICS asyn driver for Linkam T96 controllers.

Credits and licensing
---------------------

This driver is based on the original Linkam3 dirver written at Diamond Light Source, available [here](https://github.com/dls-controls/linkam3). It has been significantly updated since then at NSLS2 to fix some issues, add new functionality, and simplify some operations. For example, the license file path is no longer hard coded in the driver, and the driver can now use both USB and Serial communication. IOCs using the original diamond driver are not compatible.

Supported platforms
-------------------

The driver can be built on both Linux and Windows systems. It has been tested on Ubuntu, Debian, and
RedHat distributions.

SDK
---

If you want to change the version of the SDK:

* Replace include files in linkamT96Support/os/*.h
* Replace library files like libLinkamSDK.so in linkamT96Support/os/YOUR_DISTRIBUTION

Installation
------------

First, edit the paths to required modules in `configure/RELEASE`. Then, build the module with:

```
make
```

Next, navigate to the `iocs/linkamIOC/iocBoot/iocLinkam` directory. Open the `st.cmd` file, and ensure
that the executable at the top of the file matches your host architecture. Then, determine if you wish to connect
via USB or serial, edit the IOC prefix, and add the path to your linkam SDK license file and optionally the path
to the linkam log file. At this point you should be ready to run the IOC.

