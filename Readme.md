# SDK for Orion Gimbals

## Introduction

The Orion SDK implements all the functionality needed to control any of [Trillium Engineering](http://www.trilliumeng.com)'s Orion family of gimbaled camera systems. It also includes a set of example applications which demonstrate the basic paradigms used to connect to and control an Orion gimbal.  

The entire protocol is implemented in a single [ProtoGen](https://github.com/billvaglienti/ProtoGen) XML file. Running the top-level batch/shell scripts will use ProtoGen to generate all the necessary C code and documentation for encoding and decoding binary packets which conform to the API.  

## Getting Started

Building the Orion SDK for linux has the following prerequisites:  make, qmake, or msvc.

* __Optional:__ MultiMarkdown (<http://fletcherpenney.net/multimarkdown>)

On Ubuntu and other Debian-based distributions, MultiMarkdown can also be installed by running `sudo apt-get install libtext-multimarkdown-perl`.  

This optional dependancy will allow a development system to [generate](https://github.com/trilliumeng/orion-sdk/blob/master/Readme.md#support-and-documentation) a html version of the documentation.  Otherwise documentation is only generated into markdown files.  A preview of the documentation generated for the current release is found [here](https://static.trilliumeng.com/ApplicationData/Orion/download.html?device=OrionSDK).


## Repository Contents

The root directory contains the scripts necessary to generate the SDK code with ProtoGen as well as the parent Makefile and project files for building the SDK and example applications. It also includes several subdirectories:

### Communications

This directory contains the ProtoGen XML file and the Makefile/project necessary to build the SDK as a static library. Once the XML file has been processed, the directory will also contain all of the source code for the SDK. It also contains the low-level code for connecting to the gimbal over the Ethernet and/or serial port interfaces. To initiate a connection with a gimbal, one of the following functions must be used:

* `OrionCommOpenSerial` to connect to a gimbal over a specified serial port
* `OrionCommOpenNetwork` to automatically discover and connect to a gimbal over a network connection 
* `OrionCommOpenNetworkIp` to connect to a gimbal with a known IP address over a network connection

All three functions will return `TRUE` upon a successful connection, then `OrionCommSend` and `OrionCommReceive` may be used to send and receive Orion SDK packets. `OrionCommClose` is used to close down the connection and release all the relevant resources.

### Examples

The `Examples` directory contains some applications which demonstrate both the use of the packet SDK as well as the lower-level process of connecting to and exchanging data with a gimbal over both serial and Ethernet. For detailed information on a particular example application, please see the readme included in its subdirectory.

### Protogen

Holds the Protogen executables for various platforms, including:

* Windows
* Mac OS X
* Linux (x86, x64 and ARM)

If the ProtoGen pre-compiled binaries will not run on your platform, it can be built from the source.   Pull the 2.12.d release at <https://github.com/trilliumeng/ProtoGen/releases/tag/2.12.d_trillium>.


### Utils

The `Utils` directory provides additional functionality for manipulating the gimbal data, such as coordinate system transformations and unit conversions.

It also contains shim functions for compatibility with the legacy pre-1.3 API. These functions should be considered to be deprecated, however, as they will most likely be removed in a future release.

## Building the SDK

There are three parts to the output of `orion-sdk`'s build process: Two libriaries, `Communications/libOrionComm.a` and `Utils/libOrionUtils.a`, which implement the entire SDK and can be linked into any application, and a series of example applications in the `Examples` directory which are based on those libraries.

### Using Qt/qmake

If [Qt](https://www.qt.io) is installed on the host machine, the libraries and all the example applications can be compiled with `qmake` using the `Public.pro` project file in the root directory. Qt Creator can be used for a graphical interface to `qmake`, or the project can be built on the command line as follows:

```
qmake
make
```

### Using `make`

The SDK and all the examples can also be built by simply running `make` in the root directory. This will invoke the Makefiles in all of the subdirectories and create the libraries and their dependent example applications. It is also possible to use `make` to cross compile for other platforms. An example invocation to build the libraries and examples for an ARM-based processor might look like this:

```
make TARGET=arm CC=arm-none-linux-gnueabi-gcc AR=arm-none-linux-gnueabi-ar
```

### Using MSVC

Also included are solution and project files compatible with Microsoft Visual Studio versions 2013 and later. The solution file `Public.sln` located in the root directory contains the MSVC projects to build the two libraries as well as all of the example applications that depend on those libraries.

## Support and Documentation

Once the API code has been generated by ProtoGen, a complete ICD will be generated at `Communications/OrionPublic.markdown`. If MultiMarkdown is installed, an HTML version of the same ICD will be generated at `Communications/OrionPublic.html`.

For further assistance, please contact Trillium directly through one of the following methods:

* __Email:__ support@trilliumeng.com
* __Phone:__ +1 509.281.3332, ext. 2
