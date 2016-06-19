# SDK for Orion Gimbals

## Introduction

The Orion SDK implements all the functionality needed to control any of [Trillium Engineering](http://www.trilliumeng.com)'s Orion family of gimbaled camera systems. It also includes a set of example applications which demonstrate the basic paradigms used to connect to and control an Orion gimbal.

The entire protocol is implemented in a single [ProtoGen](https://github.com/billvaglienti/ProtoGen) XML file. Running the top-level batch/shell scripts will use ProtoGen to generate all the necessary C code for encoding and decoding binary packets which conform to the API.

## Getting Started

Building the Orion SDK for linux has the following prerequisites:

* Qt 5.6 (<http://qt.io>)
* __Optional:__ MultiMarkdown (<http://fletcherpenney.net/multimarkdown>)

On Ubuntu and other Debian-based distributions, MultiMarkdown can also be installed by running `sudo apt-get install libtext-multimarkdown-perl`.

## Repository Contents

The root directory contains the scripts necessary to generate the SDK code with ProtoGen as well as the parent Makefile and project files for building the SDK and example applications. It also includes several subdirectories:

### Communications

This directory contains the ProtoGen XML file and the Makefile/project necessary to build the SDK as a static library. Once the XML file has been processed, the directory will also contain all of the source code for the SDK.

### Examples

Contains some example applications which link to the library in the `Communications` directory and demonstrate the use of the packet SDK as well as the connection process over both serial and Ethernet. See [Example Applications](#example-applications) for more details.

### Protogen

Holds the Protogen executables for various platforms, including:

* Windows
* Mac OS X
* Linux (x86, x64 and ARM)

If the ProtoGen binary will not run, it can be built from the source, at <https://github.com/billvaglienti/ProtoGen>. The version used by this SDK is tagged as 1.4.5.b.

### Utils

The `Utils` directory provides additional functionality for manipulating the gimbal data, such as coordinate system transformations and unit conversions.

It also contains shim functions for compatibility with the legacy pre-1.3 API. These functions should be considered to be deprecated, however, as they will most likely be removed in a future release.

## Building the SDK – OS X and Linux

Once the prerequisites are installed, the SDK and all the examples can be built by simply running `make` in the root directory. This process will generate two static libraries, `Communications/libOrionComm.a` and `Utils/libOrionUtils.a`, which implement the entire SDK and can be linked into any application. It will also build the example applications in the `Examples` directory which are based on those libraries.

## Building the SDK – Windows

TBD

## Example Applications

The `Examples` directory contains some applications which demonstrate both the use of the packet SDK as well as the lower-level process of connecting to and exchanging data with a gimbal.

### `EncodeDecode`

The `EncodeDecode` application is the simplest of the examples and does not require a gimbal to run. It only demonstrates the use of the packet parsing routines by encoding and decoding packets from a static byte buffer.

### `LinuxComm`

All other example applications besides `EncodeDecode` depend on this library, which provides a API to connect to a gimbal and communicate with it over either Ethernet or a serial port. To initiate a connection with a gimbal, one of the following functions must be used:

* `LinuxCommOpenSerial`
* `LinuxCommOpenNetwork`

Both functions return a file descriptor which can then be used with `LinuxCommSend` and `LinuxCommReceive` to send and receive Orion SDK packets. `LinuxCommClose` is used to close down the file descriptor and release all the relevant resources.

### `GpsAndHeading`

All Orion gimbals support external GPS and heading reference data to assist the INS filter. In many use cases, there is already a GPS-enabled device (e.g., autopilot) on the vehicle and it is useful to share GPS data between that device and the gimbal in order to minimize the number of GPS antennas necessary for operation.

Similarly, if another device has a better heading estimate than the gimbal, its heading data can be sent to the gimbal to be used as a correction in the INS filter. The `GpsAndHeading` application demonstrates both functions.

The `GpsAndHeading` application takes several optional arguments, which ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal, or omit to connect via Ethernet.
* __Lat__: GPS latitude, default is 45.7º.
* __Lon__: GPS longitude, default is -121.5º.
* __Alt__: GPS WGS-84 ellipsoid height, default is 300 m.
* __Vel_N__: GPS velocity vector north component, default is 3 m/s.
* __Vel_E__: GPS velocity vector east component, default is 22 m/s.
* __Vel_D__: GPS velocity vector down component, default is -4 m/s.
* __Hdg__: Platform heading, default is 270º.

When the application is run, it will connect to the gimbal, send the GPS and heading data, wait for a response, then exit.

### `GeoPoint`

The `GeoPoint` application demonstrates the 'geopoint' operational mode, which locks the gimbal's line of sight on to a commanded LLA position. The application takes several optional arguments, which ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal, or omit to connect via Ethernet.
* __Lat__: Target latitude, default is 45.7º.
* __Lon__: Target longitude, default is -121.5º.
* __Alt__: Target WGS-84 ellipsoid height, default is 30 m.
* __Vel_N__: Target velocity vector north component, default is 0 m/s.
* __Vel_E__: Target velocity vector east component, default is 0 m/s.
* __Vel_D__: Target velocity vector down component, default is 0 m/s.

When the application is run, it will connect to the gimbal, send the geopoint command, wait for a response, then exit.

## Support and Documentation

Once the API code has been generated by ProtoGen, a complete ICD will be generated at `Communications/OrionPublic.markdown`. If MultiMarkdown is installed, an HTML version of the same ICD will be generated at `Communications/OrionPublic.html`.

For further assistance, please contact Trillium directly through one of the following methods:

* __Email:__ support@trilliumeng.com
* __Phone:__ +1 509.281.3332, ext. 2

## Licensing

The Orion SDK is released under the MIT License. For more licensing options, please contact Trillium at <sales@trilliumeng.com>.