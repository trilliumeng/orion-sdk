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

This directory contains the ProtoGen XML file and the Makefile/project necessary to build the SDK as a static library. Once the XML file has been processed, the directory will also contain all of the source code for the SDK. It also contains the low-level code for connecting to the gimbal over the Ethernet and/or serial port interfaces. To initiate a connection with a gimbal, one of the following functions must be used:

* `OrionCommOpenSerial`
* `OrionCommOpenNetwork`

Both functions will return `TRUE` upon a successful connection, then `OrionCommSend` and `OrionCommReceive` may be used to send and receive Orion SDK packets. `OrionCommClose` is used to close down the connection and release all the relevant resources.

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

### `GpsAndHeading`

All Orion gimbals support external GPS and heading reference data to assist the INS filter. In many use cases, there is already a GPS-enabled device (e.g., autopilot) on the vehicle and it is useful to share GPS data between that device and the gimbal in order to minimize the number of GPS antennas necessary for operation.

Similarly, if another device has a better heading estimate than the gimbal, its heading data can be sent to the gimbal to be used as a correction in the INS filter. The `GpsAndHeading` application demonstrates both functions.

The `GpsAndHeading` application takes several optional arguments, which are ordered and have defaults as follows:

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

The `GeoPoint` application demonstrates the 'geopoint' operational mode, which locks the gimbal's line of sight on to a commanded LLA position. The application takes several optional arguments, which are ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal, or omit to connect via Ethernet.
* __Lat__: Target latitude, default is 45.7º.
* __Lon__: Target longitude, default is -121.5º.
* __Alt__: Target WGS-84 ellipsoid height, default is 30 m.
* __Vel_N__: Target velocity vector north component, default is 0 m/s.
* __Vel_E__: Target velocity vector east component, default is 0 m/s.
* __Vel_D__: Target velocity vector down component, default is 0 m/s.

When the application is run, it will connect to the gimbal, send the geopoint command, wait for a response, then exit.

### `LineOfSight`

The process of finding the image position via terrain intersection with the terrain is demonstrated in the `LineOfSight` example. The application will connect to a gimbal and listen for `GeolocateTelemetryCore` messages. Once the application has received that data, it will then use a ray-tracing alogrithm to find latitude, longitude and altitude of the gimbal's current image using a terrain model. Note that this example requires an internet connection to function properly. The two optional arguments are:

* __Serial Port__: Serial port connected to gimbal, or omit to connect via Ethernet.
* __LoD__: Terrain model level of detail, from 0 to 15. Note that not all levels of detail are available in all areas.

Running the `LineOfSight` application will cause it to connect to the gimbal and continuously print its image position to the terminal. It will also uplink the computed slant range to the gimbal for its own internal use.

### `PathTrack`

The `PathTrack` example demonstrates the use of the gimbal's path tracking mode. It will send up to 15 points, as saved in a file called `path.csv`, to the gimbal for tracking along with some optional configuration parameters. The application takes several optional arguments, which are ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal, or omit to connect via Ethernet.
* __Step Angle__: Angle, in degrees, to step along the path, or omit to disable step-stare mode.
* __Cross Steps__: Number of steps across the path to make when operating in step-stare mode, or omit to disable cross-track stepping.
* __Cross Step Ratio__: Fraction of the step angle to use as a cross-track stepping angle.

When the application is run, it will connect to the gimbal, send the path command and continuously print the regularly downlinked path status from the `GeolocateTelemetryCore` message.

### `SendCommand`

The `SendCommand` example shows the use of the `OrionCmd` packet, which sets the operational mode of the gimbal. The application takes several optional arguments, which are ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal, or omit to connect via Ethernet.
* __Pan Target__: Pan target position or rate, depending on mode, default is 0 deg/s.
* __Tilt Target__: Tilt target position or rate, depending on mode, default is 0 deg/s.
* __Mode__: Operational mode: `R` for rate, `P` for position, or `D` for disabled, default is rate.
* __Stabilized__: Set to 1 to enable inertial stabilization, default is disabled.
* __Impulse Time__: Time in seconds to run the command (rate only) before zeroing, or set to 0 for a continuous command. Default is 0.

When the application is run, it will connect to the gimbal, send the command, wait for a response, then exit.

### `UserData`

The use of user data passthrough packets is demonstrated in the `UserData` example. The application acts as both a sender and receiver, so two instances can be run to communicate with each other through the gimbal. There are two optional arguments:

* __Serial Port__: Serial port connected to gimbal, or omit to connect via Ethernet.
* __Gimbal Comm Port__: Destination serial port on the gimbal, defaults to 2. Contact support for more details on the port numbering scheme.

Once the application has connected to the gimbal, the application will forward all keyboard input to the specified serial port on the gimbal and monitor the gimbal data stream for incoming user data packets.

## Support and Documentation

Once the API code has been generated by ProtoGen, a complete ICD will be generated at `Communications/OrionPublic.markdown`. If MultiMarkdown is installed, an HTML version of the same ICD will be generated at `Communications/OrionPublic.html`.

For further assistance, please contact Trillium directly through one of the following methods:

* __Email:__ support@trilliumeng.com
* __Phone:__ +1 509.281.3332, ext. 2

## Licensing

The publicly available version of the Orion SDK is licensed under a Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License. For commercial licensing options, please contact Trillium at <sales@trilliumeng.com>.
