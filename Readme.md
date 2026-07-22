# SDK for Orion Gimbals

## Deprecations planned for - **June, 2027**
#### Deprecation of project files: our transition to CMake
* OrionSDK will no longer provide a **Public.sln** for MSVC tooling. 
    * (see section below about **CMake MSVC generation**)
* OrionSDK will no longer provide a **Public.pro** for QMake tooling.
* OrionSDK will no longer provide raw **Makefile**(s)

After the transtition, the Orion SDK will utilize **CMake** natively where possible and fall back to generators when needed.

## Introduction

The Orion SDK implements all the functionality needed to control any of [Trillium Engineering](http://www.trilliumeng.com)'s Orion family of gimbaled camera systems. It also includes a set of example applications which demonstrate the basic paradigms used to connect to and control an Orion gimbal.  

The entire protocol is implemented in a single [ProtoGen](https://github.com/billvaglienti/ProtoGen) XML file. Running the top-level batch/shell scripts will use ProtoGen to generate all the necessary C code and documentation for encoding and decoding binary packets which conform to the API.  

## Getting Started

### Windows Prerequisites

* Visual Studio 2022 or [VS 2022 Build Tools](https://aka.ms/vs/17/release/vs_BuildTools.exe) (toolset v143) — direct installer download
* Windows SDK 10.0.17134+ (included with VS 2022 Build Tools by default)
* Protogen (pre-built binary included at `Protogen/windows/ProtoGen.exe`)

### Linux Prerequisites

* **Compiler toolchain**: make, qmake, gcc
* **Protogen** (pre-built binaries are included for common platforms)
* **Package Dependencies**: `build-essential`, `cmake`, `pkg-config`, `multimedia-devel`
* **Linux**:
    * libffmpeg
        * libavcodec
        * libavformat
        * libavutil
        * libswscale
    * libjpeg
    * libm
* __Optional:__ MultiMarkdown (<http://fletcherpenney.net/multimarkdown>)

On Ubuntu and other Debian-based distributions, MultiMarkdown can also be installed by running `sudo apt-get install libtext-multimarkdown-perl`.  

This optional dependancy will allow a development system to [generate](https://github.com/trilliumeng/orion-sdk/blob/master/Readme.md#support-and-documentation) a html version of the documentation.  Otherwise documentation is only generated into markdown files.  A preview of the documentation generated for the current release is found [here](https://static.trilliumeng.com/ApplicationData/Orion/download.html?device=OrionSDK).  The generated documentation is for the last release version.  It is recommended that you generate the docs when you generate the protocol code and reference it for your development if you are using a pre-released branch of the protocol.

## Repository Contents
The root directory contains the scripts necessary to generate the SDK code with ProtoGen as well as the parent Makefile and project files for building the SDK and example applications. It also includes several subdirectories:

### Communications
This directory contains the ProtoGen XML file and the Makefile/project necessary to build the SDK as a static library. Once the XML file has been processed, the directory will also contain all of the source code for the SDK. It also contains the low-level code for connecting to the gimbal over the Ethernet and/or serial port interfaces. To initiate a connection with a gimbal, one of the following functions must be used:

* `OrionCommOpenSerial` to connect to a gimbal over a specified serial port
* `OrionCommOpenNetwork` to automatically discover and connect to a gimbal over a network connection 
* `OrionCommOpenNetworkIp` to connect to a gimbal with a known IP address over a network connection

All three functions will return `TRUE` upon a successful connection, then `OrionCommSend` and `OrionCommReceive` may be used to send and receive Orion SDK packets. `OrionCommClose` is used to close down the connection and release all the relevant resources.

#### **NEW Extended API** 
The included **OrionComm** APIs have traditionally only supported a single connection. Internally, the state of the connection was managed by static variables.
Starting in version **3.1.0** we have added a set of APIs that do not use static variables. These APIs allow developers to create multiple concurrent connections using the `CommSocket_t` structure.

```C++
    CommSocket_t connection1;
    CommSocket_t connection2;

    // Look for a gimbal using connection1
    OrionCommOpenNetworkEx(&connection1);

    // connect to a separate gimbal at a specific address on connection2
    OrionCommOpenNetworkIpEx("192.3.4.5", &connection2);
```
**The API can also be used to open multiple connections to the same gimbal.** <br>
For the full details see the [MultipleConnections](Examples/MultipleConnections/Readme.md) example.

### Examples
The `Examples` directory contains some applications which demonstrate both the use of the packet SDK as well as the lower-level process of connecting to and exchanging data with a gimbal over both serial and Ethernet. For detailed information on a particular example application, please see the readme included in its subdirectory.  

The VideoPlayer example app has some specific dependencies and will fail to build if they are not met.  Please see the [VideoPlayer Readme](https://github.com/trilliumeng/orion-sdk/tree/master/Examples/VideoPlayer)

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

### Using `CMake`
CMake provides a platform abstracted build. From the root folder (top most **CMakeLists.txt**)
```bash
cmake -B build
cmake --build build --config Release
cmake --install build --prefix install
```
Build artifacts can now be found in the *install* directory.

### Using `Qmake`
If [Qt](https://www.qt.io) is installed on the host machine, the libraries and all the example applications can be compiled with `qmake` using the `Public.pro` project file in the root directory. Qt Creator can be used for a graphical interface to `qmake`, or the project can be built on the command line as follows:

```bash
    qmake
    make
```

### Using `make`
The SDK and all the examples can also be built by simply running `make` in the root directory. This will invoke the Makefiles in all of the subdirectories and create the libraries and their dependent example applications. It is also possible to use `make` to cross compile for other platforms. An example invocation to build the libraries and examples for an ARM-based processor might look like this:

```bash
make TARGET=arm CC=arm-none-linux-gnueabi-gcc AR=arm-none-linux-gnueabi-ar
```

### Using `MSVC`
Also included are solution and project files for Microsoft Visual Studio 2022 (or VS 2022 Build Tools). The solution file `Public.sln` located in the root directory contains the MSVC projects to build the two libraries as well as all of the example applications that depend on those libraries. All projects use toolset v143; earlier versions of Visual Studio are not supported.

From the `Public` directory:

```
MSBuild Public.sln -p:Configuration=Release -p:Platform=Win32
```

Do not use the `-m` flag (parallel build). The solution runs protogen as a pre-build step to generate `OrionPublicPacket.h`; parallel builds start dependent projects before that step completes.

### Using CMake To `Generate an MSVC Project`
We currently provide a **Public.sln** to load natively into Visual Studio. Developers can still use CMake to generate native project files for **OrionSDK**.
```bash
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A Win32 ..
```
__Optionally__ CMake can then be used to build the MSVC project with:
```bash
cmake --build . --config Release
```

## Support and Documentation

Once the API code has been generated by ProtoGen, a complete ICD will be generated at `Communications/OrionPublic.markdown`. If MultiMarkdown is installed, an HTML version of the same ICD will be generated at `Communications/OrionPublic.html`.

For further assistance, please contact Trillium directly through one of the following methods:

* __Email:__ support@trilliumeng.com
* __Phone:__ +1 509.281.3332, ext. 2
