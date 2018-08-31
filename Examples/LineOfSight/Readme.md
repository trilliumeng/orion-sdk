# Line of Sight Example Application

The process of finding the image position by estimating the terrain intersection location is demonstrated in the `LineOfSight` example. 

## Theory of Operation

The application will connect to a gimbal and listen for `GeolocateTelemetryCore` messages, which includes all the data necessary to geo-reference the video imagery. Once the application has received that data, it will then use a ray-marching alogrithm (`getTerrainIntersection` from `Utils/GeolocateTelemetry.c`) to estimate the LLA position of the center gimbal's current line of sight using a terrain model.

Running the `LineOfSight` application will cause it to connect to the gimbal and continuously print its image position to the terminal. It will also uplink the computed slant range to the gimbal for its own internal use.

__NOTE:__ This example does not currently work in Windows or without an internet connection, as it uses a shell script (which itself uses `curl` and `gunzip`) to fetch terrain model data from the internet.

## Command-line Parameters

The three optional arguments are:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
* __LoD__: Terrain model level of detail, from 0 to 15 (default 12). Higher LoDs improve accuracy but are not available in all areas.
