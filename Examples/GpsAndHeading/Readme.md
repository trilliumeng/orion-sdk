# External GPS and Heading Example Application

All Orion gimbals support external GPS and heading reference data to assist the onboard INS filter. In many use cases, there is already a GPS-enabled device (e.g., autopilot) on the vehicle and it is useful to share GPS data between that device and the gimbal in order to minimize the number of GPS antennas necessary for operation.

Similarly, if another device has a better heading estimate than the gimbal, its heading data can be sent to the gimbal to be used as a correction in the INS filter. The `GpsAndHeading` application demonstrates both functions.

## Theory of operation

When the application is run, it will attempt to connect to the gimbal and send the GPS and heading data as specified on the command line. After sending each piece of data, the application will wait up to five seconds for acknowledgement from the gimbal and will print an error to the console and exit if no acknowledgement is received.

## Command-line Parameters

The `GpsAndHeading` application takes several optional arguments, which are ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
* __Lat__: GPS latitude, default is 45.7º.
* __Lon__: GPS longitude, default is -121.5º.
* __Alt__: GPS WGS-84 ellipsoid height, default is 300 m.
* __Vel_N__: GPS velocity vector north component, default is 3 m/s.
* __Vel_E__: GPS velocity vector east component, default is 22 m/s.
* __Vel_D__: GPS velocity vector down component, default is -4 m/s.
* __Hdg__: Platform heading, default is 270º.
