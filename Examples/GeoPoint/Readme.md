# GeoPoint Example Application

The `GeoPoint` application demonstrates the 'geopoint' operational mode, which fixes the gimbal's line of sight on to a commanded LLA position. As the gimbal's position and attitude changes, the video's ground location will not change.

## Theory of Operation

A 'geopoint' is specified by a position (in lat/lon/alt form), as well as a velocity (in north/east/down form). When run, this application will parse any command-line parameters into a position and velocity geopoint command and print the command to the console. It will then use the `encodeGeopointCmdPacket` function to serialize the command into a data packet and send it to the gimbal, waiting up to 5 seconds for acknowledgement.

## Command-line Parameters

The application takes several optional arguments, which are ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
* __Lat__: Target latitude, default is 45.7º.
* __Lon__: Target longitude, default is -121.5º.
* __Alt__: Target WGS-84 ellipsoid height, default is 30 m.
* __Vel_N__: Target velocity vector north component, default is 0 m/s.
* __Vel_E__: Target velocity vector east component, default is 0 m/s.
* __Vel_D__: Target velocity vector down component, default is 0 m/s.
