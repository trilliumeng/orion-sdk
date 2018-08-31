# Path Tracking Example Application

The `PathTrack` example demonstrates the use of the gimbal's path tracking mode. It will send up to 15 points, as saved in a file called `path.csv`, to the gimbal for tracking along with some optional configuration parameters.

## Theory of Operation

When the application is run, it will first read data from `path.csv`, which defines a path as follows:

```
point_0_lat, point_0_lon, point_0_hae
point_1_lat, point_1_lon, point_1_hae
[ . . . ]
point_14_lat, point_14_lon, point_14_hae
```

Each point on the path should be a tuple of latitude, longitude – both in units of degrees – and ellipsoid height, specified in meters above the WGS-84 Ellipsoid.

Once the points have been read in, each will be converted to Earth-centered, Earth-fixed (ECEF) coordinates and placed into an `OrionPath_t` structure, which is then encoded into an `OrionPath` message and sent to the gimbal. The application will then continuously print the regularly downlinked path status from the `GeolocateTelemetryCore` message.

## Command-line Parameters

The application takes several optional arguments, which are ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
* __Step Angle__: Angle, in degrees, to step along the path, or omit to disable step-stare mode.
* __Cross Steps__: Number of steps across the path to make when operating in step-stare mode, or omit to disable cross-track stepping.
* __Cross Step Ratio__: Fraction of the step angle to use as a cross-track stepping angle.
