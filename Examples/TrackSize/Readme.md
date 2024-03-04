# Track Size Example Application

The `Track Size` example application demonstrates how to adjust the track box size when object tracking.

## Theory of Operation

Once connected to the gimbal, this application will serialize a zero-length request packet which will prompt the gimbal to respond with a `TrackOptions` message.
It will then wait up to 5 seconds for a response and use `decodeTrackOptionsPacketStructure` to deserialize the response packet.
The application will then wait for receipt of a GeolocateTelemetry packet prior to sending command to increase the track box size, populating a new packet using `encodeTrackOptionsPacketStructure`. 

Receipt of the existing track options and telemetry is important when sending track box size changes, ensuring expected results and that other track settings are not changed unwittingly.

## Command-line Parameters

There are two optional arguments:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
