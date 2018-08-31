# Camera Info Example Application

The `CameraInfo` example application demonstrates how to request data on the cameras installed in the gimbal.

## Theory of Operation

Once connected to the gimbal, this application will serialize a zero-length request packet which will prompt the gimbal to respond with an `OrionCameras` message. It will then wait up to 5 seconds for a response; use `decodeOrionCamerasPacketStructure` to deserialize the response packet; then print the index, type, maximum zoom ratio, and horizontal field of view limits of each camera currently installed in the gimbal.

## Command-line Parameters

There are two optional arguments:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
