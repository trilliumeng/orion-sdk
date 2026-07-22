# Multiple Connections Example Application

The `Multiple Connections` example application demonstrates how to connect to and request data from multiple gimbals or create multiple connections to the same Gimbal.

## Theory of Operation
The focus of this example is to demonstrate the new Extended APIs.

### NEW APIs used
* `OrionCommOpenNetworkIpEx` - Connects to an IP using a given `CommSocket_t` and IP address
* `OrionCommSendEx` - Sends the `OrionPkt_t` out to the connection associated with the given `CommSocket_t`.
* `OrionCommCloseEx` - closes the connection if it is open and cleans up the `CommSocket_t` variable state

**Once connected to both gimbals, this application will**:
* serialize a zero-length request packet which will prompt the gimbals to respond with an `OrionCameras` message.
* The example code will then wait up to 5 seconds for a response
* use `decodeOrionCamerasPacketStructure` to deserialize the response packet
* The example then prints the index, type, maximum zoom ratio, and horizontal field of view limits of each camera, currently installed in the gimbal (per connection).

**If there were no errors the example continues on to exercise the traditional API (static).**
* Tests single connection open/send/receive
* Tests reconnecting on the singular connection
* Tests calling open twice using the static API

## Command-line Parameters
There are two arguments:
* __IP Address1__: Known IP address for Ethernet connection 1 – Gimbal 1
* __IP Address2__: Known IP address for Ethernet connection 2 – Gimbal 2
    *  Can be the same as gimbal 1
