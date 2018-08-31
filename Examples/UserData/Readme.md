# User Data Example Application

The use of user data passthrough packets is demonstrated in the `UserData` example. The application acts as both a sender and receiver, so two instances can be run to communicate with each other through the gimbal. 

## Theory of Operation

Once the application has connected to the gimbal, it will capture all keyboard input and periodically encode it into an `OrionUserData` message which is then sent to the gimbal. Upon receipt of that packet, the gimbal will forward it to the serial port specified on the command line.

At the same time, the application will constantly monitor the incoming gimbal data stream for any `OrionUserData` messages that have been forwarded to its connection and print the contents to the console.

As an example of how to connect two applications through the gimbal, assume "App 1" is connected to a gimbal over Ethernet at IP 192.168.3.29 and "App 2" is connected to the primary RS232 connection via `/dev/ttyS0`. The necessary commands would be as follows:

__App 1:__
```
./UserData 192.168.3.29 2
```

__App 2:__
```
./UserData /dev/ttyS0 0
```

Note that the value for the destination comm port should be an integer – see the definition of the `OrionUserDataPort_t` enumeration for details.

## Command-line Parameters

There are three optional arguments:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
* __Gimbal Comm Port__: Destination serial port on the gimbal, defaults to `USER_DATA_PORT_PRIMARY`.
