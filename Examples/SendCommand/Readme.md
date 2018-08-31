# Send Command Example Application

The `SendCommand` example shows the use of the `OrionCmd` packet, which sets the operational mode of the gimbal.

## Theory of Operation

When the application is run, it will take any specified command line parameters and use them to populate an `OrionCmd_t` structure. Then the structure will be serialized into a data packet by the `encodeOrionCmdPacket` function and sent to the gimbal. The application will then wait up to 5 seconds for acknowledgement from the gimbal before exiting.

## Command-line Parameters

The application takes several optional arguments, which are ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
* __Pan Target__: Pan target position or rate, depending on mode, default is 0 deg/s.
* __Tilt Target__: Tilt target position or rate, depending on mode, default is 0 deg/s.
* __Mode__: Operational mode: `R` for rate, `P` for position, or `D` for disabled, 'F' for FFC, default is rate.
* __Stabilized__: Set to 1 to enable inertial stabilization, default is disabled.
* __Impulse Time__: Time in seconds to run the command (rate only) before zeroing, or set to 0 for a continuous command. Default is 0.
