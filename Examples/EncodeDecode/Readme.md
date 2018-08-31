# Encode/Decode Example Application

The `EncodeDecode` application is the simplest of the examples and does not require a gimbal connection to run. It only demonstrates the use of the packet parsing routines by encoding and decoding packets from a static byte buffer.

## Theory of Operation

First, a simple OrionCmd message structure (see also the `SendCommand` example application) is populated with some generic values and encoded into `PktOut`, a data structure containing the serialized packet data ready for transmitting, by the function `encodeOrionCmdPacket`.

Next, the data in `PktOut` is copied into a generic data buffer, much like one that was received from a serial port or network socket. After that, the `ProcessData` function steps through the buffer byte-by-byte, looking for an Orion packet using `LookForOrionPacketInByte`. Once a packet is found, the data is deserialized back into an `OrionCmd_t` structure using the `decodeOrionCmdPacket` function and the decoded command message contents are printed to the console before exiting the application.

## Command-line Parameters

* None
