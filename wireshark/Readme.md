## Orion Public wireshark lua dissector
This **OrionPublic** lua dissector can assist with packet inspection in wireshark.

## What is it?
The two files desribed below work together to create a complete wireshark packet dissector for Orion Public Protocol.
* **OrionPublic.lua** - Generated using ProtoGen 3
* **OrionPublic_packet.lua**  - Hand written to compliment the generated packet dissector, improving the readability of the packet data


## How to use it:
### Use on a single capture
* 🗹 Copy **OrionPublic_packet.lua** into the same folder as the wireshark pcap. Open the pcap from that folder (double click it)
* 🗹 In wireshark open Tools->Lua Console and paste the contents of **OrionPublic.lua** into the console, evaluate and close the console

### (Optionally✌) Install as a wireshark plugin
* 🗹 Create **plugins** directory for "OrionPublic"
    * `%APPDATA%\Wireshark\plugins\OrionPublic` - Personal
    * `WIRESHARK\plugins\OrionPublic` - Global
    * See [Wireshark Plugin Folders Documentation](https://www.wireshark.org/docs/wsug_html_chunked/ChPluginFolders.html)

* 🗹 Copy the two **OrionPublic** lua files from this directory to the OrionPublic folder under *plugins*
    * ✔ OrionPublic.lua
    * ✔ OrionPublic_packet.lua 

* 🗹 the **OrionPublic** dissector will load on all openings of wireshark