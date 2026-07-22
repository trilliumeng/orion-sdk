# Orion SDK Python Examples

Python implementations of the C examples found in `Public/Examples/`.

## Prerequisites

```bash
# Generate the Python SDK first
cd Public
./GenerateOrionPublicPacket.sh  # Linux/Mac
GenerateOrionPublicPacketWin.bat  # Windows
```

## Quick Start

```bash
# Most examples just need the gimbal IP
python camera_info.py 192.168.1.100
python receive_telemetry.py 192.168.1.100
```

---

## Examples

<details>
<summary><b>encode_decode.py</b> - Packet serialization demo (no gimbal needed)</summary>

**C equivalent:** `Examples/EncodeDecode/`

Demonstrates encoding a command packet to bytes and decoding it back. No gimbal connection required.

```bash
python encode_decode.py
```
</details>


<details>
<summary><b>basic_connection.py</b> - Basic connection and commands</summary>

Demonstrates connecting to a gimbal and sending pan/tilt commands.

```bash
python basic_connection.py <gimbal_ip>
python basic_connection.py  # Auto-discover
```
</details>

<details>
<summary><b>receive_telemetry.py</b> - Display live telemetry</summary>

**C equivalent:** `Examples/TrackerFeed/`

Continuously displays gimbal telemetry (pan, tilt, position, FOV).

```bash
python receive_telemetry.py <gimbal_ip>
```
</details>

<details>
<summary><b>csv_receive_telemetry.py</b> - Log telemetry to CSV</summary>

Extended telemetry logging to CSV files for analysis.

```bash
python csv_receive_telemetry.py <gimbal_ip>
```
</details>

<details>
<summary><b>camera_info.py</b> - Display camera specifications</summary>

**C equivalent:** `Examples/CameraInfo/`

Requests and displays camera configuration including type, zoom ratio, and field of view.

```bash
python camera_info.py <gimbal_ip>
python camera_info.py  # Auto-discover
```
</details>

<details>
<summary><b>geo_point.py</b> - Send geopoint command</summary>

**C equivalent:** `Examples/GeoPoint/`

Commands the gimbal to look at a geographic location (lat/lon/alt).

```bash
python geo_point.py <gimbal_ip> [LAT LON ALT] [VN VE VD]

# Examples
python geo_point.py 192.168.1.100
python geo_point.py 192.168.1.100 45.7 -121.5 30.0
python geo_point.py 192.168.1.100 45.7 -121.5 30.0 5.0 3.0 0.0  # With velocity
```
</details>

<details>
<summary><b>gps_and_heading.py</b> - Send external GPS/heading data</summary>

**C equivalent:** `Examples/GpsAndHeading/`

Sends external GPS position and heading data to assist the gimbal's INS filter.

```bash
python gps_and_heading.py <gimbal_ip> [LAT LON ALT] [VN VE VD] [HDG]

# Example
python gps_and_heading.py 192.168.1.100 45.7 -121.5 300 3.0 22.0 -4.0 270
```
</details>

<details>
<summary><b>track_size.py</b> - Adjust object tracking box size</summary>

**C equivalent:** `Examples/TrackSize/`

Increases the tracking box size by 1%. Useful for adjusting object tracking sensitivity.

```bash
python track_size.py <gimbal_ip>
```
</details>

<details>
<summary><b>user_data.py</b> - User data passthrough</summary>

**C equivalent:** `Examples/UserData/`

Sends keyboard input through the gimbal as user data packets. Two instances can communicate through the gimbal.

```bash
python user_data.py <gimbal_ip> [port]

# port: 0=Ethernet, 2=Primary RS-232, 3=FP2, 4=FP1
python user_data.py 192.168.1.100 2
```
</details>

<details>
<summary><b>path_track.py</b> - Path following with waypoints</summary>

**C equivalent:** `Examples/PathTrack/`

Sends a path of waypoints for the gimbal to follow. Reads waypoints from `path.csv`.

```bash
python path_track.py <gimbal_ip> [step_angle] [cross_steps] [cross_ratio]

# Examples
python path_track.py 192.168.1.100
python path_track.py 192.168.1.100 5.0        # 5 degree step-stare
python path_track.py 192.168.1.100 5.0 3 0.5  # With cross-track stepping
```

**path.csv format:**
```csv
# latitude, longitude, altitude
45.523, -122.676, 100
45.524, -122.677, 100
```
</details>

<details>
<summary><b>send_config.py</b> - Upload configuration file</summary>

**C equivalent:** `Examples/SendConfig/`

Uploads gimbal settings from an OrionUi `.orionconfig` file with retry logic.

```bash
python send_config.py <gimbal_ip> <config_file.orionconfig>
```
</details>

<details>
<summary><b>line_of_sight.py</b> - Terrain intersection calculation</summary>

**C equivalent:** `Examples/LineOfSight/`

Calculates where the gimbal's line of sight intersects terrain using elevation data from Cesium Ion. Sends computed slant range back to the gimbal.

**Requirements:**
```bash
pip install requests
```

```bash
python line_of_sight.py <gimbal_ip> [tile_level]

# tile_level: 1-14, higher = more detail (default: 12)
python line_of_sight.py 192.168.1.100
python line_of_sight.py 192.168.1.100 14
```

Terrain tiles are cached in the `cache/` directory.
</details>

<details>
<summary><b>video_player.py</b> - Video capture with metadata</summary>

**C equivalent:** `Examples/VideoPlayer/`

Receives H.264 video stream, parses KLV metadata, and saves snapshots as JPEG.

**Requirements:**
```bash
pip install av pillow
```

```bash
python video_player.py <gimbal_ip> <video_dest_ip> [port] [record_file.ts]

# video_dest_ip: Your machine's IP where video should be sent
python video_player.py 192.168.1.100 192.168.1.50
python video_player.py 192.168.1.100 192.168.1.50 15004 recording.ts
```

**Controls:**
- `S` - Save current frame as JPEG
- `Q` - Quit
</details>

---

## Example Mapping

| Python Example | C Equivalent |
|---------------|--------------|
| `encode_decode.py` | `Examples/EncodeDecode/` |
| `camera_info.py` | `Examples/CameraInfo/` |
| `geo_point.py` | `Examples/GeoPoint/` |
| `gps_and_heading.py` | `Examples/GpsAndHeading/` |
| `track_size.py` | `Examples/TrackSize/` |
| `user_data.py` | `Examples/UserData/` |
| `path_track.py` | `Examples/PathTrack/` |
| `send_config.py` | `Examples/SendConfig/` |
| `line_of_sight.py` | `Examples/LineOfSight/` |
| `video_player.py` | `Examples/VideoPlayer/` |
| `basic_connection.py` | `Examples/SendCommand/` |
| `receive_telemetry.py` | `Examples/TrackerFeed/` |
