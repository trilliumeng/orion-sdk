# Video Player Example Application

The `VideoPlayer` application demonstrates the use of the `ffmpeg` libraries to decode the gimbal's native video and embedded metadata stream, 

## Dependencies

This application depends on the `ffmpeg` and `libjpeg` libraries in order to build, both of which are available through most package managers:

__apt:__
```
sudo apt install libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libjpeg-dev
```

__yum:__
```
yum install ffmpeg-devel libjpeg-devel
```

__Homebrew:__
```
brew install ffmpeg jpeg
```

## Theory of Operation

Once the `VideoPlayer` application connects to the gimbal, it will configure an `OrionNetworkVideo_t` structure with the parameters necessary to stream video to the host computer, serialize that data using `encodeOrionNetworkVideoPacketStructure`, then send it to the gimbal.

At this point, it will open the specified port on the host computer and begin capturing video frames and metadata. As new video frames arrive, the application will decompress the data and store the latest frame in memory. All incoming KLV metadata is parsed and stored in an associative array for later retrieval.

When the user presses the 'S' key on the keyboard, the application will then extract the latest video frame and metadata from the stream parser, compress the video as a JPEG, convert the metadata into EXIF info, and save everything out to disk. The user can also press 'Q' to quit at any time.

## Command-line Parameters

The `VideoPlayer` application takes several optional arguments, which are ordered and have defaults as follows:

* __Serial Port__: Serial port connected to gimbal – omit to connect via Ethernet.
* __IP Address__: Known IP address for Ethernet connection – omit to attempt to auto-detect.
* __Video IP__: Video destination IP (typically the host's address).
* __Video Port__: Video destination port, default is 15004.
* __Record Path__: Filepath for saving recorded video, omit to disable recording.
