#include "OrionPublicPacket.h"
#include "fielddecode.h"
#include "OrionComm.h"
#include "StreamDecoder.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <jpeglib.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, OrionNetworkVideo_t *pSettings, char *pVideoUrl);
static int ProcessKeyboard(void);
static void SaveJpeg(uint8_t *pData, int Width, int Height, const char *pPath, int Quality);

int main(int argc, char **argv)
{
    uint8_t VideoFrame[1280 * 720 * 3] = { 0 }, MetaData[1024] = { 0 };
    OrionNetworkVideo_t Settings = { 0 };
    char VideoUrl[32] = "";
    int FrameCount = 0;

    // Video port will default to 15004
    Settings.Port = 15004;
    Settings.StreamType = STREAM_TYPE_H264;

    // Process the command line arguments
    ProcessArgs(argc, argv, &Settings, VideoUrl);

    // Send the network video settings
    encodeOrionNetworkVideoPacketStructure(&PktOut, &Settings);
    OrionCommSend(&PktOut);

    // If we can't open the video stream
    if (StreamOpen(VideoUrl) == 0)
    {
        // Tell the user and get out of here
        printf("Failed to open video at %s\n", VideoUrl);
        KillProcess("", 1);
    }
    else
        printf("Press S to capture a snapshot or Q to quit\n");

    // Loop forever
    while (1)
    {
        // Run the MPEG-TS stream processor
        if (StreamProcess())
        {
            // If we got a new frame/metadata pair, print a little info to the screen
            printf("Captured %5d frames\r", ++FrameCount);
        }

        // Switch on keyboard input (if any)
        switch (ProcessKeyboard())
        {
        case 's':
        case 'S':
        {
            int Width, Height, Size;
            char Path[64];

            // If we can read the current frame out of the decoder
            if (StreamGetVideoFrame(VideoFrame, &Width, &Height, sizeof(VideoFrame)))
            {
                // Create a file path based on the current frame index
                sprintf(Path, "%05d.jpg", FrameCount);

                // Now save the image as a JPEG
                SaveJpeg(VideoFrame, Width, Height, Path, 75);

                // Print some confirmation to stdout
                printf("\nSaved file %s\n", Path);
            }

            // If we can read a KLV UAS data packet out of the decoder
            if (StreamGetMetaData(MetaData, &Size, sizeof(MetaData)))
            {
                // TODO: Add metadata parsing
            }
            
            break;
        }

        case 'q':
        case 'Q':
            KillProcess("Exiting...", 0);
            break;

        default:
            break;
        };

        // Flush the stdout buffer and sleep for roughly half a frame time
        fflush(stdout);
        usleep(15000);
    }

    // Done (actually, we'll never get here...)
    return 0;

}// main

static void SaveJpeg(uint8_t *pData, int Width, int Height, const char *pPath, int Quality)
{
    FILE *pFile;

    // If we manage to open the file we're trying to write to
    if ((pFile = fopen(pPath, "wb")) != NULL)
    {
        struct jpeg_compress_struct Info;
        struct jpeg_error_mgr Error;

        // Not sure why this has to happen first...
        Info.err = jpeg_std_error(&Error);

        // Initialize the compressor subsystem
        jpeg_create_compress(&Info);
        jpeg_stdio_dest(&Info, pFile);

        // Populate some information regarding the image format
        Info.image_width      = Width;
        Info.image_height     = Height;
        Info.input_components = 3;
        Info.in_color_space   = JCS_RGB;

        // Now initialize all the internal stuff to defaults
        jpeg_set_defaults(&Info);

        // It's definitely called fastest for a reason... May want to disable this, though, for quality's sake
        Info.dct_method = JDCT_FASTEST;

        // Set quality and get to compressin'
        jpeg_set_quality(&Info, Quality, 1);
        jpeg_start_compress(&Info, 1);

        // Allocate a scanline array
        JSAMPARRAY pScanLines = (JSAMPARRAY)malloc(Height * sizeof(JSAMPROW));

        // For each scanline in the image
        for (unsigned int i = 0; i < Height; i++)
        {
            // Point this scanline row to the appropriate row in the input data
            pScanLines[i] = &pData[i * Info.image_width * Info.input_components];
        }

        // Write the JPEG data to disk
        jpeg_write_scanlines(&Info, pScanLines, Height);

        // Free the scanline array
        free(pScanLines);

        // Finally, finish and close the file
        jpeg_finish_compress(&Info);
        jpeg_destroy_compress(&Info);
        fclose(pFile);
    }

}// SaveJpeg

// This function just shuts things down consistently with a nice message for the user
static void KillProcess(const char *pMessage, int Value)
{
    // Print out the error message that got us here
    printf("%s\n", pMessage);
    fflush(stdout);

    // Close down the active file descriptors
    OrionCommClose();

    // Finally exit with the proper return value
    exit(Value);

}// KillProcess

static void ProcessArgs(int argc, char **argv, OrionNetworkVideo_t *pSettings, char *pVideoUrl)
{
    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    switch (argc)
    {
    case 3:
        pSettings->Port = atoi(argv[2]);
    case 2:
    {
        uint8_t Octets[4];

        if (sscanf(argv[1], "%3hhu.%3hhu.%3hhu.%3hhu", &Octets[0], &Octets[1], &Octets[2], &Octets[3]))
        {
            int Index = 0;
            pSettings->DestIp = uint32FromBeBytes(Octets, &Index);
            sprintf(pVideoUrl, "udp://%s:%d", argv[1], pSettings->Port);
        }
        break;
    }
    default:
        printf("USAGE: %s [/path/to/serial | gimbal_ip] video_ip [port]\n", argv[0]);
        KillProcess("", 1);
        break;
    };

}// ProcessArgs

#ifdef _WIN32
# include <conio.h>
#else
# include <termios.h>
#endif // _WIN32

// Look for a keypress from the user
static int ProcessKeyboard(void)
{
#ifdef _WIN32
    return (_kbhit() == 0) ? 0 : _getch();
#else
    struct termios Old, New;
    char c = 0;

    // If we can get the current attributes for stdin
    if (tcgetattr(fileno(stdin), &Old) >= 0)
    {
        // Copy the current attributes into a new structure
        New = Old;

        // Turn off the echo and canonical output and disable blocking
        New.c_lflag &= ~(ICANON | ECHO);
        New.c_cc[VMIN] = New.c_cc[VTIME] = 0;

        // If we can successfully overwrite the current settings
        if (tcsetattr(fileno(stdin), TCSANOW, &New) >= 0)
        {
            // Try reading from stdin
            if (read(fileno(stdin), &c, 1) != 1)
            {
                // If there's some sort of error, clear whatever came out of read()
                c = 0;
            }
        }

        // Finally, revert the stdin settings to what they were before we were called
        tcsetattr(fileno(stdin), TCSANOW, &Old);
    }

    // And last but not least, return the character we read from stdin (or NULL for nothing)
    return c;
#endif // _WIN32

}// ProcessKeyboard
