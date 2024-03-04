#include "OrionPublicPacket.h"
#include "OrionComm.h"
#include "GeolocateTelemetry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

static GeolocateTelemetry_t GeoTelem;
static BOOL RxGeoTelem = FALSE;
static TrackOptions_t TrackOptions;
static BOOL RxTrackOptions = FALSE;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv);
static BOOL ProcessData(void);
static void IncreaseTrackSize(void);

int main(int argc, char **argv)
{
    int WaitCount = 0;

    // Process the command line arguments
    ProcessArgs(argc, argv);

    // Request the track options packet
    MakeOrionPacket(&PktOut, getTrackOptionsPacketID(), 0);
    OrionCommSend(&PktOut);

    // Wait for confirmation from the gimbal, or 10 seconds - whichever comes first
    while ((++WaitCount < 10) && (ProcessData() == FALSE)) usleep(100000);

    // Attempt to increase the track box size for object tracking.
    IncreaseTrackSize();

    // If we timed out waiting, tell the user and return an error code
    if (WaitCount >= 50) KillProcess("Gimbal failed to respond", -1);

    // Done
    return 0;

}// main

static BOOL ProcessData(void)
{
    // Loop through any new incoming packets
    while (OrionCommReceive(&PktIn))
    {
        // If this is a response to the request for track option data that was just sent
        if (PktIn.ID == getTrackOptionsPacketID())
        {
            // If the cameras packet decodes properly
            if (decodeTrackOptionsPacketStructure(&PktIn, &TrackOptions))
            {
                RxTrackOptions = TRUE;
                return RxTrackOptions && RxGeoTelem;
            }
        }
        // Or possibly this is a gelocation data record that is also required
        else if (PktIn.ID == getGeolocateTelemetryCorePacketID())
        {
            if (decodeGeolocateTelemetryCorePacketStructure(&PktIn, &GeoTelem.base))
            {
                // If it decoded properly, extrapolate the core data to the full GeolocateTelemetry_t struct
                ConvertGeolocateTelemetryCore(&GeoTelem.base, &GeoTelem);
                RxGeoTelem = TRUE;
                return RxTrackOptions && RxGeoTelem;
            }
        }
    }

    // Haven't gotten the response we're looking for yet
    return FALSE;

}// ProcessData

// This function increases the track box size, but first verifies existing settings have been received.
static void IncreaseTrackSize(void)
{
    if( RxGeoTelem == FALSE || RxTrackOptions == FALSE )
    {
        printf("%s\n", "Could not increase track size without receiving necessary settings.");
        fflush(stdout);
        return;
    }

    // Check if we have data in GeoTelem and that we have an active track
    // This is required as the gimbal may be employing dynamic track sizing
    if(GeoTelem.base.hasTrackData && GeoTelem.base.primaryTrackData.Active)
    {
        TrackOptions.TrackSize = GeoTelem.base.primaryTrackData.Size + 0.01f;
    }
    else{
        // At this point, we have all the other track options settings, just change the trackBox size from packetData
        TrackOptions.TrackSize += 0.01f;
    }

    // Send the Track Options packet.
    encodeTrackOptionsPacketStructure( &PktOut, &TrackOptions );
    OrionCommSend( &PktOut );

    printf("%s\n", "Increased object track size.");
}

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

static void ProcessArgs(int argc, char **argv)
{
    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

}// ProcessArgs
