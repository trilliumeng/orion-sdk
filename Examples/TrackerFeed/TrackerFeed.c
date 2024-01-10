#include "OrionPublicPacket.h"
#include "earthposition.h"
#include "OrionComm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv);
static BOOL ProcessData(void);

int main(int argc, char **argv)
{
    int WaitCount = 0;

    
    // Process the command line arguments
    ProcessArgs(argc, argv);


    MakeOrionPacket(&PktOut, getGeolocateTelemetryCorePacketID(), 0);    
    OrionCommSend(&PktOut);

    // Wait for confirmation from the gimbal, or 5 seconds - whichever comes first
    while(WaitCount < 9999999999) {
      while ((++WaitCount < 50) && (ProcessData() == FALSE)) usleep(100000);
    }

    // If we timed out waiting, tell the user and return an error code
    // if (WaitCount >= 50) KillProcess("Gimbal failed to respond", -1);

    // Done
    return 0;

}// main

static BOOL ProcessData(void)
{
    // Loop through any new incoming packets
    while (OrionCommReceive(&PktIn))
    {
        printf("packet recv'd\n");
        // If this is a response to the request we just sent
        if (PktIn.ID == getGeolocateTelemetryCorePacketID())
        {
            printf("  packet matched\n");
            GeolocateTelemetryCore_t core;

            // If the cameras packet decodes properly
            if (decodeGeolocateTelemetryCorePacketStructure(&PktIn, &core))
            {
                printf("    packet decoded\n");
                printf("    pan/tilt:  %1.3f /  %1.3f\n", core.pan, core.tilt);
                printf("    hfov/vfov: %1.3f /  %1.3f\n", core.hfov, core.vfov);
                
                char TypeString[16];
                switch (core.mode) {
                  case ORION_MODE_TRACK: strcpy(TypeString, "Track"); break;
                  default:               strcpy(TypeString, "Unknown"); break;
                }
                printf("    mode: %-7s \n", TypeString);

                return TRUE;
            }
        }
    }

    // Haven't gotten the response we're looking for yet
    return FALSE;

}// ProcessData

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
