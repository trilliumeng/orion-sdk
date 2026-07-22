#include "OrionComm.h"
#include "OrionPublicPacket.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

// A few helper functions, etc.
static void KillProcess(const char *pMessage, int Value, CommSocket_t *gimbal1, CommSocket_t *gimbal2);
static int ProcessArgs(int argc, char **argv, CommSocket_t *gimbal1, CommSocket_t *gimbal2);
static BOOL ProcessData(CommSocket_t *commSocket);
static BOOL ProcessDataSingle(void);
static BOOL TestSingleConnection(const char *label);

int main(int argc, char **argv)
{
    int WaitCount = 0;

    // Create the communication socket struct
    CommSocket_t gimbal1 = {0};
    CommSocket_t gimbal2 = {0};
    memset(&PktOut, 0, sizeof(OrionPkt_t));
#ifdef _WIN32
    if (WSAStartup(MAKEWORD(2,2), &gimbal1.WSAData) != 0) return 1;
#endif
    // Process the command line arguments
    int argsCheck  = ProcessArgs(argc, argv, &gimbal1, & gimbal2);
    if(argsCheck == 1 ) {
        KillProcess("Failed to connect to Gimbal 1", -1, &gimbal1, & gimbal2);
    }
    if(argsCheck == 2) {
        KillProcess("Failed to connect to Gimbal 2", -1, &gimbal1, & gimbal2);
    }
    printf("Connected to two separate Gimbals!\n\n");
    fflush(stdout);
    // Request the camera settings packet
    printf("Sending request Camera Information to each connection\n\n");

    MakeOrionPacket(&PktOut, getOrionCamerasPacketID(), 0);
    OrionCommSendEx(&PktOut, &gimbal1);
    OrionCommSendEx(&PktOut, &gimbal2);

    // Wait for confirmation from the gimbal, or 5 seconds - whichever comes first
    BOOL waitingConnection1 = TRUE;
    BOOL waitingConnection2 = TRUE;
    while ((++WaitCount < 50) && (waitingConnection1 || waitingConnection2)){
        if(waitingConnection1) {
            waitingConnection1 = !ProcessData(&gimbal1);
            if(!waitingConnection1) {
                printf("Received Camera Data for Gimbal 1 on %s\n\n", inet_ntoa(gimbal1.SocketStruct.sin_addr));
                fflush(stdout);
            }
        }
        if(waitingConnection2) {
            waitingConnection2 = !ProcessData(&gimbal2);
            if(!waitingConnection2) {
                printf("Received Camera Data for Gimbal 2 on %s\n\n", inet_ntoa(gimbal2.SocketStruct.sin_addr));
                fflush(stdout);
            }
        }
        usleep(100000);
    }

    // If we timed out waiting, tell the user and return an error code
    if (WaitCount >= 50 || waitingConnection1 || waitingConnection2){
        KillProcess("Example did not properly decode the camera info data", 4, &gimbal1, & gimbal2);
    }

    int failures = 0;

    // Test 1: Basic single-connection open/send/receive
    printf("\nTest 1: single-connection to %s\n", argv[1]);
    fflush(stdout);
    if (OrionCommOpenNetworkIp(argv[1]))
    {
        if (!TestSingleConnection("Test 1"))
            failures++;
        OrionCommClose();
    }
    else { printf("Test 1: FAILED to connect\n"); failures++; }

    // Test 2: Reconnect — close and reopen, verify clean state
    printf("\nTest 2: reconnect to %s\n", argv[1]);
    fflush(stdout);
    if (OrionCommOpenNetworkIp(argv[1]))
    {
        if (!TestSingleConnection("Test 2"))
            failures++;
        OrionCommClose();
    }
    else { printf("Test 2: FAILED to connect\n"); failures++; }

    // Test 3: Double-open — open again without closing, verify auto-close
    printf("\nTest 3: double-open to %s (no close first)\n", argv[1]);
    fflush(stdout);
    if (OrionCommOpenNetworkIp(argv[1]))
    {
        // Open again without closing — should auto-close the first connection
        if (OrionCommOpenNetworkIp(argv[1]))
        {
            if (!TestSingleConnection("Test 3"))
                failures++;
            OrionCommClose();
        }
        else { printf("Test 3: FAILED to re-open\n"); failures++; }
    }
    else { printf("Test 3: FAILED to connect\n"); failures++; }

    // Clean up multi-connection sockets
    OrionCommCloseEx(&gimbal1);
    OrionCommCloseEx(&gimbal2);

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL TESTS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;

}// main

static BOOL ProcessData(CommSocket_t *commSocket)
{
    // Loop through any new incoming packets
    while (OrionCommReceiveEx(&PktIn, commSocket))
    {
        // If this is a response to the request we just sent
        if (PktIn.ID == getOrionCamerasPacketID())
        {
            OrionCameras_t Cameras;

            // If the cameras packet decodes properly
            if (decodeOrionCamerasPacketStructure(&PktIn, &Cameras))
            {
                int i;

                // Print a header row to stdout
                printf(" Index  Type     Zoom  WFOV  NFOV\n");
                printf("----------------------------------\n");

                // Loop through each camera in the array
                for (i = 0; i < Cameras.NumCameras; i++)
                {
                    OrionCamSettings_t *pSettings = &Cameras.OrionCamSettings[i];
                    float ArraySize = pSettings->PixelPitch * pSettings->ArrayWidth;
                    float Zoom = 1.0f, Wfov, Nfov;
                    char TypeString[16];

                    // If this camera doesn't exist, skip it
                    if (pSettings->Type == CAMERA_TYPE_NONE)
                        continue;

                    // Build a type string based on the type enumeration
                    switch (pSettings->Type)
                    {
                    case CAMERA_TYPE_VISIBLE: strcpy(TypeString, "Visible"); break;
                    case CAMERA_TYPE_SWIR:    strcpy(TypeString, "SWIR");    break;
                    case CAMERA_TYPE_MWIR:    strcpy(TypeString, "MWIR");    break;
                    case CAMERA_TYPE_LWIR:    strcpy(TypeString, "LWIR");    break;
                    default:                  strcpy(TypeString, "Unknown"); break;
                    }

                    // Calculate max zoom ratio for use in OrionCameraState, avoiding (unlikely) divide by zero
                    if (pSettings->MinFocalLength > 0)
                        Zoom = pSettings->MaxFocalLength / pSettings->MinFocalLength;

                    // Compute wide and narrow horizontal FOV in radians
                    Wfov = atan2f(0.5f * ArraySize, pSettings->MinFocalLength) * 2.0f;
                    Nfov = atan2f(0.5f * ArraySize, pSettings->MaxFocalLength) * 2.0f;

                    // Print the index, type, max zoom, and min/max FOV in degrees for this camera
                    printf(" %5d  %-7s %5.1f %5.1f %5.1f\n", i, TypeString, Zoom, degreesf(Wfov), degreesf(Nfov));
                }

                // Packet decoded: Mission accomplished
                return TRUE;
            }
        }
    }

    // Haven't gotten the response we're looking for yet
    return FALSE;

}// ProcessData

// Single-connection API version of ProcessData
static BOOL ProcessDataSingle(void)
{
    OrionPkt_t Pkt;

    while (OrionCommReceive(&Pkt))
    {
        if (Pkt.ID == getOrionCamerasPacketID())
        {
            OrionCameras_t Cameras;
            if (decodeOrionCamerasPacketStructure(&Pkt, &Cameras))
                return TRUE;
        }
    }
    return FALSE;

}// ProcessDataSingle

// Send a request and wait for a response using the single-connection API
static BOOL TestSingleConnection(const char *label)
{
    int WaitCount = 0;

    OrionCommSend(&PktOut);
    while (++WaitCount < 50)
    {
        if (ProcessDataSingle())
        {
            printf("%s: received camera data - OK\n", label);
            return TRUE;
        }
        usleep(100000);
    }
    printf("%s: TIMEOUT\n", label);
    return FALSE;

}// TestSingleConnection

// This function just shuts things down consistently with a nice message for the user
static void KillProcess(const char *pMessage, int Value, CommSocket_t *gimbal1, CommSocket_t* gimbal2)
{
    // Print out the error message that got us here
    printf("%s\n", pMessage);
    fflush(stdout);

    // Close down the active file descriptors
    OrionCommCloseEx(gimbal1);
    OrionCommCloseEx(gimbal2);

    // Finally exit with the proper return value
    exit(Value);

}// KillProcess

static int ProcessArgs(int argc, char **argv, CommSocket_t *gimbal1, CommSocket_t* gimbal2)
{
    // If we can't connect to a gimbal, kill the app right now
    char* gimbal1IP;
    char* gimbal2IP;
    if (argc >= 3)
    {
        gimbal1IP = argv[1];
        gimbal2IP = argv[2];
        // IP address...?
        if (OrionCommIpStringValid(gimbal1IP) && OrionCommIpStringValid(gimbal2IP))        {
            // Try connecting to a gimbal at this IP
            if(!OrionCommOpenNetworkIpEx(gimbal1IP, gimbal1)){
                return 1;
            }
            if(!OrionCommOpenNetworkIpEx(gimbal2IP, gimbal2)) {
                return 2;
            }
        }else {
            KillProcess("One of the IPs is invalid", 1, gimbal1, gimbal2);
        }
    }
    else if (argc == 2)
    {
        // Single IP: connect both Ex sockets to the same gimbal
        gimbal1IP = argv[1];
        if (OrionCommIpStringValid(gimbal1IP)) {
            if(!OrionCommOpenNetworkIpEx(gimbal1IP, gimbal1)){
                return 1;
            }
            if(!OrionCommOpenNetworkIpEx(gimbal1IP, gimbal2)) {
                return 2;
            }
        } else {
            KillProcess("IP is invalid", 1, gimbal1, gimbal2);
        }
    }
    else {
        printf("Usage: %s <ip> [ip2]\n", argv[0]);
        printf("  1 IP:  3 connections to same gimbal (2 multi + 1 single)\n");
        printf("  2 IPs: 2 multi connections + 1 single to first IP\n");
        exit(1);
    }
    return 0; // success
}// ProcessArgs

