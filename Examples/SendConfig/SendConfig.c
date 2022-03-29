#include "OrionPublicPacket.h"
#include "earthposition.h"
#include "OrionComm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn;

// A few helper functions, etc.
static void KillProcess(int ExitCode, const char *pFormat, ...);
static void ProcessArgs(int argc, char **argv);
static BOOL ProcessData(void);

// Allow up to 5 retries before giving up
#define MAX_RETRIES 5

// Doubly-linked list of OrionPkt_t's
typedef struct OrionPktList_t
{
    OrionPkt_t Pkt;
    int Retries;
    struct OrionPktList_t *pPrev;
    struct OrionPktList_t *pNext;
} OrionPktList_t;

// Packet list manipulation functions
static OrionPktList_t *CreateNode(void);
static void InsertNode(OrionPktList_t **pList, OrionPktList_t *pNode);
static OrionPktList_t *TakeNode(OrionPktList_t **pIterator);
static OrionPktList_t *FindNode(OrionPktList_t *pIterator, const OrionPkt_t *pPkt);

static int CheckStatus(void);

// List of packets to send and failed packets
static OrionPktList_t *pPktList = NULL, *pFailedPkts = NULL;

// Number of packets to send and number complete
static int PktCount = 0, PktsDone = 0;

int main(int argc, char **argv)
{
    OrionPktList_t *pIterator;

    // Process the command line arguments
    ProcessArgs(argc, argv);

    // Start at the root node now that the list has (maybe) been populated by ProcessArgs
    pIterator = pPktList;

    // As long as we have a node to process
    while (pIterator != NULL)
    {
        // If we haven't hit our retry limit for this packet
        if (pIterator->Retries < MAX_RETRIES)
        {
            // Don't send packet 0x20 unless/until there's nothing left to send
            if ((pIterator->Pkt.ID != ORION_PKT_PRIVATE_20) || (pIterator->pNext == pIterator->pPrev))
            {
                // Send this packet and increment its retry counter
                OrionCommSend(&pIterator->Pkt);
                pIterator->Retries++;
            }
        }
        // This packet timed out
        else
        {
            // Update the pointer to the list of all packets if we are removing the first element.
            if (pIterator == pPktList)
                pPktList = pIterator->pNext;

            // Pull the node from the list and add it to the list of failed packets
            InsertNode(&pFailedPkts, TakeNode(&pIterator));
            continue;
        }

        // Look for responses; if all the packets were acked, break out of the loop
        if (ProcessData() == TRUE)
            break;
        // Otherwise move on to the next node, looping around to the top when we hit the end
        else
            pIterator = (pIterator->pNext) ? pIterator->pNext : pPktList;

        // Sleep for 50ms to give the system some room to breathe
        usleep(50000);
    }

    // Toss out a newline before printing any failed packet IDs or exiting
    printf("\n");

    // Start at the root node now that the list has (maybe) been populated by ProcessArgs
    pIterator = pFailedPkts;

    if (pIterator == NULL)
        printf("All packets sent successfully!\n");
    else
    {
        // Print the packets that weren't acked.
        printf("The following packet IDs failed to send:\n");
        while (pIterator != NULL)
        {
            printf("0x%02x\n", pIterator->Pkt.ID);
            pIterator = pIterator->pNext;
        }

    }

    // Get out of here!
    return 0;

}// main

static BOOL ProcessData(void)
{
    // Loop through any new incoming packets
    while (OrionCommReceive(&PktIn))
    {
        // Look to see if this is a response to one of our packets
        OrionPktList_t *pNode = FindNode(pPktList, &PktIn);

        // If this is a valid response
        if (pNode != NULL)
        {
            // Update the pointer to the list of all packets if we are removing the first element.
            if (pNode == pPktList)
                pPktList = pNode->pNext;

            // Pull the corresponding node from the list, free it, and increment the number of valid acks
            free(TakeNode(&pNode));
            PktsDone++;
        }
    }

    // Update the status output, and return if we've gotten all the responses we'd expect
    return CheckStatus();

}// ProcessData

// This function just shuts things down consistently with a nice message for the user
static void KillProcess(int ExitCode, const char *pFormat, ...)
{
    va_list pArgs;

    // Close down the active file descriptors
    OrionCommClose();

    // Print out the error message that got us here
    va_start(pArgs, pFormat);
    vprintf(pFormat, pArgs);
    va_end(pArgs);

    // Add a newline and flush the stdout buffer
    printf("\n");
    fflush(stdout);

    // Finally exit with the proper return value
    exit(ExitCode);

}// KillProcess

static void ProcessArgs(int argc, char **argv)
{
    OrionPktList_t *pNode = CreateNode();

    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess(1, "");

    // If we have a file path argument
    if (argc == 2)
    {
        // Open the specified file
        FILE *pFile = fopen(argv[1], "rb");

        // If the file opened successfully
        if (pFile != NULL)
        {
            UInt8 Byte;

            // As long as we can keep reading bytes in
            while (fread(&Byte, 1, 1, pFile) == 1)
            {
                // If this byte completes a packet
                if (LookForOrionPacketInByte(&pNode->Pkt, Byte))
                {
                    printf("Found OrionPacket ID 0x%02x in file\n", pNode->Pkt.ID);

                    // Add the current node to the list
                    InsertNode(&pPktList, pNode);

                    // Allocate a new node for the next packet and increment the packet counter
                    pNode = CreateNode();
                    PktCount++;
                }
            }

            // If our list is empty, we can't process anything so bail out
            if (pPktList == NULL)
                KillProcess(1, "No Orion packets found in %s", argv[1]);
        }
        // Can't do much without a file
        else
            KillProcess(1, "Failed to open file %s", argv[1]);
    }
    // Kill the application and print the usage info
    else
        KillProcess(1, "USAGE: %s [/dev/ttyXXX | X.X.X.X] input_file.orionconfig", argv[0]);

}// ProcessArgs


static OrionPktList_t *CreateNode(void)
{
    // Allocate a zeroed-out node and return
    return (OrionPktList_t *)calloc(1, sizeof(OrionPktList_t));

}// CreateNode

static void InsertNode(OrionPktList_t **pList, OrionPktList_t *pNode)
{
    // If the head of the list is null (i.e., list is currently empty)
    if (*pList == NULL)
    {
        // Set this node to be the head of the list
        *pList = pNode;

        // Nullify the node's links
        pNode->pNext = pNode->pPrev = NULL;
    }
    else
    {
        // Grab an iterator, starting at the head of the list
        OrionPktList_t *pIterator = *pList;

        // Find the end of the list
        while (pIterator->pNext != NULL)
            pIterator = pIterator->pNext;

        // Insert the node by updating its links and pointing the parent node to this one
        pIterator->pNext = pNode;
        pNode->pPrev = pIterator;
        pNode->pNext = NULL;
    }

}// InsertNode

static OrionPktList_t *TakeNode(OrionPktList_t **pIterator)
{
    // Default return value is NULL
    OrionPktList_t *pNode = NULL;

    // If the passed in node pointer is valid
    if (*pIterator != NULL)
    {
        // Update the neighbor nodes' link pointers to pretend like this node never existed
        if ((*pIterator)->pPrev)
            (*pIterator)->pPrev->pNext = (*pIterator)->pNext;
        if ((*pIterator)->pNext)
            (*pIterator)->pNext->pPrev = (*pIterator)->pPrev;

        // Save the current node pointer for returning
        pNode = *pIterator;

        // Move the iterator forward
        *pIterator = pNode->pNext;

        // Now clear out the removed node's link pointers
        pNode->pPrev = pNode->pNext = NULL;
    }

    // Return the node taken from the list (or NULL if it's not found)
    return pNode;

}// TakeNode

static OrionPktList_t *FindNode(OrionPktList_t *pIterator, const OrionPkt_t *pPkt)
{
    // As long as there's a node to inspect
    while (pIterator != NULL)
    {
        // If this node's packet ID and first byte (often an index byte) match, return it
        if ((pIterator->Pkt.ID == pPkt->ID) && (pIterator->Pkt.Data[0] == pPkt->Data[0]))
            return pIterator;
        // Otherwise move on to the next node
        else
            pIterator = pIterator->pNext;
    }

    // If we make it here, pPkt [most likely] isn't a response to one of our packets
    return NULL;

}// FindNode

static int CheckStatus(void)
{
    int Progress = (PktsDone * 60 + (PktCount / 2)) / PktCount, i;

    // Move back to the start of the line
    printf("\r[");

    // Print a progress bar
    for (i = 1; i <= 60; i++)
        printf("%c", (i >= Progress) ? ' ' : '=');

    // Now print the status
    printf("] (%2d/%2d)", PktsDone, PktCount);
    fflush(stdout);

    // Return 1 if we've gotten all the acks we'd expect
    return PktsDone == PktCount;

}// CheckStatus
