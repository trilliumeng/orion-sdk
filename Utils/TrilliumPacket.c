#include "TrilliumPacket.h"

// Running checksum calculation functions
static void InitChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB);
static void UpdateChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB);

BOOL LookForTrilliumPacketInByte(TrilliumPkt_t *pPkt, UInt16 Sync, UInt8 Byte)
{
    // Update the checksum tracker for every byte after the very first
    if ((pPkt->Index > 0) && (pPkt->Index < ((UInt16)pPkt->Length + 4)))
        UpdateChecksum(Byte, &pPkt->Check0, &pPkt->Check1);

    // Decide what to do based on our handy dandy state tracking variable
    switch (pPkt->Index)
    {
    // Sync byte 0
    case 0:
        // If this byte matches the first sync byte
        if (Byte == (UInt8)(Sync >> 8))
        {
            // Initialize the checksum to be this first byte and increment the state
            InitChecksum(Byte, &pPkt->Check0, &pPkt->Check1);
            pPkt->Sync0 = Byte;
            pPkt->Index++;
        }
        break;

    // Sync byte 1
    case 1:
        // If this byte matches the second sync byte, move on. Otherwise, restart the state machine
        if (Byte == (UInt8)(Sync & 0xFF))
        {
            pPkt->Sync1 = Byte;
            pPkt->Index++;
        }
        else
            pPkt->Index = 0;
        break;

    // Axis and packet type data
    case 2:
        // Axis specifier is the MSb, packet type is in the lower 7 bits
        pPkt->ID = Byte;
        pPkt->Index++;
        break;

    // Data length specifier - does not include header or checksum
    case 3:
        // If the packet is not going to overrun our bufer
        if (Byte <= TRILLIUM_PKT_MAX_SIZE)
        {
            // Store the data length and increment the state variable
            pPkt->Length = Byte;
            pPkt->Index++;
        }
        // Otherwise, dump this packet and move on
        else
            pPkt->Index = 0;
        break;

    // All other states
    default:
        // If we have not gone beyond the end of the packet
        if (pPkt->Index < ((UInt16)pPkt->Length + 6))
        {
            // Dump this byte into the data payload storage array
            pPkt->Data[pPkt->Index++ - 4] = Byte;

            // If we're at the very last byte of the packet
            if (pPkt->Index == ((UInt16)pPkt->Length + 6))
            {
                // Reset the state tracker
                pPkt->Index = 0;

                // Return TRUE if the checksum works out, otherwise return FALSE
                return (pPkt->Data[(UInt16)pPkt->Length    ] == pPkt->Check0) &&
                       (pPkt->Data[(UInt16)pPkt->Length + 1] == pPkt->Check1);
            }
        }
        // If we end up beyond the end of the packet, restart the state machine
        else
            pPkt->Index = 0;
        break;
    };

    // If we haven't already returned, we ain't done yet!
    return FALSE;

}// LookForOrionPacketInByte

BOOL MakeTrilliumPacket(TrilliumPkt_t *pPkt, UInt16 Sync, UInt8 ID, UInt16 Length)
{
    // Get a byte pointer to the start of the packet structure
    UInt8 *pData = (UInt8 *)pPkt, i;

    // If this is an invalid data length, return FALSE immediately
    if (Length > TRILLIUM_PKT_MAX_SIZE)
        return FALSE;

    // Dump in all the rote data and zero out the checksum tracker
    pPkt->Sync0 = (UInt8)(Sync >> 8);
    pPkt->Sync1 = (UInt8)(Sync & 0xFF);
    pPkt->ID = ID;
    pPkt->Length = (UInt8)Length;
    pPkt->Check0 = 1;
    pPkt->Check1 = 0;

    // Roll each byte into the running checksum
    for (i = 0; i < Length + 4; i++)
        UpdateChecksum(pData[i], &pPkt->Check0, &pPkt->Check1);

    // Negate the checksum and paste its bytes onto the end of the data payload
    pPkt->Data[Length++] = (UInt8)(pPkt->Check0 & 0xFF);
    pPkt->Data[Length++] = (UInt8)(pPkt->Check1 & 0xFF);

    // If we made it here, this is a "good" packet
    return TRUE;

}// MakeOrionPacket

static void InitChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB)
{
    // For the first iteration, both checksum bytes should be equal
    *pA = *pB = (((UInt16)Byte & 0xFF) + 1) % 251;

}// InitChecksum

static void UpdateChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB)
{
    // Surprise! It's just Fletcher's checksum mod 251 (prime number) instead of 255
    *pA = (*pA + ((UInt16)Byte & 0xFF)) % 251;
    *pB = (*pB + *pA) % 251;

}// UpdateChecksum
