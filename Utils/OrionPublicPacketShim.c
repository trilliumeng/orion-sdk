/*
 * \file
 * This module provides the connection from the automatically generated public
 * protocol code to the Orion packet routines.
 */

#include "OrionPublicProtocol.h"
#include "OrionPublicPacketShim.h"
#include "linearalgebra.h"
#include "earthposition.h"
#include "GeolocateTelemetry.h"
#include "mathutilities.h"
#include <math.h>


//! \return the packet data pointer from the packet
uint8_t* getOrionPublicPacketData(void* pkt)
{
	return ((OrionPkt_t*)pkt)->Data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getOrionPublicPacketDataConst(const void* pkt)
{
	return ((const OrionPkt_t*)pkt)->Data;
}

//! Complete a packet after the data have been encoded
void finishOrionPublicPacket(void* pkt, int size, uint32_t packetID)
{
	MakeOrionPacket((OrionPkt_t*)pkt, (uint8_t)packetID, (uint16_t)size);
}

//! \return the size of a packet from the packet header
int getOrionPublicPacketSize(const void* pkt)
{
	return ((OrionPkt_t*)pkt)->Length;
}

//! \return the ID of a packet from the packet header
uint32_t getOrionPublicPacketID(const void* pkt)
{
	return ((OrionPkt_t*)pkt)->ID;
}

void FormOrionGpsData(OrionPkt_t *pPkt, const GpsData_t *pGps)
{
    encodeGpsDataPacketStructure(pPkt, pGps);

}// FormOrionGpsData

BOOL DecodeOrionGpsData(const OrionPkt_t *pPkt, GpsData_t *pGps)
{
    if(decodeGpsDataPacketStructure(pPkt, pGps))
    {
        // Construct the data that is not transmitted

        // Compute ground speed data from NED data.
        pGps->GroundSpeed = sqrtf(SQR(pGps->VelNED[NORTH]) + SQR(pGps->VelNED[EAST]));
        pGps->GroundHeading = atan2(pGps->VelNED[EAST], pGps->VelNED[NORTH]);

        // Use GPS time information to compute the Gregorian calendar date.
        computeDateAndTimeFromWeekAndItow(pGps->Week, pGps->ITOW, pGps->leapSeconds, &pGps->Year, &pGps->Month, &pGps->Day, &pGps->Hour, &pGps->Minute, &pGps->Second);

        if(pGps->Week != 0)
            pGps->TimeValid = 1;
        else
            pGps->TimeValid = 0;

        // Determine if this is a valid, not deadreckoned, 3D fix
        if((pGps->FixType == 3) && (pGps->FixState & 0x01) && (pGps->TrackedSats >= 4))
            pGps->valid3DFix = TRUE;
        else
            pGps->valid3DFix = FALSE;

        return TRUE;
    }
    else
        return FALSE;

}// DecodeOrionGpsData
