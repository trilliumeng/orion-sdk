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



void FormOrionCommand(OrionPkt_t *pPkt, const OrionCmd_t *pCmd)
{
    encodeOrionCmdPacketStructure(pPkt, pCmd);

}// FormOrionCommand

BOOL DecodeOrionCommand(const OrionPkt_t *pPkt, OrionCmd_t *pCmd)
{
    return decodeOrionCmdPacketStructure(pPkt, pCmd);

}// DecodeOrionCommand

void FormOrionLaserCommand(OrionPkt_t *pPkt, float DutyCycle)
{
    // Limit the duty cycle to [0, 100]%
    if (DutyCycle > 1.0f) DutyCycle = 1.0f;
    else if (DutyCycle < 0.0f) DutyCycle = 0.0f;

    encodeOrionLaserCommandPacket(pPkt, DutyCycle);

}// FormOrionLaserCommand

BOOL DecodeOrionLaserCommand(const OrionPkt_t *pPkt, float *pDutyCycle)
{
    return decodeOrionLaserCommandPacket(pPkt, pDutyCycle);

}// DecodeOrionLaserCommand

void FormOrionClevisVersion(OrionPkt_t *pPkt, const char Version[16], const char PartNumber[16])
{
    encodeOrionClevisVersionPacket(pPkt, Version, PartNumber);

}// FormOrionVersion

BOOL DecodeOrionClevisVersion(const OrionPkt_t *pPkt, char Version[16], char PartNumber[16])
{
    return decodeOrionClevisVersionPacket(pPkt, Version, PartNumber);

}// DecodeOrionVersion

void FormOrionCrownVersion(OrionPkt_t *pPkt, const char Version[16], const char PartNumber[16])
{
    encodeOrionCrownVersionPacket(pPkt, Version, PartNumber);

}// FormOrionCrownVersion

BOOL DecodeOrionCrownVersion(const OrionPkt_t *pPkt, char Version[16], char PartNumber[16])
{
    return decodeOrionCrownVersionPacket(pPkt, Version, PartNumber);

}// DecodeOrionCrownVersion

void FormOrionPayloadVersion(OrionPkt_t *pPkt, const char Version[24], const char PartNumber[24])
{
    encodeOrionCrownVersionPacket(pPkt, Version, PartNumber);

}// FormOrionPayloadVersion

BOOL DecodeOrionPayloadVersion(const OrionPkt_t *pPkt, char Version[24], char PartNumber[24])
{
    return decodeOrionCrownVersionPacket(pPkt, Version, PartNumber);

}// DecodeOrionPayloadVersion

void FormOrionResetSource(OrionPkt_t *pPkt, UInt32 Vector, UInt32 Address, OrionBoardEnumeration_t sourceBoard)
{
    encodeOrionResetSourcePacket(pPkt, Vector, Address, sourceBoard);

}// FormOrionResetSource

BOOL DecodeOrionResetSource(const OrionPkt_t *pPkt, UInt32 *pVector, UInt32 *pAddress, OrionBoardEnumeration_t* pSourceBoard)
{
    return decodeOrionResetSourcePacket(pPkt, pVector, pAddress, pSourceBoard);

}// DecodeOrionResetSource

void FormOrionDiagnostics(OrionPkt_t *pPkt, const OrionDiagnostics_t *pData)
{
    encodeOrionDiagnosticsPacketStructure(pPkt, pData);

}//  FormOrionPowerData

BOOL DecodeOrionDiagnostics(const OrionPkt_t *pPkt, OrionDiagnostics_t *pData)
{
    return decodeOrionDiagnosticsPacketStructure(pPkt, pData);

}// DecodeOrionPowerData

void FormOrionPerformance(OrionPkt_t *pPkt, const OrionPerformance_t *pPerf)
{
    encodeOrionPerformancePacket(pPkt, pPerf->RmsQuad, pPerf->RmsDir, pPerf->RmsVel, pPerf->RmsPos, pPerf->Iout);
}

BOOL DecodeOrionPerformance(const OrionPkt_t *pPkt, OrionPerformance_t *pPerf)
{
    return decodeOrionPerformancePacket(pPkt, pPerf->RmsQuad, pPerf->RmsDir, pPerf->RmsVel, pPerf->RmsPos, pPerf->Iout);
}

void FormOrionCameraSwitch(OrionPkt_t *pPkt, UInt8 Index)
{
    // TO-DO: MAKE THE MAXIMUM CAMERA INDEX A #DEFINE
    encodeOrionCameraSwitchPacket(pPkt, MIN(Index, 1));

}// FormOrionCameraSwitch

BOOL DecodeOrionCameraSwitch(const OrionPkt_t *pPkt, UInt8 *pIndex)
{
    if(decodeOrionCameraSwitchPacket(pPkt, pIndex))
    {
        // TO-DO: MAKE THE MAXIMUM CAMERA INDEX A #DEFINE
        *pIndex = MIN(*pIndex, 1);

        // Great success!
        return TRUE;
    }

    // If the packet is too short or doesn't match this ID, let the caller know it failed
    return FALSE;

}// DecodeOrionCameraSwitch

// NOTE: This packet will eventually be deprecated
void FormOrionCameraState(OrionPkt_t *pPkt, UInt8 Index, float Zoom, float Focus)
{
    encodeOrionCameraStatePacket(pPkt, Zoom, Focus, Index);

}// FormOrionDigitalZoom

// NOTE: This packet will eventually be deprecated
BOOL DecodeOrionCameraState(const OrionPkt_t *pPkt, UInt8 *pIndex, float *pZoom, float *pFocus)
{
    return decodeOrionCameraStatePacket(pPkt, pZoom, pFocus, pIndex);

}// DecodeOrionDigitalZoom

void FormOrionNetworkVideo(OrionPkt_t *pPkt, UInt32 DestIp, UInt16 Port, UInt32 Bitrate)
{
    encodeOrionNetworkVideoPacket(pPkt, DestIp, Port, Bitrate);

}// FormOrionNetworkVideo

BOOL DecodeOrionNetworkVideo(const OrionPkt_t *pPkt, UInt32 *pDestIp, UInt16 *pPort, UInt32 *pBitrate)
{
    return decodeOrionNetworkVideoPacket(pPkt, pDestIp, pPort, pBitrate);

}// DecodeOrionNetworkVideo

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
        pGps->GroundSpeed = vector3Lengthf(pGps->VelNED);
        pGps->GroundHeading = atan2(pGps->VelNED[EAST], pGps->VelNED[NORTH]);

        // Use GPS time information to compute the Gregorian calendar date.
        computeDateFromWeekAndItow(pGps->Week, pGps->ITOW, &pGps->Year, &pGps->Month, &pGps->Day);

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

void FormOrionNetworkSettings(OrionPkt_t *pPkt, UInt32 Ip, UInt32 Mask, UInt32 Gateway)
{
    encodeOrionNetworkSettingsPacket(pPkt, Ip, Mask, Gateway);

}// FormOrionNetworkSettings

BOOL DecodeOrionNetworkSettings(const OrionPkt_t *pPkt, UInt32 *pIp, UInt32 *pMask, UInt32 *pGateway)
{
    return decodeOrionNetworkSettingsPacket(pPkt, pIp, pMask, pGateway);

}// DecodeOrionNetworkSettings

void FormOrionRetractCmd(OrionPkt_t *pPkt, OrionRetractCmd_t Cmd)
{
    encodeOrionRetractCmdPacket(pPkt, Cmd);
}

BOOL DecodeOrionRetractCmd(const OrionPkt_t *pPkt, OrionRetractCmd_t *pCmd)
{
    return decodeOrionRetractCmdPacket(pPkt, pCmd);
}

void FormOrionRetractStatus(OrionPkt_t *pPkt, OrionRetractCmd_t Cmd, OrionRetractState_t State, float Pos, UInt16 Flags)
{
    encodeOrionRetractStatusPacket(pPkt, Cmd, State, Pos, Flags);
}

BOOL DecodeOrionRetractStatus(const OrionPkt_t *pPkt, OrionRetractCmd_t *pCmd, OrionRetractState_t *pState, float *pPos, UInt16 *pFlags)
{
    return decodeOrionRetractStatusPacket(pPkt, pCmd, pState, pPos, pFlags);
}


/*!
 * Create a Geopoint command packet
 * \param pPkt receives the formatted packet
 * \param posLLA is the target position in latitude (radians), longitude (radians), altitude (meters WGS84)
 * \param velNED is the target velocity in North, East, Down meters per second
 */
void FormGeopointCommand(OrionPkt_t *pPkt, const double posLLA[3], const float velNED[3])
{
    // Marshal data to satisfy old API
    encodeGeopointCmdPacket(pPkt, posLLA[0], posLLA[1], posLLA[2], velNED);

}// FormGeopointCommand


/*!
 * Decode a Geopoint command packet
 * \param pPkt is the packet to decode
 * \param posLLA receives the target position in latitude (radians), longitude (radians), altitude (meters WGS84)
 * \param velNED receives the target velocity in North, East, Down meters per second
 * \return TRUE if the packet decodes successfully
 */
BOOL DecodeGeopointCommand(const OrionPkt_t *pPkt, double posLLA[3], float velNED[3])
{
    // Marshal data to satisfy old API
    return decodeGeopointCmdPacket(pPkt, &posLLA[0], &posLLA[1], &posLLA[2], velNED);

}// DecodeGeopointCommand


/*!
 * Decode a board data packet
 * \param pPkt is the packet to decode
 * \param board receives the board identifying information.
 * \return TRUE if this is a correctly formatted sensor matrix packet.
 */
BOOL DecodeBoardData(const OrionPkt_t *pPkt, OrionBoard_t* board)
{
    return decodeOrionBoardPacketStructure(pPkt, board);

}// DecodeBoardData


/*!
 * Encode a board data packet
 * \param pPkt receives the formatted packet
 * \param board is the board identifying information.
 */
void EncodeBoardData(OrionPkt_t *pPkt, const OrionBoard_t* board)
{
    encodeOrionBoardPacketStructure(pPkt, board);

}// EncodeBoardData

