#ifndef ORIONPACKETSHIM_H
#define ORIONPACKETSHIM_H

#include "TrilliumPacket.h"
#include "Constants.h"
#include "OrionPublicProtocol.h"
#include "OrionPublicPacket.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Orion-specific constants
#define ORION_SYNC         0xD00D
#define ORION_PKT_MAX_SIZE TRILLIUM_PKT_MAX_SIZE
#define ORION_PKT_OVERHEAD TRILLIUM_PKT_OVERHEAD

// Orion packets share a structure with a generic Trillium packet
typedef TrilliumPkt_t OrionPkt_t;

// And they share the same basic parsing functions
#define LookForOrionPacketInByte(a, b)  LookForTrilliumPacketInByte((TrilliumPkt_t *)a, ORION_SYNC, b)
#define MakeOrionPacket(a, b, c)        MakeTrilliumPacket(a, ORION_SYNC, b, c)

// These routines provide legacy API support, for transitioning between the
// older hand-written communications code and the newer auto-generated code

void FormOrionCommand(OrionPkt_t *pPkt, const OrionCmd_t *pCmd);
BOOL DecodeOrionCommand(const OrionPkt_t *pPkt, OrionCmd_t *pCmd);

void FormOrionClevisVersion(OrionPkt_t *pPkt, const char Version[16], const char PartNumber[16]);
BOOL DecodeOrionClevisVersion(const OrionPkt_t *pPkt, char Version[16], char PartNumber[16]);

void FormOrionCrownVersion(OrionPkt_t *pPkt, const char Version[16], const char PartNumber[16]);
BOOL DecodeOrionCrownVersion(const OrionPkt_t *pPkt, char Version[16], char PartNumber[16]);

void FormOrionPayloadVersion(OrionPkt_t *pPkt, const char Version[24], const char PartNumber[24]);
BOOL DecodeOrionPayloadVersion(const OrionPkt_t *pPkt, char Version[24], char PartNumber[24]);

void FormOrionResetSource(OrionPkt_t *pPkt, UInt32 Vector, UInt32 Address, OrionBoardEnumeration_t sourceBoard);
BOOL DecodeOrionResetSource(const OrionPkt_t *pPkt, UInt32 *pVector, UInt32 *pAddress, OrionBoardEnumeration_t* pSourceBoard);

void FormOrionDiagnostics(OrionPkt_t *pPkt, const OrionDiagnostics_t *pData);
BOOL DecodeOrionDiagnostics(const OrionPkt_t *pPkt, OrionDiagnostics_t *pData);

void FormOrionPerformance(OrionPkt_t *pPkt, const OrionPerformance_t *pPerf);
BOOL DecodeOrionPerformance(const OrionPkt_t *pPkt, OrionPerformance_t *pPerf);

void FormOrionCameraSwitch(OrionPkt_t *pPkt, UInt8 Index);
BOOL DecodeOrionCameraSwitch(const OrionPkt_t *pPkt, UInt8 *pIndex);

void FormOrionCameraState(OrionPkt_t *pPkt, UInt8 Index, float Zoom, float Focus);
BOOL DecodeOrionCameraState(const OrionPkt_t *pPkt, UInt8 *pIndex, float *pZoom, float *pFocus);

void FormOrionGpsData(OrionPkt_t *pPkt, const GpsData_t *pGps);
BOOL DecodeOrionGpsData(const OrionPkt_t *pPkt, GpsData_t *pGps);

void FormOrionNetworkSettings(OrionPkt_t *pPkt, UInt32 Ip, UInt32 Mask, UInt32 Gateway);
BOOL DecodeOrionNetworkSettings(const OrionPkt_t *pPkt, UInt32 *pIp, UInt32 *pMask, UInt32 *pGateway);

void FormOrionRetractCmd(OrionPkt_t *pPkt, OrionRetractCmd_t Cmd);
BOOL DecodeOrionRetractCmd(const OrionPkt_t *pPkt, OrionRetractCmd_t *pCmd);

void FormOrionRetractStatus(OrionPkt_t *pPkt, OrionRetractCmd_t Cmd, OrionRetractState_t State, float Pos, UInt16 Flags);
BOOL DecodeOrionRetractStatus(const OrionPkt_t *pPkt, OrionRetractCmd_t *pCmd, OrionRetractState_t *pState, float *pPos, UInt16 *pFlags);

void FormGeopointCommand(OrionPkt_t *pPkt, const double posLLA[3], const float velNED[3]);
BOOL DecodeGeopointCommand(const OrionPkt_t *pPkt, double posLLA[3], float velNED[3]);

BOOL DecodeBoardData(const OrionPkt_t *pPkt, OrionBoard_t* board);
void EncodeBoardData(OrionPkt_t *pPkt, const OrionBoard_t* board);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ORIONPACKETSHIM_H
