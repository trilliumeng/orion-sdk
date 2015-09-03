#ifndef PACKETS_H
#define PACKETS_H

#include "Types.h"

#define TRILLIUM_PKT_MAX_SIZE 140
#define TRILLIUM_PKT_OVERHEAD 6

typedef struct
{
    UInt8 Sync0;
    UInt8 Sync1;
    UInt8 ID;
    UInt8 Length;
    UInt8 Data[TRILLIUM_PKT_MAX_SIZE + 2];

    // For packet parsing use only
    UInt16 Index;
    UInt16 Check0;
    UInt16 Check1;
} TrilliumPkt_t;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

BOOL LookForTrilliumPacketInByte(TrilliumPkt_t *pPkt, UInt16 Sync, UInt8 Byte);
BOOL MakeTrilliumPacket(TrilliumPkt_t *pPkt, UInt16 Sync, UInt8 Type, UInt16 Length);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // PACKETS_H
