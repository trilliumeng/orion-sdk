#ifndef WINDOWSCOMM_H
#define WINDOWSCOMM_H

#ifdef _WIN32
#include <windows.h>
#endif // _WIN32

#include "OrionPublicPacketShim.h"

#define UDP_OUT_PORT        8745
#define UDP_IN_PORT         8746
#define TCP_PORT            8747

#ifdef __cplusplus
extern "C"
{
#endif

BOOL OrionCommOpenSerial(const char *pPath);
BOOL OrionCommOpenNetwork(void);
void OrionCommClose(void);
BOOL OrionCommSend(const OrionPkt_t *pPkt);
BOOL OrionCommReceive(OrionPkt_t *pPkt);

#ifdef __cplusplus
}
#endif

#endif // WINDOWSCOMM_H
