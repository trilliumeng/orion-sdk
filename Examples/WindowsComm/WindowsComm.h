#ifndef WINDOWSCOMM_H
#define WINDOWSCOMM_H

#include <Windows.h>

#include "OrionPublicPacketShim.h"

#define UDP_OUT_PORT        8745
#define UDP_IN_PORT         8746
#define TCP_PORT            8747

#ifdef __cplusplus
extern "C"
{
#endif

BOOL WindowsCommOpenSerial(const char *pPath);
BOOL WindowsCommOpenNetwork(void);
void WindowsCommClose(void);
BOOL WindowsCommSend(const OrionPkt_t *pPkt);
BOOL WindowsCommReceive(OrionPkt_t *pPkt);

#ifdef __cplusplus
}
#endif

#endif // WINDOWSCOMM_H
