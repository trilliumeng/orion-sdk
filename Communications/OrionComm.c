#include "OrionComm.h"

// Default socket for single-connection API
static CommSocket_t g_defaultSocket = {0};

// Multi-connection API
BOOL OrionCommOpenEx(int *pArgc, char ***pArgv, CommSocket_t* socket)
{
    // If there are at least two arguments, and the first looks like a serial port or IP
    if (*pArgc >= 2)
    {
        // Serial port...?
        if (OrionCommSerialPathValid((*pArgv)[1]))
        {
            // Decrement the number of arguments and push the pointer up one arg
            (*pArgc)--;
            (*pArgv) = &(*pArgv)[1];

            // Try opening the specified serial port
            return OrionCommOpenSerialEx((*pArgv)[0], socket);
        }
        // IP address...?
        else if (OrionCommIpStringValid((*pArgv)[1]))
        {
            // Decrement the number of arguments and push the pointer up one arg
            (*pArgc)--;
            (*pArgv) = &(*pArgv)[1];

            // Try connecting to a gimbal at this IP
            return OrionCommOpenNetworkIpEx((*pArgv)[0], socket);
        }
    }

    // If we haven't connected any other way, try using network broadcast
    return OrionCommOpenNetworkIpEx(BROADCAST_IP, socket);

}// OrionCommOpenEx

// Single-connection API wrappers
BOOL OrionCommOpen(int *pArgc, char ***pArgv)          { if (OrionCommIsOpen()) OrionCommClose(); return OrionCommOpenEx(pArgc, pArgv, &g_defaultSocket); }
BOOL OrionCommOpenSerial(const char *pPath)            { if (OrionCommIsOpen()) OrionCommClose(); return OrionCommOpenSerialEx(pPath, &g_defaultSocket); }
BOOL OrionCommOpenNetworkIp(const char *pAddress)      { if (OrionCommIsOpen()) OrionCommClose(); return OrionCommOpenNetworkIpEx(pAddress, &g_defaultSocket); }
void OrionCommClose(void)                              { OrionCommCloseEx(&g_defaultSocket); }
BOOL OrionCommSend(const OrionPkt_t *pPkt)             { return OrionCommSendEx(pPkt, &g_defaultSocket); }
BOOL OrionCommReceive(OrionPkt_t *pPkt)                { return OrionCommReceiveEx(pPkt, &g_defaultSocket); }
BOOL OrionCommIsOpen(void)                             { return OrionCommIsOpenEx(&g_defaultSocket); }
