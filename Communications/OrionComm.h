#ifndef WINDOWSCOMM_H
#define WINDOWSCOMM_H

// define a struct that can hold
#include <stdint.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <windows.h>
#endif

#include "OrionPublicPacketShim.h"

#ifdef _WIN32

#define usleep(x) Sleep(x/1000)

typedef struct CommSocket_t{
    HANDLE SerialHandle;
    SOCKET TcpSocket;
    SOCKET UdpHandle;
    WSADATA WSAData;
    uint32_t BroadcastAddr;
    uint32_t Address;
    struct sockaddr_in BroadcastStruct;
    struct sockaddr_in UdpInStruct;
    struct sockaddr_in SocketStruct;
    OrionPkt_t RxPkt;
}CommSocket_t;

#else
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>

typedef struct CommSocket_t{
    int    SerialHandle;
    int    UdpHandle;
    struct sockaddr_in BroadcastRecv;
    struct sockaddr_in BroadcastAddr;
    struct sockaddr_in SocketStruct;
    struct termios Port;
    OrionPkt_t RxPkt;
}CommSocket_t;
#endif // _WIN32

static const char BROADCAST_IP[] = {"255.255.255.255\0"};

#define UDP_OUT_PORT        8745
#define UDP_IN_PORT         8746
#define TCP_PORT            8747

#ifdef __cplusplus
extern "C"
{
#endif


// Multi-connection API (explicit CommSocket_t)
BOOL OrionCommOpenEx(int *pArgc, char ***pArgv, CommSocket_t *commSocket);
BOOL OrionCommOpenSerialEx(const char *pPath, CommSocket_t* commSocket);
BOOL OrionCommOpenNetworkIpEx(const char *pAddress, CommSocket_t* commSocket);
BOOL OrionCommIpStringValid(const char *pAddress);
BOOL OrionCommSerialPathValid(const char *pPath);
void OrionCommCloseEx(CommSocket_t *commSocket);
BOOL OrionCommSendEx(const OrionPkt_t *pPkt, CommSocket_t* commSocket);
BOOL OrionCommReceiveEx(OrionPkt_t *pPkt, CommSocket_t* commSocket);
BOOL OrionCommIsOpenEx(CommSocket_t* commSocket);
struct sockaddr *GetSockAddr(uint32_t Address, unsigned short Port, struct sockaddr_in * SockAddr);

// Single-connection API (uses internal default CommSocket_t)
BOOL OrionCommOpen(int *pArgc, char ***pArgv);
BOOL OrionCommOpenSerial(const char *pPath);
BOOL OrionCommOpenNetworkIp(const char *pAddress);
void OrionCommClose(void);
BOOL OrionCommSend(const OrionPkt_t *pPkt);
BOOL OrionCommReceive(OrionPkt_t *pPkt);
BOOL OrionCommIsOpen(void);
#define OrionCommOpenNetwork() OrionCommOpenNetworkIp("255.255.255.255")
#ifdef __cplusplus
}
#endif

#endif // WINDOWSCOMM_H
