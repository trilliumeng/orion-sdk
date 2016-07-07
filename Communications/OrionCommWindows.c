#include "OrionComm.h"

#ifdef _WIN32

#include <winsock2.h>
#include <windows.h>
#include <stdio.h>
#include <ws2tcpip.h>

static HANDLE SerialHandle = INVALID_HANDLE_VALUE;
static SOCKET TcpSocket = INVALID_SOCKET;

static struct sockaddr *GetSockAddr(uint32_t Address, unsigned short Port);

BOOL OrionCommOpenSerial(const char *pPath)
{
	// Declare variables and structures
    SerialHandle = CreateFileA(pPath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                              OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // If the handle is valid
    if (SerialHandle != INVALID_HANDLE_VALUE)
    {
        DCB Params = { sizeof(DCB) };
        COMMTIMEOUTS Timeouts;

        // Try to get the current state
        if (GetCommState(SerialHandle, &Params) == FALSE)
        {
            // If that failed, close the handle and invalidate it
            CloseHandle(SerialHandle);
            SerialHandle = INVALID_HANDLE_VALUE;
        }

        // Otherwise, fill in the fields we care about
        Params.BaudRate = CBR_115200;
        Params.ByteSize = 8;
        Params.StopBits = ONESTOPBIT;
        Params.Parity   = NOPARITY;

        // Try changing the serial port settings
        if (SetCommState(SerialHandle, &Params) == FALSE)
        {
            // Close and invalidate the handle
            CloseHandle(SerialHandle);
            SerialHandle = INVALID_HANDLE_VALUE;
        }

        // If getting the existing timeout values fails
        if (GetCommTimeouts(SerialHandle, &Timeouts) == FALSE)
        {
            // Close and invalidate the handle
            CloseHandle(SerialHandle);
            SerialHandle = INVALID_HANDLE_VALUE;
        }

        // Set the timeouts for non-blocking mode
        Timeouts.ReadIntervalTimeout = MAXDWORD;
        Timeouts.ReadTotalTimeoutConstant = 0;
        Timeouts.ReadTotalTimeoutMultiplier = 0;

        // Try passing the new timeout struct
        if (SetCommTimeouts(SerialHandle, &Timeouts) == FALSE)
        {
            // Close and invalidate the handle on failure
            CloseHandle(SerialHandle);
            SerialHandle = INVALID_HANDLE_VALUE;
        }
    }

    // Return false
    return SerialHandle != INVALID_HANDLE_VALUE;

}// OrionCommOpenSerial

BOOL OrionCommOpenNetwork(void)
{
    WSADATA WsaData;
    WSAStartup(MAKEWORD(2, 0), &WsaData);

    // Open a new UDP socket for auto-discovery
    SOCKET UdpHandle = socket(AF_INET, SOCK_DGRAM, 0);

    // If the socket looks good
    if (UdpHandle != INVALID_SOCKET)
    {
        BOOL Broadcast = TRUE;
        int WaitCount = 0;
        char Buffer[64];
        OrionPkt_t Pkt;
        u_long Arg = 1;

        // Bind to the proper port to get responses from the gimbal
        bind(UdpHandle, GetSockAddr(INADDR_ANY, UDP_IN_PORT), sizeof(struct sockaddr_in));

        // Make this socket non blocking
        ioctlsocket(UdpHandle, FIONBIO, &Arg);

        // Allow the socket to send packets to the broadcast address
        setsockopt(UdpHandle, SOL_SOCKET, SO_BROADCAST, (char *)&Broadcast, sizeof(BOOL));

        // Build a version request packet (note that it doesn't matter what you send...)
        MakeOrionPacket(&Pkt, ORION_PKT_CROWN_VERSION, 0);

        // Wait for up to 10 iterations
        while (WaitCount++ < 10)
        {
            int Size = sizeof(struct sockaddr_in);

            // Send a version request packet
            sendto(UdpHandle, (char *)&Pkt, Pkt.Length + ORION_PKT_OVERHEAD, 0, GetSockAddr(INADDR_BROADCAST, UDP_OUT_PORT), sizeof(struct sockaddr_in));

            // If we get data back forom the gimbal
            if (recvfrom(UdpHandle, Buffer, 64, 0, GetSockAddr(INADDR_ANY, UDP_IN_PORT), &Size) > 0)
            {
                // Pull the gimbal's IP address from the datagram header
                UInt32 Address = ntohl(((struct sockaddr_in *)GetSockAddr(0, 0))->sin_addr.s_addr);

                // Open a file descriptor for the TCP comm socket
                TcpSocket = socket(AF_INET, SOCK_STREAM, 0);

                // Bind to the right incoming port
                bind(TcpSocket, GetSockAddr(INADDR_ANY, TCP_PORT), sizeof(struct sockaddr_in));

                // Connect to the gimbal's server socket (note this is a blocking call)
                connect(TcpSocket, GetSockAddr(Address, TCP_PORT), sizeof(struct sockaddr_in));

                // Now make the socket non-blocking for future reads/writes
                ioctlsocket(TcpSocket, FIONBIO, &Arg);

                // Now print out the IP address that we connected to
                printf("Connected to %s\n", inet_ntoa(((struct sockaddr_in *)GetSockAddr(0, 0))->sin_addr));

                // Close the UDP socket now that we're done with it and get out of this loop
                closesocket(UdpHandle);
                break;
            }

            // Sleep for half a second
            Sleep(500);
        }
    }

    // Return a possibly valid handle to this socket
    return TcpSocket != INVALID_SOCKET;

}// OrionCommOpenNetwork

void OrionCommClose(void)
{
    // Easy enough, just close the file descriptor
    CloseHandle(SerialHandle);
    closesocket(TcpSocket);

}// OrionCommClose

BOOL OrionCommSend(const OrionPkt_t *pPkt)
{
    // Write the packet, including header data, to the file descriptor
    if (SerialHandle != INVALID_HANDLE_VALUE)
        return WriteFile(SerialHandle, (char *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD, NULL, NULL);
    else
        return send(TcpSocket, (char *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD, 0) != SOCKET_ERROR;

}// OrionCommSend

BOOL OrionCommReceive(OrionPkt_t *pPkt)
{
    static OrionPkt_t Pkt = { 0 };
    DWORD BytesRead;
    UInt8 Byte;

    if (SerialHandle != INVALID_HANDLE_VALUE)
    {
        // As long as we keep getting bytes, keep reading them in one by one
        while (ReadFile(SerialHandle, &Byte, 1, &BytesRead, NULL))
        {
            // If this byte is the end of a valid packet
            if (LookForOrionPacketInByte(&Pkt, Byte))
            {
                // Copy the packet into the passed-in location and return a success
                *pPkt = Pkt;
                return TRUE;
            }
        }
    }
    else
    {
        // As long as we keep getting bytes, keep reading them in one by one
        while (recv(TcpSocket, (char *)&Byte, 1, 0) == 1)
        {
            // If this byte is the end of a valid packet
            if (LookForOrionPacketInByte(&Pkt, Byte))
            {
                // Copy the packet into the passed-in location and return a success
                *pPkt = Pkt;
                return TRUE;
            }
        }
    }

    // Nope, no packets yet
    return FALSE;

}// OrionCommReceive

// Quickly and easily constructs a sockaddr pointer for a bunch of different functions.
//   Call this function with Address == Port == 0 to access the pointer, or pass in
//   actual values to construct a new sockaddr.
static struct sockaddr *GetSockAddr(uint32_t Address, unsigned short Port)
{
    static struct sockaddr_in SockAddr;

    // If address and port are both zero, don't modify the structure
    if (Address || Port)
    {
        // Otherwise, populate it with the requested IP address and port
        memset(&SockAddr, 0, sizeof(SockAddr));
        SockAddr.sin_family = AF_INET;
        SockAddr.sin_addr.s_addr = htonl(Address);
        SockAddr.sin_port = htons(Port);
    }

    // Return a casted pointer to the sockaddr_in structure
    return (struct sockaddr *)&SockAddr;

}// GetSockAddr
#endif // _WIN32
