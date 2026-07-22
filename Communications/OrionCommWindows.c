#include "OrionComm.h"

#ifdef _WIN32


BOOL OrionCommOpenSerialEx(const char *pPath, CommSocket_t *socket)
{
    if(socket == NULL) {
        return FALSE;
    }
	// Declare variables and structures
    socket->SerialHandle = CreateFileA(pPath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                              OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // If the handle is valid
    if ( socket->SerialHandle != INVALID_HANDLE_VALUE)
    {
        DCB Params = { sizeof(DCB) };
        COMMTIMEOUTS Timeouts;

        // Try to get the current state
        if (GetCommState( socket->SerialHandle, &Params) == FALSE)
        {
            // If that failed, close the handle and invalidate it
            CloseHandle( socket->SerialHandle);
            socket->SerialHandle = INVALID_HANDLE_VALUE;
        }

        // Otherwise, fill in the fields we care about
        Params.BaudRate = CBR_115200;
        Params.ByteSize = 8;
        Params.StopBits = ONESTOPBIT;
        Params.Parity   = NOPARITY;

        // Try changing the serial port settings
        if (SetCommState(socket->SerialHandle, &Params) == FALSE)
        {
            // Close and invalidate the handle
            CloseHandle(socket->SerialHandle);
            socket->SerialHandle = INVALID_HANDLE_VALUE;
        }

        // If getting the existing timeout values fails
        if (GetCommTimeouts(socket->SerialHandle, &Timeouts) == FALSE)
        {
            // Close and invalidate the handle
            CloseHandle(socket->SerialHandle);
            socket->SerialHandle = INVALID_HANDLE_VALUE;
        }

        // Set the timeouts for non-blocking mode
        Timeouts.ReadIntervalTimeout = MAXDWORD;
        Timeouts.ReadTotalTimeoutConstant = 0;
        Timeouts.ReadTotalTimeoutMultiplier = 0;

        // Try passing the new timeout struct
        if (SetCommTimeouts(socket->SerialHandle, &Timeouts) == FALSE)
        {
            // Close and invalidate the handle on failure
            CloseHandle(socket->SerialHandle);
            socket->SerialHandle = INVALID_HANDLE_VALUE;
        }
    }

    // Tell the user if this failed or, if not, which COM port they're trying to use
    if (socket->SerialHandle == INVALID_HANDLE_VALUE)
        printf("Failed to open %s\n", pPath);
    else
        printf("Looking for gimbal on %s...\n", pPath);

    // Return false
    return socket->SerialHandle != INVALID_HANDLE_VALUE;

}// OrionCommOpenSerialEx

BOOL OrionCommIpStringValid(const char *pAddress)
{
    uint32_t Address = inet_addr(pAddress);

    BOOL not_any = (Address != INADDR_ANY);
	//BOOL not_none = (Address != INADDR_NONE); // Apparently on some implementations INADDR_NONE and INADDR_BROADCAST (which is valid!) are the same...

    // Return TRUE if this is a valid IP address
    return not_any;// && not_none;

}// OrionCommIpStringValid

BOOL OrionCommSerialPathValid(const char *pPath)
{
    // Return TRUE if the path starts with /dev/tty
    return strncmp(pPath, "\\\\.\\COM", 7) == 0;

}// OrionCommSerialPathValid

BOOL OrionCommOpenNetworkIpEx(const char *pAddress, CommSocket_t *commSocket)
{
    commSocket->SerialHandle = INVALID_HANDLE_VALUE;

    // Open a new UDP socket for auto-discovery
    WSAStartup(MAKEWORD(2, 0), &commSocket->WSAData);
    commSocket->UdpHandle = socket(AF_INET, SOCK_DGRAM, 0);

    // If we were passed a valid IP string
    if (OrionCommIpStringValid(pAddress) == TRUE)
    {
        // Convert the IP address string to a 32-bit IPv4 address value
        commSocket->BroadcastAddr = htonl(inet_addr(pAddress));

        // Now print out the broadcast address we're pinging
        GetSockAddr(commSocket->BroadcastAddr, UDP_OUT_PORT, &commSocket->BroadcastStruct);
        printf("Looking for gimbal on %s...\n", inet_ntoa(commSocket->BroadcastStruct.sin_addr));
        
        // Roll the bytes for our GetSockAddr function
        commSocket->BroadcastAddr = ntohl(commSocket->BroadcastAddr);
    }
    else
    {
        // Close the discovery handle and return a failure
        if (commSocket->UdpHandle != INVALID_SOCKET)
          closesocket(commSocket->UdpHandle);

        return FALSE;
    }

    // If the socket looks good
    if (commSocket->UdpHandle != INVALID_SOCKET)
    {
        BOOL Broadcast = TRUE;
        int WaitCount = 0;
        char Buffer[64];
        OrionPkt_t Pkt;
        u_long Arg = 1;

        // Bind to the proper port to get responses from the gimbal
        GetSockAddr(INADDR_ANY, UDP_IN_PORT, &commSocket->SocketStruct);
        bind(commSocket->UdpHandle, (struct sockaddr *)&commSocket->SocketStruct, sizeof(struct sockaddr_in));

        // Make this socket non blocking
        ioctlsocket(commSocket->UdpHandle, FIONBIO, &Arg);

        // Allow the socket to send packets to the broadcast address
        setsockopt(commSocket->UdpHandle, SOL_SOCKET, SO_BROADCAST, (char *)&Broadcast, sizeof(BOOL));

        // Build a version request packet (note that it doesn't matter what you send...)
        MakeOrionPacket(&Pkt, ORION_PKT_CROWN_VERSION, 0);

        // Wait for up to 20 iterations
        while (WaitCount++ < 20)
        {
            int Size = sizeof(struct sockaddr_in);

            // Send a version request packet
            sendto(commSocket->UdpHandle, (char *)&Pkt, Pkt.Length + ORION_PKT_OVERHEAD, 0, (struct sockaddr *) &commSocket->BroadcastStruct, sizeof(struct sockaddr_in));

            // If we get data back forom the gimbal
            GetSockAddr(INADDR_ANY, UDP_IN_PORT, &commSocket->UdpInStruct);
            if (recvfrom(commSocket->UdpHandle, Buffer, 64, 0, (struct sockaddr *)&commSocket->UdpInStruct , &Size) > 0)
            {
                // Pull the gimbal's IP address from the datagram header
                commSocket->Address = ntohl(commSocket->UdpInStruct.sin_addr.s_addr);

                // Open a file descriptor for the TCP comm socket
                commSocket->TcpSocket = socket(AF_INET, SOCK_STREAM, 0);

                // Bind to the right incoming port
                bind(commSocket->TcpSocket, GetSockAddr(INADDR_ANY, TCP_PORT, &commSocket->SocketStruct) , sizeof(struct sockaddr_in));

                // Connect to the gimbal's server socket (note this is a blocking call)
                connect(commSocket->TcpSocket, GetSockAddr(commSocket->Address, TCP_PORT, &commSocket->SocketStruct), sizeof(struct sockaddr_in));

                // Now make the socket non-blocking for future reads/writes
                ioctlsocket(commSocket->TcpSocket, FIONBIO, &Arg);

                // Now print out the IP address that we connected to and break out of the loop
                printf("Connected to %s\n", inet_ntoa(commSocket->SocketStruct.sin_addr));
                break;
            }

            // Sleep for 1/10th of a second
            Sleep(100);
        }

        // If we timed out waiting for a response, let the user know
        if (WaitCount >= 20)
        {
            // Broadcast address byte roll, part one million
            commSocket->BroadcastAddr = htonl(commSocket->BroadcastAddr);

            // Let the user know we failed to connect

            printf("Failed to connect to %s\n", inet_ntoa(commSocket->BroadcastStruct.sin_addr));
            commSocket->TcpSocket = INVALID_SOCKET;
        }

        // Close the UDP socket now that we're done with it
        closesocket(commSocket->UdpHandle);
    }

    // Return a possibly valid handle to this socket
    return commSocket->TcpSocket != INVALID_SOCKET;

}// OrionCommOpenNetworkIpEx

void OrionCommCloseEx(CommSocket_t *commSocket)
{
    // Easy enough, just close the file descriptor
    CloseHandle(commSocket->SerialHandle);
    closesocket(commSocket->TcpSocket);
    commSocket->SerialHandle = INVALID_HANDLE_VALUE;
    commSocket->TcpSocket = INVALID_SOCKET;

    // Clear parse state for clean reconnect
    memset(&commSocket->RxPkt, 0, sizeof(commSocket->RxPkt));

}// OrionCommCloseEx

BOOL OrionCommSendEx(const OrionPkt_t *pPkt, CommSocket_t * commSocket)
{
    DWORD Bytes;

    // Write the packet, including header data, to the file descriptor
    if (commSocket->SerialHandle != INVALID_HANDLE_VALUE)
        return WriteFile(commSocket->SerialHandle, (char *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD, &Bytes, NULL);
    else
        return send(commSocket->TcpSocket, (char *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD, 0) != SOCKET_ERROR;

}// OrionCommSendEx

BOOL OrionCommReceiveEx(OrionPkt_t *pPkt, CommSocket_t * commSocket)
{
    UInt8 Byte;

    if (commSocket->SerialHandle != INVALID_HANDLE_VALUE)
    {
        COMSTAT Status;

        // As long as there are bytes to be read out of the receive queue
        while (ClearCommError(commSocket->SerialHandle, NULL, &Status) && (Status.cbInQue > 0))
        {
            DWORD BytesRead = 0;

            // Read a byte
            ReadFile(commSocket->SerialHandle, &Byte, 1, &BytesRead, NULL);

            // If this byte is the end of a valid packet
            if ((BytesRead == 1) && (LookForOrionPacketInByte(&commSocket->RxPkt, Byte) == TRUE))
            {
                // Copy the packet into the passed-in location and return a success
                *pPkt = commSocket->RxPkt;
                return TRUE;
            }
            // Otherwise, if some sort of error occurred
            else if (BytesRead != 1)
            {
                // Close and invalidate the serial port
                CloseHandle(commSocket->SerialHandle);
                commSocket->SerialHandle = INVALID_HANDLE_VALUE;
            }
        }
    }
    else
    {
        // As long as we keep getting bytes, keep reading them in one by one
        while (recv(commSocket->TcpSocket, (char *)&Byte, 1, 0) == 1)
        {
            // If this byte is the end of a valid packet
            if (LookForOrionPacketInByte(&commSocket->RxPkt, Byte))
            {
                // Copy the packet into the passed-in location and return a success
                *pPkt = commSocket->RxPkt;
                return TRUE;
            }
        }
    }

    // Nope, no packets yet
    return FALSE;

}// OrionCommReceiveEx

BOOL OrionCommIsOpenEx(CommSocket_t *commSocket)
{
    // Return TRUE if one of the handles is valid
    return (commSocket->SerialHandle != INVALID_HANDLE_VALUE) || (commSocket->TcpSocket != INVALID_SOCKET);

}// OrionCommIsOpenEx

// Quickly and easily constructs a sockaddr pointer for a bunch of different functions.
//   Call this function with Address == Port == 0 to access the pointer, or pass in
//   actual values to construct a new sockaddr.
struct sockaddr *GetSockAddr(uint32_t Address, unsigned short Port, struct sockaddr_in * SockAddr)
{
           // If address and port are both zero, don't modify the structure
    if (Address || Port)
    {
        // Otherwise, populate it with the requested IP address and port
        memset(SockAddr, 0, sizeof(*SockAddr));
        SockAddr->sin_family = AF_INET;
        SockAddr->sin_addr.s_addr = htonl(Address);
        SockAddr->sin_port = htons(Port);
    }

           // Return a casted pointer to the sockaddr_in structure
    return (struct sockaddr *) SockAddr;

}// GetSockAddr
#endif // _WIN32
