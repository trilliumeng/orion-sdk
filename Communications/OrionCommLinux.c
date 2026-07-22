#include "OrionComm.h"

#if defined(__linux__) || defined(__APPLE__)

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

BOOL OrionCommOpenSerialEx(const char *pPath, CommSocket_t *commSocket)
{
    // Open a file descriptor for the serial port
    commSocket->SerialHandle = open(pPath, O_RDWR | O_NOCTTY | O_NDELAY);

    // If we actually managed to open something
    if (commSocket->SerialHandle >= 0)
    {
        struct termios Port;

        // Make sure this is a serial port and that we can get its attributes
        if (!isatty(commSocket->SerialHandle) || tcgetattr(commSocket->SerialHandle, &Port))
        {
            // If we can't, close and invalidate the file descriptor
            close(commSocket->SerialHandle);
            commSocket->SerialHandle = -1;
        } 
        else
        {
            // Otherwise, clear out the port attributes structure
            memset(&Port, 0, sizeof(Port));

            // Now set up all the other miscellaneous flags appropriately
            Port.c_cflag = B115200 | CS8 | CLOCAL | CREAD;

            // Try passing the new attributes to the port
            if (tcsetattr(commSocket->SerialHandle, TCSANOW, &Port) != 0)
            {
                // If it didn't work, close and invalidate the port
                close(commSocket->SerialHandle);
                commSocket->SerialHandle = -1;
            } else { // succesfully configured, update the store port information
                commSocket->Port = Port;
            }
        }
    }

    // Tell the user if this failed or, if not, which COM port they're trying to use
    if (commSocket->SerialHandle == -1)
        printf("Failed to open %s\n", pPath);
    else
        printf("Looking for gimbal on %s...\n", pPath);

    // Return the file descriptor
    return commSocket->SerialHandle != -1;

}// OrionCommOpenSerialEx

BOOL OrionCommIpStringValid(const char *pAddress)
{
    uint32_t Address; 

    // Return TRUE if this is a valid IP address
    return inet_pton(AF_INET, pAddress, &Address) == 1;

}// OrionCommIpStringValid

BOOL OrionCommSerialPathValid(const char *pPath)
{
    // Return TRUE if the path starts with /dev/tty
    return strncmp(pPath, "/dev/tty", 8) == 0;

}//  OrionCommSerialPathValid

BOOL OrionCommOpenNetworkIpEx(const char *pAddress, CommSocket_t *commSocket)
{
    // Open a new UDP socket for auto-discovery
    commSocket->UdpHandle = socket(AF_INET, SOCK_DGRAM, 0);
    uint32_t BroadcastAddr = INADDR_BROADCAST;
    char IpString[INET_ADDRSTRLEN];

    // Try converting the address to a uint32_t
    if (OrionCommIpStringValid(pAddress) == TRUE)
    {
        // Convert the IPaddress string to a 32-bit IPv4 address value
        inet_pton(AF_INET, pAddress, &BroadcastAddr);

        // Now print out the broadcast address we're pinging
        printf("Looking for gimbal on %s...\n", inet_ntop(AF_INET, &BroadcastAddr, IpString, INET_ADDRSTRLEN));

        // Roll the bytes for our GetSockAddr function
        BroadcastAddr = ntohl(BroadcastAddr);
    }
    else
    {
        // Close the discovery handle and return a failure
        close(commSocket->UdpHandle);
        return FALSE;
    }

    // If the socket looks good
    if (commSocket->UdpHandle >= 0)
    {
        BOOL Broadcast = TRUE;
        int WaitCount = 0;
        char Buffer[64];
        OrionPkt_t Pkt;

        // Bind to the proper port to get responses from the gimbal
        if(bind(commSocket->UdpHandle, GetSockAddr(INADDR_ANY, UDP_IN_PORT, &commSocket->BroadcastAddr) , sizeof(struct sockaddr_in)) < 0 )
        {
            printf("could not bind the udp broadcast socket");
            close(commSocket->UdpHandle);
            return FALSE;
        }

        // Make this socket non blocking
        fcntl(commSocket->UdpHandle, F_SETFL, O_NONBLOCK);

        // Allow the udp handle to reuse the address
        setsockopt(commSocket->UdpHandle, SOL_SOCKET, SO_REUSEADDR, (char *)&Broadcast, sizeof(BOOL));

        // Allow the socket to send packets to the broadcast address
        setsockopt(commSocket->UdpHandle, SOL_SOCKET, SO_BROADCAST, (char *)&Broadcast, sizeof(BOOL));

        // Build a version request packet (note that it doesn't matter what you send...)
        MakeOrionPacket(&Pkt, ORION_PKT_CROWN_VERSION, 0);

        // Wait for up to 20 iterations
        while (WaitCount++ < 20)
        {
            socklen_t Size = sizeof(struct sockaddr_in);

            // Send a version request packet
            sendto(commSocket->UdpHandle, (char *)&Pkt, Pkt.Length + ORION_PKT_OVERHEAD, 0, GetSockAddr(BroadcastAddr, UDP_OUT_PORT, &commSocket->BroadcastAddr), sizeof(struct sockaddr_in));

            // If we get data back forom the gimbal
            if (recvfrom(commSocket->UdpHandle, Buffer, 64, 0, GetSockAddr(INADDR_ANY, UDP_IN_PORT, &commSocket->BroadcastRecv), &Size) > 0)
            {
                // Pull the gimbal's IP address from the datagram header
                UInt32 Address = ntohl(commSocket->BroadcastRecv.sin_addr.s_addr);

                // Open a file descriptor for the TCP comm socket
                commSocket->SerialHandle = socket(AF_INET, SOCK_STREAM, 0);

                // Connect to the gimbal's server socket (note this is a blocking call)                
                if(connect(commSocket->SerialHandle, GetSockAddr(Address, TCP_PORT, &commSocket->SocketStruct), sizeof commSocket->SocketStruct) != 0)
                {
                    printf("Failed to connect to %s", inet_ntoa(commSocket->SocketStruct.sin_addr));
                    continue;
                }

                // Now make the socket non-blocking for future reads/writes
                if(fcntl(commSocket->SerialHandle, F_SETFL, O_NONBLOCK) != 0)
                {
                    printf("Failed to set the TCP Socket to non-blocking");
                }

                // Convert the IP address to network byte order
                Address = htonl(Address);

                // Now print out the IP address that we connected to and break out of the loop
                printf("Connected to %s\n", inet_ntop(AF_INET, &Address, IpString, INET_ADDRSTRLEN));
                break;
            }

            // Sleep for 1/10th of a second
            usleep(100000);
        }

        // If we timed out waiting for a response, let the user know
        if (WaitCount >= 20)
        {
            // Broadcast address byte roll, part one million
            BroadcastAddr = htonl(BroadcastAddr);

            // Let the user know we failed to connect
            printf("Failed to connect to %s\n", inet_ntop(AF_INET, &BroadcastAddr, IpString, INET_ADDRSTRLEN));
        }

        // Close the UDP handle down now that we're done with it
        close(commSocket->UdpHandle);
    }

    // Return a possibly valid handle to this socket
    return commSocket->SerialHandle != -1;

}// OrionCommOpenNetworkIpEx

void OrionCommCloseEx(CommSocket_t * commSocket)
{
    // Close and invalidate handles
    close(commSocket->SerialHandle);
    commSocket->SerialHandle = -1;
    close(commSocket->UdpHandle);
    commSocket->UdpHandle = -1;

    // Clear parse state for clean reconnect
    memset(&commSocket->RxPkt, 0, sizeof(commSocket->RxPkt));

}// OrionCommCloseEx

BOOL OrionCommSendEx(const OrionPkt_t *pPkt, CommSocket_t *commSocket)
{
    // Write the packet, including header data, to the file descriptor
    return (write(commSocket->SerialHandle, (char *)pPkt, pPkt->Length  + ORION_PKT_OVERHEAD) > 0);

}// OrionCommSendEx

BOOL OrionCommReceiveEx(OrionPkt_t *pPkt, CommSocket_t *commSocket)
{

    if(commSocket->SerialHandle == 0) {
        return FALSE;
    }
    UInt8 Buffer;
    // As long as we keep getting bytes, keep reading them in one by one
    while (read(commSocket->SerialHandle, (char *)&Buffer, 1) == 1)
    {
        // If this byte is the end of a valid packet
        if (LookForOrionPacketInByte(&commSocket->RxPkt, Buffer))
        {
            // Copy the packet into the passed-in location and return a success
            *pPkt = commSocket->RxPkt;
            return TRUE;
        }
    }

    // Nope, no packets yet
    return FALSE;

}// OrionCommReceiveEx

BOOL OrionCommIsOpenEx(CommSocket_t *commSocket)
{
    // Return TRUE if the file descriptor is valid
    return (commSocket->SerialHandle >= 0);

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
#endif // __linux__
