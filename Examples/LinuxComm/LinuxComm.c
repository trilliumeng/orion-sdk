#include "LinuxComm.h"

#include <arpa/inet.h>

static struct sockaddr *GetSockAddr(uint32_t Address, unsigned short port);

int LinuxCommOpenSerial(const char *pPath)
{
    // Open a file descriptor for the serial port
    int Handle = open(pPath, O_RDWR | O_NOCTTY | O_NDELAY);

    // If we actually managed to open something
    if (Handle >= 0)
    {
        struct termios Port;

        // Make sure this is a serial port and that we can get its attributes
        if (!isatty(Handle) || tcgetattr(Handle, &Port))
        {
            // If we can't, close and invalidate the file descriptor
            close(Handle);
            Handle = -1;
        } 
        else
        {
            // Otherwise, clear out the port attributes structure
            memset(&Port, 0, sizeof(Port));

            // Now set up all the other miscellaneous flags appropriately
            Port.c_cflag = B115200 | CS8 | CLOCAL | CREAD;

            // Try passing the new attributes to the port
            if (tcsetattr(Handle, TCSANOW, &Port) != 0)
            {
                // If it didn't work, close and invalidate the port
                close(Handle);
                Handle = -1;
            }
        }
    }

    // Return the file descriptor
    return Handle;

}// LinuxCommOpenSerial

int LinuxCommOpenNetwork(void)
{
    // Open a new UDP socket for auto-discovery
    int TcpHandle = -1, UdpHandle = socket(AF_INET, SOCK_DGRAM, 0);

    // If the socket looks good
    if (UdpHandle >= 0)
    {
        BOOL Broadcast = TRUE;
        int WaitCount = 0;
        char Buffer[64];
        OrionPkt_t Pkt;

        // Bind to the proper port to get responses from the gimbal
        bind(UdpHandle, GetSockAddr(INADDR_ANY, UDP_IN_PORT), sizeof(struct sockaddr_in));

        // Make this socket non blocking
        fcntl(UdpHandle, F_SETFL, O_NONBLOCK);

        // Allow the socket to send packets to the broadcast address
        setsockopt(UdpHandle, SOL_SOCKET, SO_BROADCAST, (char *)&Broadcast, sizeof(BOOL));

        // Build a version request packet (note that it doesn't matter what you send...)
        MakeOrionPacket(&Pkt, ORION_PKT_CROWN_VERSION, 0);

        // Wait for up to 10 iterations
        while (WaitCount++ < 10)
        {
            socklen_t Size = sizeof(struct sockaddr_in);

            // Send a version request packet
            sendto(UdpHandle, (char *)&Pkt, Pkt.Length + ORION_PKT_OVERHEAD, 0, GetSockAddr(INADDR_BROADCAST, UDP_OUT_PORT), sizeof(struct sockaddr_in));

            // If we get data back forom the gimbal
            if (recvfrom(UdpHandle, Buffer, 64, 0, GetSockAddr(INADDR_ANY, UDP_IN_PORT), &Size) > 0)
            {
                // Pull the gimbal's IP address from the datagram header
                UInt32 Address = ntohl(((struct sockaddr_in *)GetSockAddr(0, 0))->sin_addr.s_addr);
                char IpString[INET_ADDRSTRLEN];

                // Open a file descriptor for the TCP comm socket
                TcpHandle = socket(AF_INET, SOCK_STREAM, 0);

                // Bind to the right incoming port
                bind(TcpHandle, GetSockAddr(INADDR_ANY, TCP_PORT), sizeof(struct sockaddr_in));

                // Connect to the gimbal's server socket (note this is a blocking call)
                connect(TcpHandle, GetSockAddr(Address, TCP_PORT), sizeof(struct sockaddr_in));

                // Now make the socket non-blocking for future reads/writes
                fcntl(TcpHandle, F_SETFL, O_NONBLOCK);

                // Convert the IP address to network byte order
                Address = htonl(Address);

                // Now print out the IP address that we connected to
                printf("Connected to %s\n", inet_ntop(AF_INET, &Address, IpString, INET_ADDRSTRLEN));

                // Close the UDP socket now that we're done with it and get out of this loop
                close(UdpHandle);
                break;
            }

            // Sleep for half a second
            usleep(500000);
        }
    }

    // Return a possibly valid handle to this socket
    return TcpHandle;

}// LinuxCommOpenNetwork

void LinuxCommClose(int Handle)
{
    // Easy enough, just close the file descriptor
    close(Handle);

}// LinuxCommClose

void LinuxCommSend(int Handle, const OrionPkt_t *pPkt)
{
    // Write the packet, including header data, to the file descriptor
    write(Handle, (char *)pPkt, pPkt->Length + ORION_PKT_OVERHEAD);

}// LinuxCommSend

BOOL LinuxCommReceive(int Handle, OrionPkt_t *pPkt)
{
    static OrionPkt_t Pkt = { 0 };
    UInt8 Buffer;

    // As long as we keep getting bytes, keep reading them in one by one
    while (read(Handle, (char *)&Buffer, 1) == 1)
    {
        // If this byte is the end of a valid packet
        if (LookForOrionPacketInByte(&Pkt, Buffer))
        {
            // Copy the packet into the passed-in location and return a success
            *pPkt = Pkt;
            return TRUE;
        }
    }

    // Nope, no packets yet
    return FALSE;

}// LinuxCommReceive

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
