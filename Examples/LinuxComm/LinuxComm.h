#ifndef LINUXCOMM_H
#define LINUXCOMM_H

#include "OrionPacket.h"

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

#define UDP_OUT_PORT        8745
#define UDP_IN_PORT         8746
#define TCP_PORT            8747

int LinuxCommOpenSerial(const char *pPath);
int LinuxCommOpenNetwork(void);
void LinuxCommClose(int Handle);
void LinuxCommSend(int Handle, const OrionPkt_t *pPkt);
BOOL LinuxCommReceive(int Handle, OrionPkt_t *pPkt);

#endif // LINUXCOMM_H
