/**
 * \file
 * Brief description. Longer description.
 *
 * 
 * \author $Author$
 * \date $Date$
 */




#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include "udpclient.h"

static const char *serverAddr = "127.0.0.1";
int outport=4321;

bool udpSend(const char *hostip, int port, const char *msg){
    sockaddr_in servaddr;
    int fd = socket(AF_INET,SOCK_DGRAM,0);
    if(fd<0){
        perror("cannot open socket");
        return false;
    }
    
    
    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(hostip);
    servaddr.sin_port = htons(port);
    
    char opt=1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(int));
          
    if (sendto(fd, msg, strlen(msg)+1, 0, // +1 to include terminator
               (sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        perror("cannot send message");
        close(fd);
        return false;
    }
    close(fd);
    return true;
}

/// a useful function to get a UNIX timestamp with fractional seconds.
static double gettime(){
    timespec ts;
    clock_gettime(CLOCK_REALTIME,&ts);
    
    double t = ts.tv_nsec;
    t *= 1e-9;
    t += ts.tv_sec;
    return t;
}

/// a useful function to write a UDP packet prefixed by a timestamp
/// and formatted with printf semantics.
void udpwrite(const char *s,...){
    va_list args;
    va_start(args,s);
    char buf[1024];
    sprintf(buf,"time=%f ",gettime());
    vsnprintf(buf+strlen(buf),1024-strlen(buf),s,args);
    //printf("%s\n",buf);
    udpSend(serverAddr,outport,buf); // SEND TO 
    va_end(args);
}

