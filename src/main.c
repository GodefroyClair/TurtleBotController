//
//  main.c
//  TurtleBotController
//
//  Created by Manuel Deneu on 10/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//

#include <stdio.h>


//#include <sys/ioctl.h> //ioctl() call defenitions
#include <unistd.h> // write read close
#include <string.h> // memset




#include "TurtleBotSerial.h"
#include "RosMessages.h"


static int onData( int topicID , unsigned char *inbuffer )
{
    return 1;
}


int main(int argc, const char * argv[])
{
    
    const char* portName = argc > 1? argv[1] : "/dev/tty.usbmodem14311";
    
    printf("try open '%s' \n" ,portName);
    
    int fd = openSerialPort(portName);
   
    if( fd < 0 )
    {
        printf("Error while opening port '%s'\n" , portName);
        return 1;
    }
    
    if( sendTopicsRequest(fd) == 0)
    {
        printf("Error sending topics request\n");
    }

    
    int stop = 0;
    runProcessLoop(fd, &stop, onData);

    close(fd);
    return 0;
}



