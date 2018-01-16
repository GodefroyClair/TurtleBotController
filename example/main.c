//
//  main.c
//  TurtleBotController
//
//  Created by Manuel Deneu on 10/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//

#include <stdio.h>


#include <unistd.h> // write read close
#include <string.h> // memset

#include "TurtleBotSerial.h"
#include "RosMessages.h"


static int fd  = 0;
// return 0 to stop the loop
static int onData( int topicID , unsigned char *inbuffer )
{
    //printf("Received Topic #%i\n" , topicID);
    
    if (topicID == 0)
    {
        deserializeTopic(inbuffer);
    }
    else if( topicID == ID_cmd_vel_rc100)
    {
        
        
        RosTwist twist;
        
        deserializeRosTwist(inbuffer, &twist);
        
        printf("Got ID_cmd_vel_rc100 msg %f %f %f | %f %f %f \n",
               twist.linear.x,
               twist.linear.y,
               twist.linear.z,
               twist.angular.x,
               twist.angular.y,
               twist.angular.z
               );
        
        
        
        RosTwist cmd;
        memset(&cmd, 0, sizeof(RosTwist));
        
        cmd.linear.x = 0;
        
        
        static unsigned char out[MAX_PAYLOAD_SIZE];
        memset(&out, 0, MAX_PAYLOAD_SIZE);
        
        const int size = serializeRosTwist(out, &cmd);
        
        
        if (sendMessage(fd, 100, out, size) == 0)
        //if(sendCommands(fd, &cmd) == 0)
        {
            perror("Send");
        }
        
    }
    return 1;
}

int main(int argc, const char * argv[])
{
    
    const char* portName = argc > 1? argv[1] : "/dev/tty.usbmodem14441";
    
    printf("try open '%s' \n" ,portName);
    
    fd = openSerialPort(portName);
   
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



