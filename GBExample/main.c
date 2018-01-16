//
//  main.c
//  TurtleBotController
//
//  Created by Manuel Deneu on 10/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//

#include <stdio.h>

#include <GBThread.h>
#include <GBFDSource.h>
#include <GBRunLoop.h>
#include <unistd.h> // write read close
#include <string.h> // memset

#include "TurtleBotSerial.h"
#include "RosMessages.h"


typedef struct
{
    int fd;
    
    const char* portName;
    
    RosTwist cmd;
}TurtleContext;



static TurtleContext context;

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
        
        
        
        
        
    }
    return 1;
}

static void sendCommand( TurtleContext* context)
{
    
    
    
    static unsigned char out[MAX_PAYLOAD_SIZE];
    memset(&out, 0, MAX_PAYLOAD_SIZE);
    
    const int size = serializeRosTwist(out, &context->cmd);
    
    
    if (sendMessage(context->fd, 100, out, size) == 0)
    {
        perror("Send");
    }
}

static void turtleMain( GBThread *self)
{
    
    
    printf("try open '%s' \n" ,context.portName);
    
    context.fd = openSerialPort( context.portName );
    
    if( context.fd < 0 )
    {
        printf("Error while opening port '%s'\n" , context.portName);
        return;
    }
    
    if( sendTopicsRequest(context.fd) == 0)
    {
        printf("Error sending topics request\n");
    }
    
    
    int stop = 0;
    runProcessLoop( context.fd, &stop, onData);
}

static void onKey( GBRunLoopSource* source , GBRunLoopSourceNotification notification)
{
    GBFDSource* src = (GBFDSource*) source;
    
    static char buf[64] = { 0};
    
    const GBSize sizeRead = GBFDSourceRead(src, &buf, 64);
    if( sizeRead)
    {
        buf[sizeRead-1] = 0;
        
        if( buf[0] == 'z')
        {
            printf("Got forward\n");
            
            context.cmd.linear.x += 0.1;
            
            sendCommand(&context);
        }
        else if( buf[0] == 's')
        {
            printf("Got backward\n");
            context.cmd.linear.x -= 0.1;
            
            sendCommand(&context);
        }
        else if( buf[0] == 'q')
        {
            context.cmd.angular.z += 0.1;
            sendCommand(&context);
            printf("turn left\n");
        }
        else if( buf[0] == 'd')
        {
            context.cmd.angular.z -= 0.1;
            sendCommand(&context);
            printf("turn right\n");
        }
        else if( buf[0] == ' ')
        {
            memset(&context.cmd, 0, sizeof(RosTwist));
            sendCommand(&context);
            printf("stop\n");
        }
        else if( buf[0] == 't')
        {
            
             
            sendTopicsRequest(context.fd);
        }
        else if( buf[0] == 'p')
        {
            GBRunLoopStop( GBRunLoopSourceGetRunLoop(source));
        }
            
    }
    
}

int main(int argc, const char * argv[])
{
    memset(&context.cmd, 0, sizeof(RosTwist));
    
    context.portName = argc > 1? argv[1] : "/dev/tty.usbmodem14441";
    
    GBThread* turtleThread = GBThreadInit();
    
    GBThreadSetMain(turtleThread, turtleMain);
    
    GBThreadStart(turtleThread);
    
    GBFDSource* inSource = GBFDSourceInitWithFD(STDIN_FILENO, onKey);
    
    
    GBRunLoop* rl = GBRunLoopInit();
    GBRunLoopAddSource(rl, inSource);
    
    GBRunLoopRun(rl);
    
    memset(&context.cmd, 0, sizeof(RosTwist));
    sendCommand(&context);
    
    sleep(1);
    
    close( context.fd);
    
    GBRelease(turtleThread);
    GBRelease(inSource);
    GBRelease(rl);
    return 0;
}







