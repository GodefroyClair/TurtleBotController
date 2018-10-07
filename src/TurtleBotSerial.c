//
//  TurtleBotSerial.c
//  TurtleBotController
//
//  Created by Manuel Deneu on 11/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//
#include <string.h>
#include <sys/termios.h>
#include <fcntl.h> // O_RDWR | O_NOCTTY | O_NDELAY
#include <unistd.h>
#include "TurtleBotSerial.h"


static int setInterfaceAttributes (int fd, int speed, int parity);

int openSerialPort( const char* portName)
{
    if( portName ==  NULL)
        return -1;
    
    int fd =  open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (fd <=0 )
    {
        return -1;
    }

    if( setInterfaceAttributes (fd, TURTLEBOT_DEFAULT_BAUDRATE, 0) == 0)
    {
        printf("Error set_interface_attribs\n");
    }
    
    return fd;   
}

static int setInterfaceAttributes (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //perror("tcgetattr");
        perror("tggetattr");
        return 0;
    }
    
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        
        return 0;
    }
    return 1;
}


/* *** *** *** *** *** *** *** *** *** *** *** *** *** *** *** *** *** *** *** */

int sendTopicsRequest( int fd)
{
    if( fd <= 0)
        return 0;
    
    return  write(fd, REQ_TOPIC_V2, REQ_TOPIC_V2_SIZE) == REQ_TOPIC_V2_SIZE;
}

int runProcessLoop( int fd , int* stopFlag , OnDataCallback dataCallback)
{
    if( fd <= 0)
        return 0;
    if( stopFlag == NULL)
        return 0;
    if( dataCallback == NULL)
        return 0;

    int run = 1;
    
    while (run && (*stopFlag == 0) )
    {
        
        char startByte = 0;
        ssize_t ret = read(fd, &startByte, 1);
        
        if( ret == 0)
        {
            continue;
        }
        
        if( startByte != START_FLAG)
        {
            continue;
        }
        
        char ver = 0;
        
        ret = read(fd, &ver, 1);
        
        if( ver == PROTO_V1)
        {
            //printf("protocol_ver1\n");
        }
        else if( ver == PROTO_V2)
        {
            //printf("protocol_ver2\n");
        }
        else
        {
            printf("Protocol version unknown\n");
            continue;
        }
        
        /* **** **** **** **** **** **** **** */
        
        int16_t payloadLen = 0;
        
        ret = read(fd, &payloadLen, 2);
        
        if( ret != 2)
        {
            printf("Error reading len : bytes read : %zi\n" ,ret);
            continue;
        }
        
        if (payloadLen <= 0)
        {
            continue;
        }
        
        if(payloadLen > MAX_PAYLOAD_SIZE)
        {
            printf("Msg len > MaxPayloadSize %zi %zi\n" , payloadLen , MAX_PAYLOAD_SIZE  );
            continue;
        }
        //assert(payloadLen <= MaxPayloadSize);
        
        /* **** **** **** **** **** **** **** */
        
        uint8_t msg_len_checksum  = 0;
        
        ret = read(fd, &msg_len_checksum, 1);
        
        if( ret != 1)
        {
            printf("Error reading msg_len_checksum : bytes read : %zi\n" ,ret);
            continue;
        }
        
        /* **** **** **** **** **** **** **** */
        
        int16_t topic_id = -1;
        
        ret = read(fd, &topic_id, 2);
        
        if( ret != 2)
        {
            printf("Error reading topic_id : bytes read : %zi\n" ,ret);
            continue;
        }
        
        /* **** **** **** **** **** **** **** */
        
        static uint8_t msgBuf[MAX_PAYLOAD_SIZE] = { 0};
        
        memset(msgBuf, 0,MAX_PAYLOAD_SIZE);
        ret = read(fd, msgBuf, payloadLen);
        
        if( ret != payloadLen)
        {
            //printf("Error reading Payload : got %zi instead of %zi \n" , ret ,payloadLen);
            continue;
        }
        
        
        uint8_t msg_checksum = 0;
        
        ret = read(fd, &msg_checksum, 1);
        
        if( ret != 1)
        {
            printf("Error reading payload checksum \n");
            continue;
        }
        
        
        if(dataCallback(topic_id , msgBuf , payloadLen) == 0)
        {
            run = 0;
            break;
        }
        /*
        
        if( topic_id == ID_Topic)
        {
            deserializeTopic( msgBuf);
        }
        else if( topic_id == ID_SensorState) // SensorState
        {
            deserializeSensorState(msgBuf);
        }
        
        else if( topic_id == ID_IMU) // IMU
        {
            deserializeIMU(msgBuf);
        }
        
        else
        {
            
            printf("Topic id %i\n",  topic_id);
            printf("Msg size %i \n" , payloadLen);
            //printf("BUf '%s'\n" ,(const char*) msgBuf);
            
            
            for( int i =0; i<(int)payloadLen ; i++)
            {
                if( (i % 8 ) == 0)
                {
                    //printf("\n");
                }
                printf("%c",(char)( msgBuf[i] == 0? '?' :msgBuf[i] ) );
            }
            for( int i =0; i<(int)payloadLen ; i++)
            {
                if( (i % 8 ) == 0)
                {
                    printf("\n");
                }
                printf(" %x ",msgBuf[i]);
            }
            
            printf("\n");
        }
         */
        
    }
    
    *stopFlag = 0;
    
    return 1;
}

int sendCommands( int fd, const RosTwist* cmd)
{
    if( fd <= 0 || cmd == NULL)
        return 0;
    
    return sendMessage(fd, ID_cmd_vel_rc100, (const unsigned char*) cmd, sizeof(cmd));
}


int sendMessage( int fd, int topic, const unsigned char* msgBuffer , size_t messageSize)
{
    if( fd <= 0 || msgBuffer == NULL || topic < 0 || messageSize == 0)
        return 0;
    
    if( messageSize > MAX_PAYLOAD_SIZE)
    {
        fprintf(stderr, "[sendMessage] Warning : message size (%zi) exceeds MAX_PAYLOAD_SIZE  (%zi) " , messageSize , MAX_PAYLOAD_SIZE);
        return 0;
    }
                                    ;
    const size_t totalMsgSize = MSG_HeaderSize + messageSize + 1;
    
    // msg_len_checksum = 255 - ( ((length&255) + (length>>8))%256 )
    const uint32_t l =  (messageSize&255) + (messageSize>>8);
    
    const char msg_len_checksum = 255 - ( l%256 );
    
    
    uint32_t sumCh = topic&255;
    sumCh += topic>>8;
    
    
    for( size_t i =0;i<messageSize; ++i )
    {
        sumCh += msgBuffer[i];
    }
    //msg_checksum = 255 - ( ((topic&255) + (topic>>8) + sum([ord(x) for x in msg]))%256 )
    const char msg_checksum = 255 - ( sumCh % 256 );
                                       
    
    
    static char msgBuf[MAX_PAYLOAD_SIZE + MSG_HeaderSize] = { 0};
    
    memset(msgBuf, 0,MAX_PAYLOAD_SIZE);

    msgBuf[0] = START_FLAG;
    msgBuf[1] = PROTO_V2;
    msgBuf[2] = (char) messageSize&255;
    msgBuf[3] = (char) messageSize>>8;
    msgBuf[4] = msg_len_checksum;
    msgBuf[5] = (char) topic&255;
    msgBuf[6] = (char) topic>>8;
    
    const int offset = 7;
    memcpy(msgBuf + offset, msgBuffer, messageSize);
    
    msgBuf[offset + messageSize] = msg_checksum;
    
    /*
     msg_checksum = 255 - ( ((topic&255) + (topic>>8) + sum([ord(x) for x in msg]))%256 )
     data = "\xff" + self.protocol_ver  + chr(length&255) + chr(length>>8) + chr(msg_len_checksum) + chr(topic&255) + chr(topic>>8)
     data = data + msg + chr(msg_checksum)
     self.port.write(data)
     */
    
    /*
    printf("send Message size %zi\n" , totalMsgSize);
    
    
    for( size_t i=0; i<totalMsgSize;++i)
    {
        
        if( i % 8 == 0)
        {
            printf("\n");
        }
        printf(" %x " , (unsigned char) msgBuf[i]);
    }
     */
    
    return write(fd, msgBuf, totalMsgSize) == totalMsgSize;
}




/*
 void set_blocking (int fd, int should_block)
 {
 struct termios tty;
 memset (&tty, 0, sizeof tty);
 if (tcgetattr (fd, &tty) != 0)
 {
 perror("tggetattr");
 //error_message ("error %d from tggetattr", errno);
 return;
 }
 
 tty.c_cc[VMIN]  = should_block ? 1 : 0;
 tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
 
 if (tcsetattr (fd, TCSANOW, &tty) != 0)
 {
 perror("setting term attributes");
 //error_message ("error %d setting term attributes", errno);
 }
 }
 */
