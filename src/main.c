//
//  main.c
//  TurtleBotController
//
//  Created by Manuel Deneu on 10/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//

#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h> //ioctl() call defenitions
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <GroundBase.h>

#include "RosMessages.h"




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

int
set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //perror("tcgetattr");
        perror("tggetattr");
        return -1;
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
        
        return -1;
    }
    return 0;
}
/*
int setRTS( int fd , int v, int millis)
{
    int RTS_flag;
    RTS_flag = TIOCM_RTS;
    
    ioctl(fd,TIOCMBIC,&RTS_flag);//Clear RTS pin
    usleep(1000*millis);
    ioctl(fd,TIOCMBIS,&RTS_flag);//Set RTS pin
    
    usleep(1000*millis);
    return -1;
    
}
*/


int main(int argc, const char * argv[])
{
    
    const char* portName = "/dev/tty.usbmodem14311";
    
    printf("try open '%s' \n" ,portName);
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    
    
    printf("open did return fd %i \n" , fd);
    if (fd <=0 )
        printf("Error open port '%s'\n" , portName);
    
    
    if(set_interface_attribs (fd, B57600, 0) != 0)  // set speed to 115,200 bps, 8n1 (no parity)
    {
        printf("Error set_interface_attribs\n");
    }
    
    
    
    //const char* requestTopicsV1 = "\xff\xff\x00\x00\x00\x00\xff";
    const char* requestTopicsV2 = "\xff\xfe\x00\x00\xff\x00\x00\xff";
    //self.port.write("\xff" + self.protocol_ver + "\x00\x00\xff\x00\x00\xff")
    if( write(fd, requestTopicsV2, 8) > 0 )
    {
        printf("Write ok");
    }
    
    //sleep(1);
    
    
    const char flag = '\xff';
    
    const char protocol_ver1 = 0xff;
    const char protocol_ver2 = 0xfe;
    
    const size_t sizeToRead = 1;
    char buf[1] = { '0'};
    
    
    
    const size_t MaxPayloadSize = 704;
    while (1)
    {
        
        
        //usleep(1000*200);
        ssize_t ret = read(fd, buf, sizeToRead);
        
        if( ret == 0)
        {
            continue;
        }
        //printf("Read %zi bytes '%s' \n", ret , buf);
        
        if( buf[0] != flag)
        {
            continue;
        }
        
        //printf("Got flag %x  %x\n" , flag , buf[0]);
    
        char ver = 0;
        
        ret = read(fd, &ver, 1);
        
        
        //printf("Version is %x -> " , ver);
        
        if( ver == protocol_ver1)
        {
            //printf("protocol_ver1\n");
        }
        else if( ver == protocol_ver2)
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
        
        
        if(payloadLen > MaxPayloadSize)
        {
            printf("Msg len > MaxPayloadSize %zi %zi\n" , payloadLen , MaxPayloadSize  );
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
        
        //printf("Len Checksum : %i\n" , msg_len_checksum);
        
        /* **** **** **** **** **** **** **** */
        
        int16_t topic_id = -1;
        
        ret = read(fd, &topic_id, 2);
        
        if( ret != 2)
        {
            printf("Error reading topic_id : bytes read : %zi\n" ,ret);
            continue;
        }
        
        // Topic ID : 10
        
        
        
        
        /* **** **** **** **** **** **** **** */
        
        static uint8_t msgBuf[MaxPayloadSize] = { 0};
        
        
        memset(msgBuf, 0,MaxPayloadSize);
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
        
        
        
        //printf("-------------\n");
        
        
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
            continue;
            
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

    }

    close(fd);
    return 0;
}



