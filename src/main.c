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




typedef enum
{
    TopicID_
} TurtleBot_TopicID;

void
set_blocking (int fd, int should_block)
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

static int deserialize(unsigned char *inbuffer);
static int deserializeSensorState(unsigned char *inbuffer);


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
    
    
    
    const char* requestTopics = "\xff\xff\x00\x00\x00\x00\xff";
               const char* r2 = "\xff\xfe\x00\x00\xff\x00\x00\xff";
    //self.port.write("\xff" + self.protocol_ver + "\x00\x00\xff\x00\x00\xff")
    if( write(fd, r2, 8) > 0 )
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
        
        if( topic_id == 0)
        {
            deserialize( msgBuf);
        }
        else if( topic_id == 125)
        {
            deserializeSensorState(msgBuf);
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




static int deserialize(unsigned char *inbuffer)
{
    
    
    uint16_t topic_id;
    const char* topic_name;
    const char* message_type;
    const char* md5sum;
    int32_t buffer_size;
    
    int offset = 0;
    topic_id =  ((uint16_t) (*(inbuffer + offset)));
    topic_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
    
    offset += sizeof( topic_id );
    uint32_t length_topic_name;
    memcpy(&length_topic_name, (inbuffer + offset), sizeof(uint32_t));
    offset += 4;
    for(unsigned int k= offset; k< offset+length_topic_name; ++k){
        inbuffer[k-1]=inbuffer[k];
    }
    inbuffer[offset+length_topic_name-1]=0;
    topic_name = (char *)(inbuffer + offset-1);
    offset += length_topic_name;
    uint32_t length_message_type;
    memcpy(&length_message_type, (inbuffer + offset), sizeof(uint32_t));
    offset += 4;
    for(unsigned int k= offset; k< offset+length_message_type; ++k){
        inbuffer[k-1]=inbuffer[k];
    }
    inbuffer[offset+length_message_type-1]=0;
    message_type = (char *)(inbuffer + offset-1);
    offset += length_message_type;
    uint32_t length_md5sum;
    memcpy(&length_md5sum, (inbuffer + offset), sizeof(uint32_t));
    offset += 4;
    for(unsigned int k= offset; k< offset+length_md5sum; ++k){
        inbuffer[k-1]=inbuffer[k];
    }
    inbuffer[offset+length_md5sum-1]=0;
    md5sum = (char *)(inbuffer + offset-1);
    offset += length_md5sum;
    union {
        int32_t real;
        uint32_t base;
    } u_buffer_size;
    u_buffer_size.base = 0;
    u_buffer_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_buffer_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_buffer_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_buffer_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    buffer_size = u_buffer_size.real;
    offset += sizeof(buffer_size);
    
    printf("Got topic_id %i topic_name '%s' message_type '%s'\n" , topic_id , topic_name , message_type );
    
    
    return offset;
}




static int deserializeSensorState(unsigned char *inbuffer)
{
    
    uint32_t sec, nsec;
    
    uint8_t bumper;
    uint8_t cliff;
    uint8_t button;
    int32_t left_encoder;
    int32_t right_encoder;
    float battery;
    
    int offset = 0;
    
    sec =  ((uint32_t) (*(inbuffer + offset)));
    sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    
    offset += sizeof(sec);
    
    nsec =  ((uint32_t) (*(inbuffer + offset)));
    nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    
    offset += sizeof(nsec);
    bumper =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof( bumper);
    cliff =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(cliff);
    button =  ((uint8_t) (*(inbuffer + offset)));
    offset += sizeof(button);
    union {
        int32_t real;
        uint32_t base;
    } u_left_encoder;
    u_left_encoder.base = 0;
    u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    left_encoder = u_left_encoder.real;
    offset += sizeof(left_encoder);
    union {
        int32_t real;
        uint32_t base;
    } u_right_encoder;
    u_right_encoder.base = 0;
    u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    right_encoder = u_right_encoder.real;
    offset += sizeof( right_encoder);
    union {
        float real;
        uint32_t base;
    } u_battery;
    u_battery.base = 0;
    u_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
    u_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    u_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    u_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    battery = u_battery.real;
    offset += sizeof(battery);
    
    
    printf(" %i %i\n " , left_encoder  , right_encoder);
    return offset;
}
