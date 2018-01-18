//
//  TurtleBotSerial.h
//  TurtleBotController
//
//  Created by Manuel Deneu on 11/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//

#ifndef TurtleBotSerial_h
#define TurtleBotSerial_h

#include <stdio.h>
#include <termios.h> // B57600
#include "RosMessages.h"

#define TURTLEBOT_DEFAULT_BAUDRATE (int) B57600

#define PROTO_V1 (const char) 0xff
#define PROTO_V2 (const char) 0xfe

#define REQ_TOPIC_V1 (const char*) "\xff\xff\x00\x00\x00\x00\xff"
#define REQ_TOPIC_V2 (const char*) "\xff\xfe\x00\x00\xff\x00\x00\xff"

#define REQ_TOPIC_V1_SIZE (size_t) 7
#define REQ_TOPIC_V2_SIZE (size_t) 8

#define START_FLAG (const char) '\xff'


#define MAX_PAYLOAD_SIZE (size_t) 704


// this includes the last byte (msg checksum)
#define MSG_HeaderSize (size_t) 7
/*
+ 1 // byte xff
+ 2 // payload len
+ 1 // payload len checksum
+ 2 // topic (int16_t)
//+ messageSize
+ 1 // msg checksum
 */

typedef int (*OnDataCallback)( int topicID , unsigned char *inbuffer , size_t inbufferSize );

// returns -1 on error
int openSerialPort( const char* portName);



// 1 on sucess, 0 otherwise.
int sendTopicsRequest( int fd);

// 1 on sucess, 0 otherwise.
int sendCommands( int fd, const RosTwist* cmd);

// 1 on sucess, 0 otherwise.
// this method is NOT REANTRANT (use of static buffer)
int sendMessage( int fd, int topic, const unsigned char* msgBuffer , size_t messageSize);

// this method is NOT REANTRANT (use of static buffer)
int runProcessLoop( int fd , int* stopFlag , OnDataCallback dataCallback);



#endif /* TurtleBotSerial_h */
