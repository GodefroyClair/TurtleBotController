//
//  RosMessages.h
//  TurtleBotController
//
//  Created by Manuel Deneu on 11/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//

#ifndef RosMessages_h
#define RosMessages_h



typedef struct
{
    uint32_t sec;
    uint32_t nsec;
}RosTime;

typedef struct
{
    uint32_t seq;
    RosTime stamp;
    const char* frame_id;
} RosHeader;


typedef struct
{
    float x;
    float y;
    float z;
    float w;
}RosQuaternion;

typedef struct
{
    float x;
    float y;
    float z;
}RosVector3;

typedef struct
{
    RosHeader header;
    RosQuaternion orientation;
    float orientation_covariance[9];
    RosVector3 angular_velocity;
    float angular_velocity_covariance[9];
    RosVector3 linear_acceleration;
    float linear_acceleration_covariance[9];
    
} RosIMU;



#endif /* RosMessages_h */
