//
//  RosMessages.h
//  TurtleBotController
//
//  Created by Manuel Deneu on 11/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//

#ifndef RosMessages_h
#define RosMessages_h


/*
 Definitions are mostly taken from here :
 https://github.com/ROBOTIS-GIT/OpenCR/tree/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/sensor_msgs
 
 */
#include <stdint.h>


typedef enum
{
    ID_Topic = 0,
    
    ID_SensorState = 125,
    ID_IMU = 126,
    
    /*
     127?
     128
     129
     130
     
     
     */
    
} TurtleBot_TopicID;

/*
 
 The following structs & methods are taken from Turtlebot OpenCV Firmware :
 https://github.com/ROBOTIS-GIT/OpenCR/
 */

// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/ros/time.h
typedef struct
{
    uint32_t sec;
    uint32_t nsec;
}RosTime;

// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/std_msgs/Header.h
typedef struct
{
    uint32_t seq;
    RosTime stamp;
    const char* frame_id;
} RosHeader;

// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/rosserial_msgs/TopicInfo.h
typedef struct
{
    uint16_t topic_id;
    const char* topic_name;
    const char* message_type;
    const char* md5sum;
    int32_t buffer_size;
    
}RosTopicInfo;

// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/geometry_msgs/Quaternion.h
typedef struct
{
    float x;
    float y;
    float z;
    float w;
}RosQuaternion;

// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/geometry_msgs/Vector3.h
typedef struct
{
    float x;
    float y;
    float z;
    
}RosVector3;


// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/sensor_msgs/Imu.h
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





// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/geometry_msgs/Twist.h
// For usage example : check // https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/scripts/turtlebot3_teleop_key
typedef struct
{
    RosVector3 linear;
    RosVector3 angular;
} RosTwist;


int deserializeTopic(unsigned char *inbuffer);
int deserializeSensorState(unsigned char *inbuffer);
int deserializeIMU(unsigned char *inbuffer);
int deserializeHeader(unsigned char *inbuffer, RosHeader* header);

int deserializeVector3(unsigned char *inbuffer , RosVector3* vector);
int serializeVector3(unsigned char *outbuffer, const RosVector3* vector);

int deserializeAvrFloat64(const unsigned char* inbuffer, float* f);
int serializeAvrFloat64(unsigned char* outbuffer, const float f);


int serializeRosTwist(unsigned char *outbuffer , const RosTwist* twist);


#endif /* RosMessages_h */
