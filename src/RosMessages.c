//
//  RosMessages.c
//  TurtleBotController
//
//  Created by Manuel Deneu on 11/01/2018.
//  Copyright © 2018 Manuel Deneu. All rights reserved.
//

#include <string.h>
#include <stdio.h>
#include "RosMessages.h"

/* **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** */

// https://github.com/ROBOTIS-GIT/OpenCR/blob/3efe545bb3299d6a4841fd7a0943fac6972a3150/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros_lib/ros/msg.h

int deserializeAvrFloat64(const unsigned char* inbuffer, float* f)
{
    uint32_t* val = (uint32_t*)f;
    inbuffer += 3;
    
    // Copy truncated mantissa.
    *val = ((uint32_t)(*(inbuffer++)) >> 5 & 0x07);
    *val |= ((uint32_t)(*(inbuffer++)) & 0xff) << 3;
    *val |= ((uint32_t)(*(inbuffer++)) & 0xff) << 11;
    *val |= ((uint32_t)(*inbuffer) & 0x0f) << 19;
    
    // Copy truncated exponent.
    uint32_t exp = ((uint32_t)(*(inbuffer++)) & 0xf0)>>4;
    exp |= ((uint32_t)(*inbuffer) & 0x7f) << 4;
    if (exp != 0)
    {
        *val |= ((exp) - 1023 + 127) << 23;
    }
    
    // Copy negative sign.
    *val |= ((uint32_t)(*(inbuffer++)) & 0x80) << 24;
    
    return 8;
}


int serializeAvrFloat64(unsigned char* outbuffer, const float f)
{
    const int32_t* val = (int32_t*) &f;
    int32_t exp = ((*val >> 23) & 255);
    if (exp != 0)
    {
        exp += 1023 - 127;
    }
    
    int32_t sig = *val;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = (sig << 5) & 0xff;
    *(outbuffer++) = (sig >> 3) & 0xff;
    *(outbuffer++) = (sig >> 11) & 0xff;
    *(outbuffer++) = ((exp << 4) & 0xF0) | ((sig >> 19) & 0x0F);
    *(outbuffer++) = (exp >> 4) & 0x7F;
    
    // Mark negative bit as necessary.
    if (f < 0)
    {
        *(outbuffer - 1) |= 0x80;
    }
    
    return 8;
}

/* **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** */

// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/rosserial_msgs/TopicInfo.h

int deserializeTopic(unsigned char *inbuffer)
{
    
    RosTopicInfo topicInfo;
    /*
    uint16_t topic_id;
    const char* topic_name;
    const char* message_type;
    const char* md5sum;
    int32_t buffer_size;
     */
    
    int offset = 0;
    topicInfo.topic_id =  ((uint16_t) (*(inbuffer + offset)));
    topicInfo.topic_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
    
    offset += sizeof( topicInfo.topic_id );
    uint32_t length_topic_name;
    memcpy(&length_topic_name, (inbuffer + offset), sizeof(uint32_t));
    offset += 4;
    for(unsigned int k= offset; k< offset+length_topic_name; ++k){
        inbuffer[k-1]=inbuffer[k];
    }
    inbuffer[offset+length_topic_name-1]=0;
    topicInfo.topic_name = (char *)(inbuffer + offset-1);
    offset += length_topic_name;
    uint32_t length_message_type;
    memcpy(&length_message_type, (inbuffer + offset), sizeof(uint32_t));
    offset += 4;
    for(unsigned int k= offset; k< offset+length_message_type; ++k){
        inbuffer[k-1]=inbuffer[k];
    }
    inbuffer[offset+length_message_type-1]=0;
    topicInfo.message_type = (char *)(inbuffer + offset-1);
    offset += length_message_type;
    uint32_t length_md5sum;
    memcpy(&length_md5sum, (inbuffer + offset), sizeof(uint32_t));
    offset += 4;
    for(unsigned int k= offset; k< offset+length_md5sum; ++k){
        inbuffer[k-1]=inbuffer[k];
    }
    inbuffer[offset+length_md5sum-1]=0;
    topicInfo.md5sum = (char *)(inbuffer + offset-1);
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
    topicInfo.buffer_size = u_buffer_size.real;
    offset += sizeof( topicInfo.buffer_size);
    
    printf("Got topic_id %i topic_name '%s' message_type '%s'\n" , topicInfo.topic_id , topicInfo.topic_name , topicInfo.message_type );
    
    
    return offset;
}

/* **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** */


int deserializeSensorState(unsigned char *inbuffer)
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
    
    
    //printf(" %i %i\n " , left_encoder  , right_encoder);
    return offset;
}

/* **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** */

// // https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/std_msgs/Header.h

int deserializeHeader(unsigned char *inbuffer , RosHeader* header)
{
    int offset = 0;
    header->seq =  ((uint32_t) (*(inbuffer + offset)));
    header->seq |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    header->seq |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    header->seq |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    offset += sizeof(header->seq);
    
    header->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
    header->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    header->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    header->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    offset += sizeof(header->stamp.sec);
    
    header->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
    header->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
    header->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
    header->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
    offset += sizeof(header->stamp.nsec);
    uint32_t length_frame_id;
    memcpy(&length_frame_id, (inbuffer + offset), sizeof(uint32_t));
    offset += 4;
    for(unsigned int k= offset; k< offset+length_frame_id; ++k){
        inbuffer[k-1]=inbuffer[k];
    }
    inbuffer[offset+length_frame_id-1]=0;
    header->frame_id = (char *)(inbuffer + offset-1);
    offset += length_frame_id;
    return offset;
}

/* **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** */

// // https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/geometry_msgs/Quaternion.h

int deserializeQuaternion(unsigned char *inbuffer , RosQuaternion *quaternion)
{
    int offset = 0;
    offset += deserializeAvrFloat64(inbuffer + offset, &(quaternion->x));
    offset += deserializeAvrFloat64(inbuffer + offset, &(quaternion->y));
    offset += deserializeAvrFloat64(inbuffer + offset, &(quaternion->z));
    offset += deserializeAvrFloat64(inbuffer + offset, &(quaternion->w));
    return offset;
}

/* **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** */

// https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/sensor_msgs/Imu.h

int deserializeIMU(unsigned char *inbuffer)
{
    RosIMU imu;
    
    int offset = 0;
    offset += deserializeHeader( inbuffer + offset ,&imu.header);           // this->header.deserialize(inbuffer + offset);
    offset +=  deserializeQuaternion(inbuffer + offset, &imu.orientation);  // this->orientation.deserialize(inbuffer + offset);
    for( uint32_t i = 0; i < 9; i++)
    {
        offset += deserializeAvrFloat64(inbuffer + offset, &(imu.orientation_covariance[i] ) );
    }
    offset += deserializeVector3(inbuffer + offset, &imu.angular_velocity);// this->angular_velocity.deserialize(inbuffer + offset);
    
    for( uint32_t i = 0; i < 9; i++)
    {
        offset += deserializeAvrFloat64(inbuffer + offset, &(imu.angular_velocity_covariance[i]));
    }
    offset += deserializeVector3(inbuffer + offset , &imu.linear_acceleration);// this->linear_acceleration.deserialize(inbuffer + offset);
    
    for( uint32_t i = 0; i < 9; i++)
    {
        offset += deserializeAvrFloat64(inbuffer + offset, &(imu.linear_acceleration_covariance[i] ) );
    }
    
    printf("IMU %i (%f,%f,%f,%f) \n" , imu.header.stamp.sec , imu.orientation.w , imu.orientation.x ,imu.orientation.y ,imu.orientation.z);
    
    return offset;
}

/* **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** */

// // https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/geometry_msgs/Vector3.h

int deserializeVector3(unsigned char *inbuffer , RosVector3* vector)
{
    int offset = 0;
    offset += deserializeAvrFloat64(inbuffer + offset, &(vector->x));
    offset += deserializeAvrFloat64(inbuffer + offset, &(vector->y));
    offset += deserializeAvrFloat64(inbuffer + offset, &(vector->z));
    return offset;
}

int serializeVector3(unsigned char *outbuffer, const RosVector3* vector) 
{
    int offset = 0;
    offset += serializeAvrFloat64(outbuffer + offset, vector->x);
    offset += serializeAvrFloat64(outbuffer + offset, vector->y);
    offset += serializeAvrFloat64(outbuffer + offset, vector->z);
    return offset;
}

/* **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** */

// // https://github.com/ROBOTIS-GIT/OpenCR/blob/aee618962039549d42f2663e0339f4d45291c4b6/arduino/opencr_fw/opencr_fw_arduino/src/arduino/libraries/turtlebot3_ros_lib/geometry_msgs/Twist.h

int serializeRosTwist(unsigned char *outbuffer , const RosTwist* twist)
{
    int offset = 0;
    offset += serializeVector3(outbuffer+ offset, &twist->linear);
    offset += serializeVector3(outbuffer + offset ,&twist->angular );
    return offset;
}
