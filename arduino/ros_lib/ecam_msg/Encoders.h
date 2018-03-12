#ifndef _ROS_ecam_msg_Encoders_h
#define _ROS_ecam_msg_Encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ecam_msg
{

  class Encoders : public ros::Msg
  {
    public:
      typedef int64_t _front_left_type;
      _front_left_type front_left;
      typedef int64_t _front_right_type;
      _front_right_type front_right;
      typedef int64_t _rear_left_type;
      _rear_left_type rear_left;
      typedef int64_t _rear_right_type;
      _rear_right_type rear_right;

    Encoders():
      front_left(0),
      front_right(0),
      rear_left(0),
      rear_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_front_left;
      u_front_left.real = this->front_left;
      *(outbuffer + offset + 0) = (u_front_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_left.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_front_left.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_front_left.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_front_left.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_front_left.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->front_left);
      union {
        int64_t real;
        uint64_t base;
      } u_front_right;
      u_front_right.real = this->front_right;
      *(outbuffer + offset + 0) = (u_front_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_right.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_front_right.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_front_right.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_front_right.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_front_right.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->front_right);
      union {
        int64_t real;
        uint64_t base;
      } u_rear_left;
      u_rear_left.real = this->rear_left;
      *(outbuffer + offset + 0) = (u_rear_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_left.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rear_left.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rear_left.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rear_left.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rear_left.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rear_left);
      union {
        int64_t real;
        uint64_t base;
      } u_rear_right;
      u_rear_right.real = this->rear_right;
      *(outbuffer + offset + 0) = (u_rear_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_right.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rear_right.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rear_right.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rear_right.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rear_right.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rear_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_front_left;
      u_front_left.base = 0;
      u_front_left.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_left.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_left.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_left.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_front_left.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_front_left.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_front_left.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_front_left.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->front_left = u_front_left.real;
      offset += sizeof(this->front_left);
      union {
        int64_t real;
        uint64_t base;
      } u_front_right;
      u_front_right.base = 0;
      u_front_right.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_right.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_right.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_right.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_front_right.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_front_right.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_front_right.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_front_right.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->front_right = u_front_right.real;
      offset += sizeof(this->front_right);
      union {
        int64_t real;
        uint64_t base;
      } u_rear_left;
      u_rear_left.base = 0;
      u_rear_left.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_left.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_left.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_left.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rear_left.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rear_left.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rear_left.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rear_left.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rear_left = u_rear_left.real;
      offset += sizeof(this->rear_left);
      union {
        int64_t real;
        uint64_t base;
      } u_rear_right;
      u_rear_right.base = 0;
      u_rear_right.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_right.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_right.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_right.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rear_right.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rear_right.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rear_right.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rear_right.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rear_right = u_rear_right.real;
      offset += sizeof(this->rear_right);
     return offset;
    }

    const char * getType(){ return "ecam_msg/Encoders"; };
    const char * getMD5(){ return "189368fd89d1fdc2727ab57efab8cd75"; };

  };

}
#endif