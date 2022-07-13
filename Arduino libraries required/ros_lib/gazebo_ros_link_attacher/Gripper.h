#ifndef _ROS_SERVICE_Gripper_h
#define _ROS_SERVICE_Gripper_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_ros_link_attacher
{

static const char GRIPPER[] = "gazebo_ros_link_attacher/Gripper";

  class GripperRequest : public ros::Msg
  {
    public:
      typedef bool _activate_gripper_type;
      _activate_gripper_type activate_gripper;

    GripperRequest():
      activate_gripper(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_activate_gripper;
      u_activate_gripper.real = this->activate_gripper;
      *(outbuffer + offset + 0) = (u_activate_gripper.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->activate_gripper);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_activate_gripper;
      u_activate_gripper.base = 0;
      u_activate_gripper.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->activate_gripper = u_activate_gripper.real;
      offset += sizeof(this->activate_gripper);
     return offset;
    }

    virtual const char * getType() override { return GRIPPER; };
    virtual const char * getMD5() override { return "923c733a75af78e3834068ddb1235791"; };

  };

  class GripperResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    GripperResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return GRIPPER; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class Gripper {
    public:
    typedef GripperRequest Request;
    typedef GripperResponse Response;
  };

}
#endif
