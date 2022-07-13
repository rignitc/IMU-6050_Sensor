#ifndef _ROS_SERVICE_Attach_h
#define _ROS_SERVICE_Attach_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_ros_link_attacher
{

static const char ATTACH[] = "gazebo_ros_link_attacher/Attach";

  class AttachRequest : public ros::Msg
  {
    public:
      typedef const char* _model_name_1_type;
      _model_name_1_type model_name_1;
      typedef const char* _link_name_1_type;
      _link_name_1_type link_name_1;
      typedef const char* _model_name_2_type;
      _model_name_2_type model_name_2;
      typedef const char* _link_name_2_type;
      _link_name_2_type link_name_2;

    AttachRequest():
      model_name_1(""),
      link_name_1(""),
      model_name_2(""),
      link_name_2("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_model_name_1 = strlen(this->model_name_1);
      varToArr(outbuffer + offset, length_model_name_1);
      offset += 4;
      memcpy(outbuffer + offset, this->model_name_1, length_model_name_1);
      offset += length_model_name_1;
      uint32_t length_link_name_1 = strlen(this->link_name_1);
      varToArr(outbuffer + offset, length_link_name_1);
      offset += 4;
      memcpy(outbuffer + offset, this->link_name_1, length_link_name_1);
      offset += length_link_name_1;
      uint32_t length_model_name_2 = strlen(this->model_name_2);
      varToArr(outbuffer + offset, length_model_name_2);
      offset += 4;
      memcpy(outbuffer + offset, this->model_name_2, length_model_name_2);
      offset += length_model_name_2;
      uint32_t length_link_name_2 = strlen(this->link_name_2);
      varToArr(outbuffer + offset, length_link_name_2);
      offset += 4;
      memcpy(outbuffer + offset, this->link_name_2, length_link_name_2);
      offset += length_link_name_2;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_model_name_1;
      arrToVar(length_model_name_1, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_name_1; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_name_1-1]=0;
      this->model_name_1 = (char *)(inbuffer + offset-1);
      offset += length_model_name_1;
      uint32_t length_link_name_1;
      arrToVar(length_link_name_1, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name_1; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name_1-1]=0;
      this->link_name_1 = (char *)(inbuffer + offset-1);
      offset += length_link_name_1;
      uint32_t length_model_name_2;
      arrToVar(length_model_name_2, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_name_2; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_name_2-1]=0;
      this->model_name_2 = (char *)(inbuffer + offset-1);
      offset += length_model_name_2;
      uint32_t length_link_name_2;
      arrToVar(length_link_name_2, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name_2; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name_2-1]=0;
      this->link_name_2 = (char *)(inbuffer + offset-1);
      offset += length_link_name_2;
     return offset;
    }

    virtual const char * getType() override { return ATTACH; };
    virtual const char * getMD5() override { return "ff39d0bc8e054b10e21a2f298cb7fb05"; };

  };

  class AttachResponse : public ros::Msg
  {
    public:
      typedef bool _ok_type;
      _ok_type ok;

    AttachResponse():
      ok(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
     return offset;
    }

    virtual const char * getType() override { return ATTACH; };
    virtual const char * getMD5() override { return "6f6da3883749771fac40d6deb24a8c02"; };

  };

  class Attach {
    public:
    typedef AttachRequest Request;
    typedef AttachResponse Response;
  };

}
#endif
