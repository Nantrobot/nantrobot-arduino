#ifndef _ROS_joy_controller_joystick_h
#define _ROS_joy_controller_joystick_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace joy_controller
{

  class joystick : public ros::Msg
  {
    public:
      float stick1_h;
      float stick1_v;
      float stick2_h;
      float stick2_v;
      int32_t intStick_h;
      int32_t intStick_v;
      uint8_t square;
      uint8_t cross;
      uint8_t circle;
      uint8_t triangle;
      uint8_t L1;
      uint8_t R1;
      uint8_t L2;
      uint8_t R2;
      uint8_t SE;
      uint8_t ST;
      uint8_t stick1;
      uint8_t stick2;
      uint32_t mode_type1;
      uint32_t mode_type2;
      uint8_t on_state;

    joystick():
      stick1_h(0),
      stick1_v(0),
      stick2_h(0),
      stick2_v(0),
      intStick_h(0),
      intStick_v(0),
      square(0),
      cross(0),
      circle(0),
      triangle(0),
      L1(0),
      R1(0),
      L2(0),
      R2(0),
      SE(0),
      ST(0),
      stick1(0),
      stick2(0),
      mode_type1(0),
      mode_type2(0),
      on_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_stick1_h;
      u_stick1_h.real = this->stick1_h;
      *(outbuffer + offset + 0) = (u_stick1_h.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stick1_h.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stick1_h.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stick1_h.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stick1_h);
      union {
        float real;
        uint32_t base;
      } u_stick1_v;
      u_stick1_v.real = this->stick1_v;
      *(outbuffer + offset + 0) = (u_stick1_v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stick1_v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stick1_v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stick1_v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stick1_v);
      union {
        float real;
        uint32_t base;
      } u_stick2_h;
      u_stick2_h.real = this->stick2_h;
      *(outbuffer + offset + 0) = (u_stick2_h.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stick2_h.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stick2_h.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stick2_h.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stick2_h);
      union {
        float real;
        uint32_t base;
      } u_stick2_v;
      u_stick2_v.real = this->stick2_v;
      *(outbuffer + offset + 0) = (u_stick2_v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stick2_v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stick2_v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stick2_v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stick2_v);
      union {
        int32_t real;
        uint32_t base;
      } u_intStick_h;
      u_intStick_h.real = this->intStick_h;
      *(outbuffer + offset + 0) = (u_intStick_h.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intStick_h.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intStick_h.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intStick_h.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intStick_h);
      union {
        int32_t real;
        uint32_t base;
      } u_intStick_v;
      u_intStick_v.real = this->intStick_v;
      *(outbuffer + offset + 0) = (u_intStick_v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intStick_v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intStick_v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intStick_v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intStick_v);
      *(outbuffer + offset + 0) = (this->square >> (8 * 0)) & 0xFF;
      offset += sizeof(this->square);
      *(outbuffer + offset + 0) = (this->cross >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cross);
      *(outbuffer + offset + 0) = (this->circle >> (8 * 0)) & 0xFF;
      offset += sizeof(this->circle);
      *(outbuffer + offset + 0) = (this->triangle >> (8 * 0)) & 0xFF;
      offset += sizeof(this->triangle);
      *(outbuffer + offset + 0) = (this->L1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->L1);
      *(outbuffer + offset + 0) = (this->R1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->R1);
      *(outbuffer + offset + 0) = (this->L2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->L2);
      *(outbuffer + offset + 0) = (this->R2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->R2);
      *(outbuffer + offset + 0) = (this->SE >> (8 * 0)) & 0xFF;
      offset += sizeof(this->SE);
      *(outbuffer + offset + 0) = (this->ST >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ST);
      *(outbuffer + offset + 0) = (this->stick1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stick1);
      *(outbuffer + offset + 0) = (this->stick2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stick2);
      *(outbuffer + offset + 0) = (this->mode_type1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mode_type1 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mode_type1 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mode_type1 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mode_type1);
      *(outbuffer + offset + 0) = (this->mode_type2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mode_type2 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mode_type2 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mode_type2 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mode_type2);
      *(outbuffer + offset + 0) = (this->on_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->on_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_stick1_h;
      u_stick1_h.base = 0;
      u_stick1_h.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stick1_h.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stick1_h.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stick1_h.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stick1_h = u_stick1_h.real;
      offset += sizeof(this->stick1_h);
      union {
        float real;
        uint32_t base;
      } u_stick1_v;
      u_stick1_v.base = 0;
      u_stick1_v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stick1_v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stick1_v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stick1_v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stick1_v = u_stick1_v.real;
      offset += sizeof(this->stick1_v);
      union {
        float real;
        uint32_t base;
      } u_stick2_h;
      u_stick2_h.base = 0;
      u_stick2_h.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stick2_h.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stick2_h.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stick2_h.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stick2_h = u_stick2_h.real;
      offset += sizeof(this->stick2_h);
      union {
        float real;
        uint32_t base;
      } u_stick2_v;
      u_stick2_v.base = 0;
      u_stick2_v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stick2_v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stick2_v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stick2_v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stick2_v = u_stick2_v.real;
      offset += sizeof(this->stick2_v);
      union {
        int32_t real;
        uint32_t base;
      } u_intStick_h;
      u_intStick_h.base = 0;
      u_intStick_h.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_intStick_h.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_intStick_h.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_intStick_h.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->intStick_h = u_intStick_h.real;
      offset += sizeof(this->intStick_h);
      union {
        int32_t real;
        uint32_t base;
      } u_intStick_v;
      u_intStick_v.base = 0;
      u_intStick_v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_intStick_v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_intStick_v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_intStick_v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->intStick_v = u_intStick_v.real;
      offset += sizeof(this->intStick_v);
      this->square =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->square);
      this->cross =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cross);
      this->circle =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->circle);
      this->triangle =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->triangle);
      this->L1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->L1);
      this->R1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->R1);
      this->L2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->L2);
      this->R2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->R2);
      this->SE =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->SE);
      this->ST =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ST);
      this->stick1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->stick1);
      this->stick2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->stick2);
      this->mode_type1 =  ((uint32_t) (*(inbuffer + offset)));
      this->mode_type1 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mode_type1 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mode_type1 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mode_type1);
      this->mode_type2 =  ((uint32_t) (*(inbuffer + offset)));
      this->mode_type2 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mode_type2 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mode_type2 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mode_type2);
      this->on_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->on_state);
     return offset;
    }

    const char * getType(){ return "joy_controller/joystick"; };
    const char * getMD5(){ return "f84adee99010c60edc4c5969569b08cb"; };

  };

}
#endif