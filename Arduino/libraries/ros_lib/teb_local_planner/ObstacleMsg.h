#ifndef _ROS_teb_local_planner_ObstacleMsg_h
#define _ROS_teb_local_planner_ObstacleMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TwistWithCovariance.h"

namespace teb_local_planner
{

  class ObstacleMsg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t obstacles_length;
      typedef geometry_msgs::PolygonStamped _obstacles_type;
      _obstacles_type st_obstacles;
      _obstacles_type * obstacles;
      uint32_t ids_length;
      typedef uint32_t _ids_type;
      _ids_type st_ids;
      _ids_type * ids;
      uint32_t orientations_length;
      typedef geometry_msgs::QuaternionStamped _orientations_type;
      _orientations_type st_orientations;
      _orientations_type * orientations;
      uint32_t velocities_length;
      typedef geometry_msgs::TwistWithCovariance _velocities_type;
      _velocities_type st_velocities;
      _velocities_type * velocities;

    ObstacleMsg():
      header(),
      obstacles_length(0), obstacles(NULL),
      ids_length(0), ids(NULL),
      orientations_length(0), orientations(NULL),
      velocities_length(0), velocities(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->obstacles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obstacles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obstacles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obstacles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obstacles_length);
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->obstacles[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ids_length);
      for( uint32_t i = 0; i < ids_length; i++){
      *(outbuffer + offset + 0) = (this->ids[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ids[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ids[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ids[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ids[i]);
      }
      *(outbuffer + offset + 0) = (this->orientations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->orientations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->orientations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->orientations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientations_length);
      for( uint32_t i = 0; i < orientations_length; i++){
      offset += this->orientations[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->velocities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities_length);
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += this->velocities[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t obstacles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->obstacles_length);
      if(obstacles_lengthT > obstacles_length)
        this->obstacles = (geometry_msgs::PolygonStamped*)realloc(this->obstacles, obstacles_lengthT * sizeof(geometry_msgs::PolygonStamped));
      obstacles_length = obstacles_lengthT;
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->st_obstacles.deserialize(inbuffer + offset);
        memcpy( &(this->obstacles[i]), &(this->st_obstacles), sizeof(geometry_msgs::PolygonStamped));
      }
      uint32_t ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ids_length);
      if(ids_lengthT > ids_length)
        this->ids = (uint32_t*)realloc(this->ids, ids_lengthT * sizeof(uint32_t));
      ids_length = ids_lengthT;
      for( uint32_t i = 0; i < ids_length; i++){
      this->st_ids =  ((uint32_t) (*(inbuffer + offset)));
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_ids);
        memcpy( &(this->ids[i]), &(this->st_ids), sizeof(uint32_t));
      }
      uint32_t orientations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      orientations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      orientations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      orientations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->orientations_length);
      if(orientations_lengthT > orientations_length)
        this->orientations = (geometry_msgs::QuaternionStamped*)realloc(this->orientations, orientations_lengthT * sizeof(geometry_msgs::QuaternionStamped));
      orientations_length = orientations_lengthT;
      for( uint32_t i = 0; i < orientations_length; i++){
      offset += this->st_orientations.deserialize(inbuffer + offset);
        memcpy( &(this->orientations[i]), &(this->st_orientations), sizeof(geometry_msgs::QuaternionStamped));
      }
      uint32_t velocities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocities_length);
      if(velocities_lengthT > velocities_length)
        this->velocities = (geometry_msgs::TwistWithCovariance*)realloc(this->velocities, velocities_lengthT * sizeof(geometry_msgs::TwistWithCovariance));
      velocities_length = velocities_lengthT;
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += this->st_velocities.deserialize(inbuffer + offset);
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(geometry_msgs::TwistWithCovariance));
      }
     return offset;
    }

    const char * getType(){ return "teb_local_planner/ObstacleMsg"; };
    const char * getMD5(){ return "a537b0b7fce70da7b78c2df042f56aa2"; };

  };

}
#endif