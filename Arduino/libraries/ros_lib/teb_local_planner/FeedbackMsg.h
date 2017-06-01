#ifndef _ROS_teb_local_planner_FeedbackMsg_h
#define _ROS_teb_local_planner_FeedbackMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "teb_local_planner/TrajectoryMsg.h"
#include "geometry_msgs/PolygonStamped.h"

namespace teb_local_planner
{

  class FeedbackMsg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t trajectories_length;
      typedef teb_local_planner::TrajectoryMsg _trajectories_type;
      _trajectories_type st_trajectories;
      _trajectories_type * trajectories;
      typedef uint16_t _selected_trajectory_idx_type;
      _selected_trajectory_idx_type selected_trajectory_idx;
      uint32_t obstacles_length;
      typedef geometry_msgs::PolygonStamped _obstacles_type;
      _obstacles_type st_obstacles;
      _obstacles_type * obstacles;

    FeedbackMsg():
      header(),
      trajectories_length(0), trajectories(NULL),
      selected_trajectory_idx(0),
      obstacles_length(0), obstacles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->trajectories_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectories_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectories_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectories_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectories_length);
      for( uint32_t i = 0; i < trajectories_length; i++){
      offset += this->trajectories[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->selected_trajectory_idx >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->selected_trajectory_idx >> (8 * 1)) & 0xFF;
      offset += sizeof(this->selected_trajectory_idx);
      *(outbuffer + offset + 0) = (this->obstacles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obstacles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obstacles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obstacles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obstacles_length);
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->obstacles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t trajectories_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      trajectories_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->trajectories_length);
      if(trajectories_lengthT > trajectories_length)
        this->trajectories = (teb_local_planner::TrajectoryMsg*)realloc(this->trajectories, trajectories_lengthT * sizeof(teb_local_planner::TrajectoryMsg));
      trajectories_length = trajectories_lengthT;
      for( uint32_t i = 0; i < trajectories_length; i++){
      offset += this->st_trajectories.deserialize(inbuffer + offset);
        memcpy( &(this->trajectories[i]), &(this->st_trajectories), sizeof(teb_local_planner::TrajectoryMsg));
      }
      this->selected_trajectory_idx =  ((uint16_t) (*(inbuffer + offset)));
      this->selected_trajectory_idx |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->selected_trajectory_idx);
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
     return offset;
    }

    const char * getType(){ return "teb_local_planner/FeedbackMsg"; };
    const char * getMD5(){ return "f0ca746a67d34e8b00ad2e5fcd06d909"; };

  };

}
#endif