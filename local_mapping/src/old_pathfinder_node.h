#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/String.h>

#include "local_mapping/detected_object.h"

//#include <zed_data_subscribe/detected_object.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <math.h>
#include <iostream> 
#include <string>
#include <sstream>
#include <fstream>
#include <bits/stdc++.h>		//word extraction from string
#include <vector>
#include <array>

#include <sensor_msgs/Imu.h>

using namespace std; 
struct objects_
{		
	string obj_type;
	float  y_min;
	float  y_max;
	float  z_min;
	float  x_cnt;
	float  y_cnt;
	float  z_cnt;
	//objects_() : obj_type("none"), y_min(0), y_max(0), z_min(0), x_cnt(0), y_cnt(0), z_cnt(0){} 
	//float  distance;				
};

//objects_ temp = objects_();
/*
const struct objects_ tempo_val=
{
	.obj_type = "none", 
	.y_min = 0.0, 
	.y_max = 0.0, 
	.z_min = 0.0, 
	.x_cnt = 0.0, 
	.y_cnt = 0.0, 
	.z_cnt = 0.0
};
*/
//string arr[] = {"none", "0", "0", "0", "0", "0", "0"};

struct objects
{
	//objects() : obj() {"none";0.0;0.0;0.0;0.0;0.0;0.0;};
		//objects(const vector<objects_> obj={}):obj(obj){};
	vector<objects_> obj;
	
	//objects();
	//~objects(){};
}ob;

class PathFinder
{
public:
       
	vector<string> visited_object;
	vector<string> object_list = {"window", "wall", "pole", "pipe", "tree", "net", "land"};

	//publishers
	//ros::Publisher local_setpos_pub;

	//PathFinder();
	//~PathFinder(){};


private:

	
	//ros::Subscriber imu_sub;
	//ros::Subscriber BoundBox_sub;

	//void state_cb(const mavros_msgs::State::ConstPtr& msg);
	//void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	//void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
	//void BoundBox_info_cb(const old_pathfinder::detected_object::ConstPtr& msg);	

};
	


#endif