// SRF08 
// 
// Performs a ranging on the SRF08 then reads the data back in CM
// and prints to the screen.

// include
#include "ros/ros.h"
#include "srf08_ranging_node.h"
#include <srf08_ranging/obstacle_distance.h>

#include <cstddef>
#include <math.h>
#include <vector>


int distance_up;
int distance_right;
int distance_left;
int main(int argc, char **argv)
{


  ros::init(argc, argv, "srf08_ranging_node");

  ros::NodeHandle n;
  
 ros::Publisher obstacle_distance_pub = n.advertise<srf08_ranging::obstacle_distance>("/obstacle_distance/info",10);
 srf08_ranging::obstacle_distance  obs_distance;
 

 ros::Rate rate(70.0);  
 ros::Time last_request = ros::Time::now();

while(ros::ok() )
{
 distance_up = srf08_range(address_up);
 distance_right = srf08_range(address_right);
 distance_left = srf08_range(address_left);
 ROS_INFO("obstacle up = %d , obstacle right = %d , obstacle left = %d", distance_up, distance_right, distance_left);
 
 obs_distance.header.stamp = ros::Time::now();
 obs_distance.header.frame_id ="base_drone";
 obs_distance.up = distance_up;
 obs_distance.right = distance_right;
 obs_distance.left = distance_left;
 obstacle_distance_pub.publish(obs_distance);

 ros::spinOnce();
 rate.sleep();

}


  return 0;
}



int srf08_range(int address)
{
//printf("**** SRF08 test program ****\n");

// Open port for reading and writing

  if ((fd = open(fileName, O_RDWR)) < 0) { 
    printf("Failed to open i2c port\n");
    exit(1);
  }

// Set the port options and set the address of the device we wish to speak to
  if (ioctl(fd, I2C_SLAVE, address) < 0) { 
    printf("Unable to get bus access to talk to slave\n");
    exit(1);
  }

  buf[0] = 0;    // Commands for performing a ranging on the SRF08
  buf[1] = 0x51;

// Write commands to the i2c port
  if ((write(fd, buf, 2)) != 2) { 
    printf("Error writing to i2c slave\n");
    exit(1);
  }

  usleep(100000); //usleep(650000); // this sleep waits for the ping to come back

  buf[0] = 0; // This is the register we wish to read from

 // Send register to read from

  if ((write(fd, buf, 1)) != 1) { 
    printf("Error writing to i2c slave\n");
    exit(1);
  }
  // Read back data into buf[]

    if (read(fd, buf, 4) != 4) 
    { 
      printf("Unable to read from slave\n");
      exit(1);
    }
   else 
    {
      unsigned char highByte = buf[2];
      unsigned char lowByte = buf[3];
      unsigned int result = (highByte << 8) + lowByte; // Calculate range
      // printf("Software v: %u \n", buf[0]);
      // printf("Light: %u \n", buf[1]);
      //printf("Range was: %d \n", result);
      return result;
    }
 return 0;
 
}


