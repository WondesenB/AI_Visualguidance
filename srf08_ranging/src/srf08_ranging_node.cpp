// SRF08 
// 
// Performs a ranging on the SRF08 then reads the data back in CM
// and prints to the screen.

// include
#include "ros/ros.h"
#include "srf08_ranging_node.h"


#include <cstddef>
#include <math.h>
#include <vector>


int distance;
int main(int argc, char **argv)
{


  ros::init(argc, argv, "srf08_ranging_node");

  ros::NodeHandle n;
  

 ros::Rate rate(70.0);  
 ros::Time last_request = ros::Time::now();

while(ros::ok() )
{
 distance = srf08_range();
 ROS_INFO("sr08 range = %d ", distance);

 ros::spinOnce();
 rate.sleep();

}


  return 0;
}



int srf08_range(void)
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

  usleep(750000); // this sleep waits for the ping to come back

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


