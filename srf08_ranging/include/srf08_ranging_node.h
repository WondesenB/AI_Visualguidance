

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


  
 int fd; // File descrition
 char * fileName = "/dev/i2c-0";   // Name of the port we will be using
 int address = 0x70;               // Address of the SRF08 shifted right 1 bit
 unsigned char buf[10];           // Buffer for data being read/ written on the i2c bus

int srf08_range(void);