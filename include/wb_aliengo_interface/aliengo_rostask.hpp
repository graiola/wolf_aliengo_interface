#ifndef ALIENGO_ROSTASK_H
#define ALIENGO_ROSTASK_H

#include <ros/ros.h>
#include <realtime_tools/realtime_clock.h>



// 'Dummy' output codes / annotations of functions
#define SUCCESS 1
#define FAILURE 0


// Both RT and non-RT source-files with 'main' function define the values of these variables
extern int servo_base_rate;
extern int task_servo_ratio;
extern int rostask_servo_rate;

// Most important functions provided by this module	
void init_rostask(void);
int run_rostask(bool isrealtime);

#endif
