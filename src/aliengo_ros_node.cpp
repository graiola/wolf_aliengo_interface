#include <ros/ros.h>
#include "wb_aliengo_interface/aliengo_ros_control.hpp"

static aliengo2ros::AliengoROSControl _ros_control;

int main(int argc, char**argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "aliengo_ros_control_node");

    // Load the task_period from the param server
    ros::NodeHandle nh;
    double period;
    if(!nh.getParam("/task_period",period))
    {
        ROS_ERROR("Task period not available!");
        return 1;
    }

    // Starting the ros control
    _ros_control.init();

    // Start asynchronous ROS spinner
    ros::AsyncSpinner spinner(1); // Argument is the number of threads to use (0 means as many threads as processors)
    spinner.start();

    // Update duration of the loop
    ros::Duration loop_period(period);

    // Run the servo loop
    while (ros::ok())
    {
        // Updating the ros controller
        _ros_control.update(ros::Time::now(), loop_period);

        // Keep the ros magic alive
        ros::spinOnce();

        // Sleep to keep the loop at specified period
        loop_period.sleep();
    }

    spinner.stop();

    return 0;
}
