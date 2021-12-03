#ifndef ALIENGO_ROS_CONTROL_H
#define ALIENGO_ROS_CONTROL_H

#include <memory>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "wolf_aliengo_interface/aliengo_robot_hw.hpp"


namespace aliengo2ros {

class AliengoROSControl {
public:

    std::string CLASS_NAME = "AliengoROSControl";

	AliengoROSControl();
	~AliengoROSControl();

	void init();
	void update(const ros::Time& time, const ros::Duration& period);

private:

	std::shared_ptr<ros::NodeHandle> node_handle_;
	
	// Aliengo Hardware interface
	std::shared_ptr<AliengoRobotHw> robot_hw_;

	// controller_manager provides the infrastructure to load, unload, start and stop controllers. (E.g. The DLS-supervisor)
	std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

}


#endif
