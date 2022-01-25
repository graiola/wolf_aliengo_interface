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

  /** @brief init */
	void init();

  /** @brief update */
	void update(const ros::Time& time, const ros::Duration& period);

private:

  /** @brief ROS node handle */
	std::shared_ptr<ros::NodeHandle> node_handle_;
	
  /** @brief Aliengo Hardware interface */
	std::shared_ptr<AliengoRobotHw> robot_hw_;

  /** @brief controller_manager provides the infrastructure to load, unload, start and stop controllers */
	std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

} // namespace


#endif
