/*
 * Copyright (C) 2022 multidof
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include "wolf_aliengo_interface/aliengo_ros_control.hpp"


namespace aliengo2ros {


AliengoROSControl::AliengoROSControl()
{

}


AliengoROSControl::~AliengoROSControl()
{

}


void AliengoROSControl::init()
{
    ROS_INFO_NAMED(CLASS_NAME,"Initializing AliengoROSControl");

	// Reset RobotHW
	robot_hw_.reset(new aliengo2ros::AliengoRobotHw);

	// Reseting the namespace of the node handle
	node_handle_.reset(new ros::NodeHandle(robot_hw_->getRobotName()));

	// Initializing the hardware interface
	robot_hw_->init();

	// Reseting the controller manager
	controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_.get(), *node_handle_.get()));
}


void AliengoROSControl::update(const ros::Time& time, const ros::Duration& period)
{
	// Reading sensor information
	robot_hw_->read();

	// Updating the controller manager
	controller_manager_->update(time, period);

	// Writing to the actuator
	robot_hw_->write();
}


}
