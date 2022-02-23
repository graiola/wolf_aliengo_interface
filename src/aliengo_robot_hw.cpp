/*
 * Copyright (C) 2022 Gennaro Raiola
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

#include "wolf_aliengo_interface/aliengo_robot_hw.hpp"

namespace aliengo2ros
{

using namespace hardware_interface;

int64_t utime_now() {

    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    if (timeofday.tv_sec < 0 || timeofday.tv_sec > UINT_MAX)
        throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
    uint32_t sec	= timeofday.tv_sec;
    uint32_t nsec = timeofday.tv_usec * 1000;

    return (int64_t) (((uint64_t)sec)*1000000 + ((uint64_t)nsec) / 1000);
}

AliengoRobotHw::AliengoRobotHw()
{
    robot_name_ = "aliengo";
}

AliengoRobotHw::~AliengoRobotHw()
{

}

void AliengoRobotHw::init()
{
    // Hardware interfaces: Joints
    auto joint_names = loadJointNamesFromSRDF();
    if(joint_names.size()>0)
    {
      WolfRobotHwInterface::initializeJointsInterface(joint_names);
      registerInterface(&joint_state_interface_);
      registerInterface(&joint_effort_interface_);
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register joint interface.");
      return;
    }

    // Hardware interfaces: IMU
    auto imu_name = loadImuLinkNameFromSRDF();
    if(!imu_name.empty())
    {
      WolfRobotHwInterface::initializeImuInterface(imu_name);
      registerInterface(&imu_sensor_interface_);
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register imu interface.");
      return;
    }

    aliengo_interface_.InitCmdData(aliengo_lowcmd_);
    startup_routine();
}

void AliengoRobotHw::read()
{
    // Get robot data
    aliengo_state_ = aliengo_interface_.ReceiveObservation();

    // ------
    // Joints
    // ------
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
    {
        joint_position_[jj] = static_cast<double>(aliengo_state_.motorState[aliengo_motor_idxs_[jj]].q)     ;
        joint_velocity_[jj] = static_cast<double>(aliengo_state_.motorState[aliengo_motor_idxs_[jj]].dq)    ;
        joint_effort_[jj]   = static_cast<double>(aliengo_state_.motorState[aliengo_motor_idxs_[jj]].tauEst);
    }

    // ---
    // IMU
    // ---
    imu_orientation_[0] = static_cast<double>(aliengo_state_.imu.quaternion[0]);  // w
    imu_orientation_[1] = static_cast<double>(aliengo_state_.imu.quaternion[1]);  // x
    imu_orientation_[2] = static_cast<double>(aliengo_state_.imu.quaternion[2]);  // y
    imu_orientation_[3] = static_cast<double>(aliengo_state_.imu.quaternion[3]);  // z

    imu_ang_vel_[0] = static_cast<double>(aliengo_state_.imu.gyroscope[0]);
    imu_ang_vel_[1] = static_cast<double>(aliengo_state_.imu.gyroscope[1]);
    imu_ang_vel_[2] = static_cast<double>(aliengo_state_.imu.gyroscope[2]);

    imu_lin_acc_[0] = static_cast<double>(aliengo_state_.imu.accelerometer[0]);
    imu_lin_acc_[1] = static_cast<double>(aliengo_state_.imu.accelerometer[1]);
    imu_lin_acc_[2] = static_cast<double>(aliengo_state_.imu.accelerometer[2]);
}

void AliengoRobotHw::write()
{
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
      aliengo_lowcmd_.motorCmd[aliengo_motor_idxs_[jj]].tau = static_cast<float>(joint_effort_command_[jj]  );

    aliengo_interface_.SendLowCmd(aliengo_lowcmd_);
}

void AliengoRobotHw::send_zero_command()
{
    std::array<float, 60> zero_command = {0};
    // aliengo_interface_->SendCommand(zero_command);
    aliengo_interface_.SendCommand(zero_command);
}

void AliengoRobotHw::startup_routine()
{
    send_zero_command();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

} // namespace
