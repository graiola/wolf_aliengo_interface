#ifndef ALIENGO_ROBOT_HW_H
#define ALIENGO_ROBOT_HW_H

#include <wolf_hardware_interface/wolf_robot_hw.h>
#include <aliengo_hal/aliengo_hal.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace aliengo2ros
{

class AliengoRobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  AliengoRobotHw();
  virtual ~AliengoRobotHw();

  void init();
  void read();
  void write();

private:

  /** Map Aliengo internal joint indices to WoLF joints order */
  std::array<unsigned int, 12> aliengo_motor_idxs_
          {{
          aliengohal::FL_0, aliengohal::FL_1, aliengohal::FL_2, // LF
          aliengohal::RL_0, aliengohal::RL_1, aliengohal::RL_2, // LH
          aliengohal::FR_0, aliengohal::FR_1, aliengohal::FR_2, // RF
          aliengohal::RR_0, aliengohal::RR_1, aliengohal::RR_2, // RH
          }};

  /** Aliengo-HAL */
  aliengohal::LowLevelInterface aliengo_interface_;
  aliengohal::LowState aliengo_state_ = {0};
  aliengohal::LowCmd aliengo_lowcmd_ = {0};

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();

};

}

#endif
