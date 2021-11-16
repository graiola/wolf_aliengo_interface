#ifndef ALIENGO_ROBOT_HW_H
#define ALIENGO_ROBOT_HW_H

#include <memory>
#include <unordered_map>

#include <wb_hardware_interface/wb_robot_hw.h>
#include <aliengo_hal/aliengo_hal.h>

// Temporal logger
// #include <ctime>
#include <fstream>
#include <chrono>


namespace aliengo2ros
{


class AliengoRobotHw : public hardware_interface::RobotHW, public hardware_interface::WbRobotHwInterface
{
public:
	AliengoRobotHw();
	virtual ~AliengoRobotHw();

    virtual bool registerInterfaces();

	void init();
	void read();
	void write();

private:

	// Aliengo's parameters
    std::array<int, 12> aliengo_motor_idxs_ {{
        aliengohal::FL_0, aliengohal::FL_1, aliengohal::FL_2, // LF
        aliengohal::FR_0, aliengohal::FR_1, aliengohal::FR_2, // RF
        aliengohal::RL_0, aliengohal::RL_1, aliengohal::RL_2, // LH
        aliengohal::RR_0, aliengohal::RR_1, aliengohal::RR_2, // RH
	}};
	
	// Aliengo-HAL
	// std::shared_ptr<aliengohal::LowLevelInterface> aliengo_interface_;
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
