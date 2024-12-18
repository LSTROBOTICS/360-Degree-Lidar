#pragma once

#include <frc/TimedRobot.h>         // Includes the base class for the robot structure.
#include "studica/Lidar.h"          // Includes the library for Lidar sensor integration.
#include "studica/TitanQuad.h"      // Includes the library for TitanQuad motor control (if used elsewhere).
#include "AHRS.h"                   // Includes the library for the AHRS (Attitude and Heading Reference System), used for navigation.

class Robot : public frc::TimedRobot {  // Inherits from frc::TimedRobot, providing a framework for periodic robot functions.
 public:
  // Standard robot lifecycle methods
  void RobotInit() override;          // Called when the robot is first initialized.
  void RobotPeriodic() override;      // Called periodically in all modes (useful for sensor updates).
  void DisabledInit() override;       // Called once when the robot enters Disabled mode.
  void DisabledPeriodic() override;   // Called periodically during Disabled mode.
  void AutonomousInit() override;     // Called once at the start of Autonomous mode.
  void AutonomousPeriodic() override; // Called periodically during Autonomous mode.
  void TeleopInit() override;         // Called once at the start of Teleoperation mode.
  void TeleopPeriodic() override;     // Called periodically during Teleoperation mode.
  void TestPeriodic() override;       // Called periodically during Test mode.

  // Custom methods for Lidar functionality
  void StartLidar();                  // Starts the Lidar sensor.
  void StopLidar();                   // Stops the Lidar sensor.
  float MeanFiltering(int start, int end); // Computes the mean value of Lidar data within a range.

 private:
  // Sensor and hardware objects
  AHRS navx{frc::SPI::Port::kMXP};  // Creates an AHRS (gyro/IMU) object, connected via the SPI MXP port on the RoboRIO.
  studica::Lidar m_Lidar{studica::Lidar::Port::kUSB1}; // Creates a Lidar object, connected to the top USB2.0 port.
  studica::Lidar::ScanData scandata = {{0.0}, {0.0}};  // Struct to store Lidar scan data, initialized with zeros.

  frc::Timer m_timer;  // Timer object for tracking time during robot operation.

  // Constants for Lidar index and distance filtering
  static constexpr int indexMax = 360;       // Maximum Lidar angle index (360 degrees).
  static constexpr int indexMin = 0;         // Minimum Lidar angle index (0 degrees).

  static constexpr int mlidarminDis = 100;   // Minimum valid distance for Lidar readings in mm.
  static constexpr int mlidarmaxDis = 5500;  // Maximum valid distance for Lidar readings in mm.
  static constexpr int mlidarbeyond = 1000;  // Threshold for marking a distance as beyond usable range.

};
