#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h> // Provides methods to send data to the Driver Station.

bool leioff = true;   // Boolean to track the LIDAR enable state (true = off, false = on).

// Called once when the robot is initialized.
void Robot::RobotInit() {}

// Called periodically in all modes to handle regular updates.
void Robot::RobotPeriodic() 
{
  scandata = m_Lidar.GetData();   // Fetch the latest data from the LIDAR sensor.
}

// Called once at the start of Autonomous mode.
void Robot::AutonomousInit() {
  if (!leioff) {                 // Check if LIDAR is off.
    StartLidar();                // Enable the LIDAR.
    leioff = true;               // Update the state to indicate LIDAR is active.
  }
  m_timer.Reset();               // Reset the autonomous timer to 0.
  m_timer.Start();               // Start the autonomous timer.
}

// Called periodically during Autonomous mode.
void Robot::AutonomousPeriodic()
{     
  float mlidar_distance = MeanFiltering(80, 100); // Calculate mean distance for LIDAR data in the range 80-100.

  // Send the calculated distance to the Driver Station dashboard for monitoring.
  frc::SmartDashboard::PutNumber("mlidar_distance", mlidar_distance); 
}

// Called once at the start of Teleoperation mode (manual control).
void Robot::TeleopInit() {}

// Called periodically during Teleoperation mode.
void Robot::TeleopPeriodic() {}

// Called once when the robot enters Disabled mode.
void Robot::DisabledInit() {
  if (leioff) {                // Check if LIDAR is active.
    StopLidar();               // Stop the LIDAR to save power or resources.
    leioff = false;            // Update the state to indicate LIDAR is disabled.
  }
}

// Called periodically during Disabled mode.
void Robot::DisabledPeriodic() {}

// Called periodically during Test mode.
void Robot::TestPeriodic() {}

// Entry point for the robot program.
#ifndef RUNNING_FRC_TESTS  
int main() { 
  return frc::StartRobot<Robot>(); // Start the robot application.
}
#endif

// Custom method to start the LIDAR sensor.
void Robot::StartLidar() {
  m_Lidar.Start();              // Calls the Start function from the LIDAR library.
}

// Custom method to stop the LIDAR sensor.
void Robot::StopLidar() {
  m_Lidar.Stop();               // Calls the Stop function from the LIDAR library.
}

// Function to compute the mean of LIDAR distances within a specified range.
float Robot::MeanFiltering(int Meanin, int Meanend) {    
  // Validate that the input range is within valid LIDAR index bounds.
  if (Meanin > indexMin && Meanend < indexMax) {
    double D_value = 0;        // Sum of all valid distances.
    int D_times = 0;           // Total number of valid readings.
    int Drange_T = 0;          // Count of values considered "out of range."
    int Drange_F = 0;          // Count of values within the valid range.
    double DvauleF = 0;        // Sum of "in-range" values.

    // Iterate through the specified range of LIDAR data.
    for (int i = Meanin; i < Meanend; i++) {
      // Check if the current distance is within the min and max distance thresholds.
      if (scandata.distance[i] > mlidarminDis && scandata.distance[i] < mlidarmaxDis) {
        if (scandata.distance[i] > mlidarbeyond) { // Check if the distance exceeds the "beyond" threshold.
          Drange_T++;          // Increment the out-of-range counter.
        } else {
          Drange_F++;          // Increment the in-range counter.
          DvauleF += scandata.distance[i]; // Accumulate in-range values.
        }
        D_value += scandata.distance[i];   // Add to the total sum of valid distances.
        D_times++;                         // Increment the count of valid readings.
      }
    }

    // Calculate the mean of in-range values and all valid distances.
    double DdataF = (Drange_F > 0) ? DvauleF / Drange_F : 0; // Mean of in-range values.
    double Ddatam = (D_times > 0) ? D_value / D_times : 0;   // Mean of all valid values.

    // Decide which mean to return based on out-of-range count.
    if (Drange_T > (D_times / 2)) {  // If more than half are out of range...
      return Ddatam;                // Return the mean of all valid values.
    } else {
      return DdataF;                // Otherwise, return the mean of in-range values.
    }
  } else {
    return -1;                      // Return -1 if the input range is invalid.
  }
}
