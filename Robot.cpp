// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/smartdashboard.h>
#include "rev/ColorSensorV3.h"
#include <frc/util/color.h>
#include "rev/ColorMatch.h"

//Color Sensor Code
class Robot : public frc::TimedRobot {
  //Change the Port.
 static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  rev::ColorSensorV3 m_colorSensor{i2cPort};

  //Estimate the closest match.
  //rev::ColorMatch m_colorMatcher;

 public:

    void RobotPeriodic() {

      //Detect Color.
      frc::Color detectedColor = m_colorSensor.GetColor();
      double IR = m_colorSensor.GetIR();
      frc::SmartDashboard::PutNumber("Red", detectedColor.red);
      frc::SmartDashboard::PutNumber("Green", detectedColor.green);
      frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
      frc::SmartDashboard::PutNumber("IR", IR);

    //Roughly get the distance: Helpful to see when is the best time to detect the color.
     uint32_t proximity = m_colorSensor.GetProximity();
    frc::SmartDashboard::PutNumber("Proximity", proximity);

  }

 private:

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
