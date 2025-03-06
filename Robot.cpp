// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <Robot.h>
//#include <Drivetrain.h>
#include <frc/Timer.h>
#include <fmt/core.h>

//#include <frc/smartdashboard/SmartDashboard.h>
#include <cstdio>


void Robot::RobotInit() {
  
  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

//try
//{
   //ahrs = new AHRS(SPI::Port::kMXP);
    //ahrs = new AHRS(SerialPort::Port::kUSB1);
//}
  //catch(const std::exception& e)
//{
    //std::cerr << e.what() << '\n';
    
     // The RestoreFactoryDefaults method can be used to reset the configuration parameters
     // in the SPARK MAX to their factory default state. If no argument is passed, these
     // parameters will not persist between power cycles
     //
  //m_leftLeadMotor.RestoreFactoryDefaults();
  //m_rightLeadMotor.RestoreFactoryDefaults();
  //m_leftFollowMotor.RestoreFactoryDefaults();
  //m_rightFollowMotor.RestoreFactoryDefaults();
 
 //****wpi::PortForwarder::GetInstance().Add(8888, "wpilibpi.local", 80);

//need to fix inversion
  //sparkMax6.SetInverted(true);
  //m_rearRight.SetInverted(true);
 // m_rearLeft.SetInverted(false);
  //m_frontLeft.SetInverted(false);
   
  
 m_robotDrive.DriveCartesian (-m_driverController.GetLeftX(), m_driverController.GetLeftY(),
                                 m_driverController.GetRightTriggerAxis() + -m_driverController.GetLeftTriggerAxis());

//SparkMaxConfig config();

 // config
    //.Inverted(true)
    //.SetIdleMode(SparkMaxConfig::IdleMode::kBrake);



    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
  
// Locations of the wheels relative to the robot center.
frc::Translation2d m_frontLeftLocation{0.3937_m, 0.3937_m};
frc::Translation2d m_frontRightLocation{0.3937_m, -0.3937_m};
frc::Translation2d m_rearLeftLocation{-0.3937_m, 0.3937_m};
frc::Translation2d m_rearRightLocation{-0.3937_m, -0.3937_m};

// Creating my kinematics object using the wheel locations.
//frc::MecanumDriveKinematics m_kinematics{
  //m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation};

////uncomment the next 2 lines for Boom control
//m_Boom.Restore FactoryDefaults();

//m_pidController.SetFeedbackDevice(m_encoder);

///// set PID coefficients
////uncomment the next 6 lines for Boom control
//m_pidController.SetP(kP);
//m_pidController.SetI(kI);
//m_pidController.SetD(kD);
//m_pidController.SetIZone(kIz);
//m_pidController.SetFF(kFF);
//m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  
}

//
 //This function is called every 20 ms, no matter the mode. Use
 //this for items like diagnostics that you want ran during disabled,
 // autonomous, teleoperated and test.
 //
 // <p> This runs after the mode specific periodic functions, but before
 // LiveWindow and SmartDashboard integrated updating.
 //

void Robot::RobotPeriodic()    {
 
  //frc::SmartDashboard::PutNumber("ia: ", ia);
  //frc::SmartDashboard::PutNumber("it: ", it);




}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */


void Robot::AutonomousInit() {
    // Custom Auto goes here
    //fix inversion
  //m_frontRight.SetInverted(true);
  //m_rearRight.SetInverted(true);
  //m_rearLeft.SetInverted(false);
  //m_frontLeft.SetInverted(false);

    m_timer.Reset();
    m_timer.Start();
    ia = 0;
} 
  //else {
    // Default Auto goes here
  
//-------------auto----------------------------------------------------------------------------------

void Robot::AutonomousPeriodic() 
  //if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
   {
     if (m_timer.Get() < 3_s) {
      // Drive forwards half speed, make sure to turn input squaring off
        m_robotDrive.DriveCartesianIK (0.0, 0.3, false);
    } else {
      // Stop robot
        m_robotDrive.DriveCartesianIK (0.0, 0.0, false);
  }
  ia++;
}

//----------------auto----------------------------------------------------------------------------------

void Robot::TeleopInit() {
    it = 0;
}

void Robot::TeleopPeriodic() {

  
 //Get

  it++;

  left_y = m_driverController.frc::XboxController::GetLeftX(),
  right_y = m_driverController.frc::XboxController::GetRightX();

  left_x = m_driverController.frc::XboxController::GetLeftY(),
  right_x = m_driverController.frc::XboxController::GetRightY();

  leftbumperbutton = m_driverController.frc::XboxController::GetLeftBumperButton();
  rightbumperbutton = m_driverController.frc::XboxController::GetRightBumperButton();

  lefttriggeraxis = m_driverController.frc::XboxController::GetLeftTriggerAxis();
  righttriggeraxis = m_driverController.frc::XboxController::GetRightTriggerAxis();

  bA = m_driverController.GetAButton();
  bB = m_driverController.GetBButton();
  bX = m_driverController.GetXButton();
  bY = m_driverController.GetYButton();


//Drive Cartesian
  m_robotDrive.DriveCartesianIK (-m_driverController.GetLeftY(), m_driverController.GetLeftX(),
                               m_driverController.GetRightTriggerAxis() + -m_driverController.GetLeftTriggerAxis());

m_boom.Set(right_y);
////read PID coefficients on SmartDashboard...uncomment next 8 lines for Boom control
//double p = frc::SmartDashboard::GetNumber("P Gain", 0);
//double i = frc::SmartDashboard::GetNumber("I Gain", 0);
//double d = frc::SmartDashboard::GetNumber("D Gain", 0);
//double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
//double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
//double max = frc::SmartDashboard::GetNumber("Max Output", 0);
//double min = frc::SmartDashboard::GetNumber("Min Output", 0);
//double rotations = frc::SmartDashboard::GetNumber("Set Rotation", 0);

////if PID coefficients on SmartDashboard have changed, write new values to controller...uncomment 
//next 6 lines for Boom control
//if((p !=kP)) { m_pidController.SetP(p); kP = p; }
//if((i !=kI)) { m_pidController.SetI(i); kI = i; }
//if((d !=kD)) { m_pidController.SetD(d); kD = d; }
//if((iz !=kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
//if((ff !=kFF)) { m_pidController.SetFF(ff); kFF = ff; }
//if((max !=kMaxOutput) || (min !=kMinOutpit)) { m_pidController.SetOutputRange(min, max); 
//   kMinOutput = min; kMaxOutput = max; 
//   }

////uncomment the next 3 lines for Boom control
//m_pidController.SetReference(rotations, rev::CanSparkMax::ControlType::kPosition);

//frc::SmartDashboard::PutNumber("SetPoint", rotations);
//frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder.GetPosition());
}
  
 


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
