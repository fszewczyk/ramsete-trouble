/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

import frc.robot.drivetrain.TeleopDrive;
import frc.robot.drivetrain.TurnToAngle;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  public static DriveTrain driveTrain;
  //public static TeleopDrive teleopDrive;
  //private CameraServer cam;

  //private RobotContainer m_robotContainer;
  public Robot () {
    driveTrain = new DriveTrain();
  }
  @Override
  public void robotInit() {  
    //m_robotContainer = new RobotContainer();
    cam.getInstance();
    cam.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();    
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().schedule(new TeleopDrive());
  }

  @Override
  public void teleopPeriodic() {
    //driveTrain.getGyroAngle();
    //if(oi.getYButton()) CommandScheduler.getInstance().schedule(new TurnToAngle(30));
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  
  }
}
