/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  public static NetworkTableInstance inst;
  public static NetworkTable table;
  public static NetworkTableEntry angleEntry;
  // public static TeleopDrive teleopDrive;
  private UsbCamera cam;
  private UsbCamera cam2;
  private CameraServer camServer;

  public static RobotContainer robotContainer;

  public Robot () {

    inst = NetworkTableInstance.getDefault();  
    table = inst.getTable("Dashboard");
    angleEntry = table.getEntry("Angle");
    driveTrain = new DriveTrain();
    robotContainer = new RobotContainer();
  }
  @Override
  public void robotInit() {  
    cam = CameraServer.getInstance().startAutomaticCapture();
    cam2 = CameraServer.getInstance().startAutomaticCapture();
    camServer = CameraServer.getInstance();
    cam.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 180, 30);
    cam2.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 180, 30);
   // cam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
   // cam2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
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
  public void autonomousInit() 
  {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {
  
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, new TeleopDrive());
    }

  @Override
  public void teleopPeriodic() {
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
