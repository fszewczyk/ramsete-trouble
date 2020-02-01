/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TeleopDrive extends CommandBase {
  boolean isFinished=false;
  public TeleopDrive() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Add input from pad.
    double yLeftAxis = Robot.robotContainer.getLY(); //Robot.oi.getRX();
    double xRightAxis = Robot.robotContainer.getRX(); // Robot.oi.getLY();
    
    if (Math.abs(xRightAxis) < Constants.PAD_DEADBOUND) xRightAxis=0;
    if (Math.abs(yLeftAxis) < Constants.PAD_DEADBOUND) yLeftAxis=0;

    double left_speed = yLeftAxis + xRightAxis;
    double right_speed = yLeftAxis - xRightAxis;

    left_speed = left_speed * left_speed * left_speed / Math.abs(left_speed);
    right_speed = right_speed * right_speed * right_speed / Math.abs(right_speed);

    Robot.driveTrain.setMotors(left_speed * Constants.DRIVE_CONTROL, right_speed * Constants.DRIVE_CONTROL);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
