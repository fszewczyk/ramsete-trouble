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
    double xLeftAxis = Robot.oi.getLX(); // should equal variable from pad
    double yLeftAxis = Robot.oi.getLY(); // should equal variable from pad
  
    Robot.driveTrain.setMotors(yLeftAxis + xLeftAxis * Constants.DRIVE_CONTROL, yLeftAxis - xLeftAxis * Constants.DRIVE_CONTROL);
    SmartDashboard.putNumber("twojstary", xLeftAxis);
    if (!Robot.oi.getYButton()){
      finish();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stopDrive();
  }
  private void finish(){
    isFinished=true;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
