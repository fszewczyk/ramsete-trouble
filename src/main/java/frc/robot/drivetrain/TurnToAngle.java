/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurnToAngle extends CommandBase {

  private final PIDController pidController = new PIDController(Constants.DRIVE_BASE_Kp, Constants.DRIVE_BASE_Ki, Constants.DRIVE_BASE_Kd);
  private boolean isFinished = false;
  private double startGyroAngle;  
  private double actualGyroAngle;

  public TurnToAngle(double angle) {

    addRequirements(Robot.driveTrain);

    pidController.setIntegratorRange(Constants.DRIVE_BASE_MIN_I, Constants.DRIVE_BASE_MAX_I);
    pidController.setTolerance(Constants.DRIVE_BASE_PID_TOLERANCE);
    pidController.reset();
    pidController.setSetpoint(angle);
  }

  @Override
  public void initialize() {
    
    startGyroAngle = Robot.driveTrain.getGyroAngle(); // should read angle from gyro
  }

  @Override
  public void execute() {
    actualGyroAngle = 0; // should read actual angle from gyro
    double speed = pidController.calculate(actualGyroAngle - startGyroAngle);
    Robot.driveTrain.setMotors(speed, -speed);
    isFinished = pidController.atSetpoint();
    }

  @Override
  public void end(final boolean interrupted) {
    Robot.driveTrain.stopDrive();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}