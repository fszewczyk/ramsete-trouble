/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TurnToAngle extends CommandBase {

  private final PIDController pidController;
  private double desireAngle;
  private double actualAngle;
  private boolean isFinished = false;
  private double prevAngle;
 
  public TurnToAngle() {
    
    pidController = new PIDController(Constants.DRIVE_BASE_Kp, Constants.DRIVE_BASE_Ki, Constants.DRIVE_BASE_Kd);
    addRequirements(Robot.driveTrain);
    pidController.setIntegratorRange(Constants.DRIVE_BASE_MIN_I, Constants.DRIVE_BASE_MAX_I);
    pidController.setTolerance(Constants.DRIVE_BASE_PID_TOLERANCE);
    pidController.reset();
  }

  @Override
  public void initialize() {
    Robot.driveTrain.resetGyro();
    desireAngle = Robot.angleEntry.getDouble(0.0); //read angle from jetson
    SmartDashboard.putNumber("visionAngle", desireAngle);
    pidController.setSetpoint(desireAngle);

  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Działam chyba", 1);

    prevAngle = actualAngle;
    actualAngle = Robot.angleEntry.getDouble(prevAngle); // should read angle from jetson
    SmartDashboard.putNumber("visionAngle", actualAngle);

    double speed = pidController.calculate(actualAngle);
    SmartDashboard.putNumber("angle speed", speed);

    Robot.driveTrain.setMotors(-speed, speed);
    isFinished = pidController.atSetpoint();
    }

  @Override
  public void end(final boolean interrupted) {
    Robot.driveTrain.stopDrive();
    SmartDashboard.putNumber("Działam chyba", 0);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
