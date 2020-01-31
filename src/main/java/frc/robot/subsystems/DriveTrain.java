/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private ADIS16448_IMU gyro;

  private TalonSRX leftMotorMaster;
  private VictorSPX leftMotorSlave;
  private TalonSRX rightMotorMaster;
  private VictorSPX rightMotorSlave;

  //private final DifferentialDriveOdometry m_odometry;

  public DriveTrain() {
    leftMotorMaster = new TalonSRX(Constants.DRIVE_BASE_LEFT_MOTOR_MASTER);
    leftMotorSlave = new VictorSPX(Constants.DRIVE_BASE_LEFT_MOTOR_SLAVE);
    rightMotorMaster = new TalonSRX(Constants.DRIVE_BASE_RIGHT_MASTER);
    rightMotorSlave = new VictorSPX(Constants.DRIVE_BASE_RIGHT_MOTOR_SLAVE);

    //gyro = new ADIS16448_IMU();

    leftMotorSlave.follow(leftMotorMaster);
    rightMotorSlave.follow(rightMotorMaster);

    leftMotorMaster.setInverted(false);
    leftMotorSlave.setInverted(false);
    rightMotorMaster.setInverted(true);
    rightMotorSlave.setInverted(true);
  }

  public void setMotors(double left_speed, double right_speed) {
    left_speed = Math.min(left_speed, 1);     left_speed = Math.max(left_speed, -1);
    right_speed = Math.min(right_speed, 1);     right_speed = Math.max(right_speed, -1);

    SmartDashboard.putNumber("left motor", left_speed);
    SmartDashboard.putNumber("right motor", right_speed);

    leftMotorMaster.set(ControlMode.PercentOutput, left_speed);
    rightMotorMaster.set(ControlMode.PercentOutput, right_speed);
  }

  public double getGyroAngle() {
    //SmartDashboard.putNumber("gyro angle", gyro.getGyroAngleZ());
    return gyro.getGyroAngleZ();
  }

  public void stopDrive() {
    setMotors(0, 0);
  }

  public void resetGyro() {
     gyro.reset();
  }

  @Override
  public void periodic() {
  }
}
