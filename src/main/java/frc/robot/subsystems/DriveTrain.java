/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
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
    setMotorControllers();
    setEncoders();
    setGyro();
    resetGyro();
  }

  public void setMotors(double left_speed, double right_speed) {
    left_speed = Math.min(left_speed, 1);     left_speed = Math.max(left_speed, -1);
    right_speed = Math.min(right_speed, 1);     right_speed = Math.max(right_speed, -1);

    SmartDashboard.putNumber("left motor", left_speed);
    SmartDashboard.putNumber("right motor", right_speed);

    leftMotorMaster.set(ControlMode.PercentOutput, left_speed*Constants.DRIVE_CONTROL);
    rightMotorMaster.set(ControlMode.PercentOutput, right_speed*Constants.DRIVE_CONTROL);

    
    getLeftEncoderPosition();
    getRightEncoderPosition();
    getLeftEncoderVelocity();
    getRightEncoderVelocity();
    getGyroAngle();
  }

  public void setMotorControllers() {
    leftMotorMaster = new TalonSRX(Constants.DRIVE_BASE_LEFT_MOTOR_MASTER);
    leftMotorSlave = new VictorSPX(Constants.DRIVE_BASE_LEFT_MOTOR_SLAVE);
    rightMotorMaster = new TalonSRX(Constants.DRIVE_BASE_RIGHT_MASTER);
    rightMotorSlave = new VictorSPX(Constants.DRIVE_BASE_RIGHT_MOTOR_SLAVE);

    leftMotorSlave.follow(leftMotorMaster);
    rightMotorSlave.follow(rightMotorMaster);

    leftMotorMaster.setInverted(false);
    leftMotorSlave.setInverted(false);
    rightMotorMaster.setInverted(true);
    rightMotorSlave.setInverted(true);
  }

  public void setEncoders() {
    leftMotorMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder,0,10);
    leftMotorMaster.setSensorPhase(true);
    leftMotorMaster.setSelectedSensorPosition(0,0,10);
    rightMotorMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder,0,10);
    rightMotorMaster.setSensorPhase(true);
    rightMotorMaster.setSelectedSensorPosition(0,0,10);
  }

  public double getLeftEncoderVelocity()  {
    SmartDashboard.putNumber("left encoder speed", leftMotorMaster.getSelectedSensorVelocity() * 0.1 * Constants.TICKS_TO_CENTIMETERS);
    return leftMotorMaster.getSelectedSensorVelocity() * 0.1 * Constants.TICKS_TO_CENTIMETERS;
  }

  public double getRightEncoderVelocity()  {
    SmartDashboard.putNumber("right encoder speed", rightMotorMaster.getSelectedSensorVelocity() * 0.1 * Constants.TICKS_TO_CENTIMETERS);
    return rightMotorMaster.getSelectedSensorVelocity() * 0.1 * Constants.TICKS_TO_CENTIMETERS;
  }

  public double getLeftEncoderPosition()  {
    SmartDashboard.putNumber("left encoder", leftMotorMaster.getSelectedSensorPosition()*Constants.TICKS_TO_CENTIMETERS);
    return leftMotorMaster.getSelectedSensorPosition()*Constants.TICKS_TO_CENTIMETERS;
  }

  public double getRightEncoderPosition()  {
    SmartDashboard.putNumber("right encoder", rightMotorMaster.getSelectedSensorPosition()*Constants.TICKS_TO_CENTIMETERS);
    return rightMotorMaster.getSelectedSensorPosition()*Constants.TICKS_TO_CENTIMETERS;
  }

  public void setGyro() {
    gyro = new ADIS16448_IMU();
  }

  public double getGyroAngle() {
    SmartDashboard.putNumber("gyro angle", gyro.getGyroAngleZ());
    return gyro.getGyroAngleZ();
  }

  public void resetGyro() {
    gyro.reset();
 }

  public void stopDrive() {
    setMotors(0, 0);
  }

  @Override
  public void periodic() {
  }
}
