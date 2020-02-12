package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.kLeftMotor1Port);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.kRightMotor1Port);

  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(Constants.kLeftMotor2Port);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(Constants.kRightMotor2Port);

  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(leftMaster,
                               leftSlave);

  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(rightMaster,
                               rightSlave);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();

  private final DifferentialDriveOdometry m_odometry;

  public DriveTrain() {
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS,
    rightMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("left dist", leftMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("right dist", rightMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("left speed", leftMaster.getSelectedSensorVelocity() * 10 * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("right speed", rightMaster.getSelectedSensorVelocity() * 10 * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("gyro", getHeading());
    }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity() * 10 * Constants.TICKS_TO_METERS,
        rightMaster.getSelectedSensorVelocity() * 10 * Constants.TICKS_TO_METERS);
  }


  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }


  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }


  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts); 
    m_drive.feed();
  }


  public void resetEncoders() 
  {
    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.setSelectedSensorPosition(0,0,10);
    rightMaster.setSelectedSensorPosition(0,0,10);

    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(false);
  }


  public double getAverageEncoderDistance() {
    return (leftMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS + rightMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS) / 2.0;
  }

 
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput/5);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }


  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }


  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }
}