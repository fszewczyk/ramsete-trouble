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

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveTrain() {
    // Sets the distance per pulse for the encoders
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS,
        rightMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("left dist", leftMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("right dist", rightMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("left speed", leftMaster.getSelectedSensorVelocity() * 10 * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("right speed", rightMaster.getSelectedSensorVelocity() * 10 * Constants.TICKS_TO_METERS);
    SmartDashboard.putNumber("gyro", getHeading());
    }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity() * 10 * Constants.TICKS_TO_METERS,
        rightMaster.getSelectedSensorVelocity() * 10 * Constants.TICKS_TO_METERS);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts); 
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() 
  {
    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.setSelectedSensorPosition(0,0,10);
    rightMaster.setSelectedSensorPosition(0,0,10);

    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(false);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS + rightMaster.getSelectedSensorPosition() * Constants.TICKS_TO_METERS) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  /*public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }*/

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
 /*public Encoder getRightEncoder() {
    return m_rightEncoder;
  }*/

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput/5);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }
}