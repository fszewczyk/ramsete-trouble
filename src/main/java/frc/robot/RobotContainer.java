package frc.robot;

import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class RobotContainer { 

  public RobotContainer() {
     configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() 
  {
       var autoVoltageConstraint =
       new DifferentialDriveVoltageConstraint(
           new SimpleMotorFeedforward(Constants.ksVolts,
                                      Constants.kvVoltSecondsPerMeter,
                                      Constants.kaVoltSecondsSquaredPerMeter),
           Constants.kDriveKinematics,
           10);

   TrajectoryConfig config =
       new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                            Constants.kMaxAccelerationMetersPerSecondSquared)
           .setKinematics(Constants.kDriveKinematics)
           .addConstraint(autoVoltageConstraint);

   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
       new Pose2d(0, 0, new Rotation2d(0)),
       List.of(
           new Translation2d(1, 1),
           new Translation2d(1.1,2)
       ),
       new Pose2d(5.2, 3, new Rotation2d(0)),
       config
   );

   RamseteCommand ramseteCommand = new RamseteCommand(
       exampleTrajectory,
       Robot.driveTrain::getPose,
       new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
       new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
       Constants.kDriveKinematics,
       Robot.driveTrain::getWheelSpeeds,
       new PIDController(0.01, 0, 0),
       new PIDController(0.01, 0, 0),
       Robot.driveTrain::tankDriveVolts,
       Robot.driveTrain
   );

   return ramseteCommand.andThen(() -> Robot.driveTrain.tankDriveVolts(0, 0));
 }
}
 
