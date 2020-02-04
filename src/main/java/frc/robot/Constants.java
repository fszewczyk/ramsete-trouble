/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
	public static final double TICKS_TO_CENTIMETERS = 0.004349359699;

	 // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
	public static final double kPDriveVel = 8.5;
	
	public static final double kTrackwidthMeters = 0.56;
    public static final DifferentialDriveKinematics kDriveKinematics =
		new DifferentialDriveKinematics(kTrackwidthMeters);
	
	public static final double kMaxSpeedMetersPerSecond = 3;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
	
	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

	public static final boolean kGyroReversed = false;

	public static int kLeftMotor1Port = 1;
	public static int kLeftMotor2Port = 2;

	public static int kRightMotor1Port = 3;
	public static int kRightMotor2Port = 4;
}
