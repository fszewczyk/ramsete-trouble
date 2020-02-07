/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
	public static final double TICKS_TO_METERS = 0.00004349359699;

    public static final double ksVolts = 0.948;
	public static final double kvVoltSecondsPerMeter = 2.93;
    public static final double kaVoltSecondsSquaredPerMeter = 0.451;

	public static final double kPDriveVel = 0;
	
	public static final double kTrackwidthMeters = 0.56;
    public static final DifferentialDriveKinematics kDriveKinematics =
		new DifferentialDriveKinematics(kTrackwidthMeters);
	
	public static final double kMaxSpeedMetersPerSecond = 0.5;
	public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
	
	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

	public static final boolean kGyroReversed = false;

	public static int kLeftMotor1Port = 1;
	public static int kLeftMotor2Port = 2;

	public static int kRightMotor1Port = 3;
	public static int kRightMotor2Port = 4;
}
