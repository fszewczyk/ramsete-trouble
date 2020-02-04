/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

	public static final int DRIVE_BASE_LEFT_MOTOR_MASTER = 1;
	public static final int DRIVE_BASE_LEFT_MOTOR_SLAVE = 2;
	public static final int DRIVE_BASE_RIGHT_MASTER = 3;
	public static final int DRIVE_BASE_RIGHT_MOTOR_SLAVE = 4;

	public static final double DRIVE_CONTROL = 0.4;

	public static final double PAD_DEADBOUND = 0.1;

	public static final double POWER_PORT_INACCURACY = 0.1;

	// Drive base PID constants
	public static final double DRIVE_BASE_Kp = 0.05;
	public static final double DRIVE_BASE_Ki = 0.0;
	public static final double DRIVE_BASE_Kd = 0.0;
	public static final double DRIVE_BASE_MIN_I = 0.0;
	public static final double DRIVE_BASE_MAX_I = 0.0;
	public static final double DRIVE_BASE_PID_TOLERANCE = 5;

	public static final int PAD_PORT = 0;

	public static final int LX_AXIS = 0;
	public static final int LY_AXIS = 1;
	public static final int RX_AXIS = 4;
	public static final int RY_AXIS = 5;
	public static final int L_TRIGGER = 2;
	public static final int R_TRIGGER = 3;
	public static final int Y_BUTTON = 4;

	// Feedforward/Feedback Gains for Trajectory Calculations
	public static final double KS_VOLTS = 0;
	public static final double KV_VOLT_SECONDS_PER_METER = 0;
	public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0;
	public static final double KP_DRIVE_VEL = 0;

	public static final double K_TRACK_WIDTH_METERS = 0;
	public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
	public static final double k_MAX_ACCELERATION_METERS_PR_SECOND_SQUARED = 3;
	public static final double K_RAMSETE_B = 2;
	public static final double K_RAMSETE_ZETA = 0.7;

	public static final double TICKS_TO_CENTIMETERS = 0.004349359699;


	public static final double TALON_KF = 1023.0/7200.0;
	public static final double TALON_KP = 0.05 ;
	public static final double TALON_KI = 0.001;
	public static final double TALON_KD = 0.001;
	public static final int PID_LOOP_ID = 0;
	public static final int TIMEOUT = 100;
	public static final double SPEED_CONVERT = 8525;
}
