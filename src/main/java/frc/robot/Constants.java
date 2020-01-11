/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class UnitsConstants {
		public static final double kF2M = 0.3048;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
	}

	public static final class ChassisConstants {
		public static final int kLeftMasterMotor = 0;
		public static final int kLeftFollowerMotor = 1;
		public static final int kRightMasterMotor = 2;
		public static final int kRightFollowerMotor = 3;

		public static final double kMaxSpeedFPS = 10.0; // feet per second
		public static final double kMaxSpeedMPS = kMaxSpeedFPS * UnitsConstants.kF2M; // meters per second
		public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

		public static final double kTrackWidth = 1.5 * UnitsConstants.kF2M * 2; // meters
		public static final double kWheelDiameter = 6.0 * UnitsConstants.kF2M;
		public static final double kWheelRadius = kWheelDiameter / 2.0;
		public static final int kEncoderResolution = 4096;

		public static final double kP = 1;
		public static final double kI = 0;
		public static final double kD = 0;
	}

	public final static class ShooterConstants {
		public static final int kShooterMotor = 10;
		public static final int kAngleMotor = 11;
		public static final int kTiltMotor = 12;

		public static final double kP = 1;
		public static final double kI = 0;
		public static final double kD = 0;
	}

	public final static class SpinnerConstants {
		public static final int kSpinnerMotor = 20;

		public static final double kP = 1;
		public static final double kI = 0;
		public static final double kD = 0;
	}
}
