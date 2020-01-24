/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.util.Color;

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
		public static final double kC2F = (9.0 / 5.0) + 32.0;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
	}

	public static final class ChassisConstants {
		public static final int kLeftMasterMotor = 4;
		public static final int kLeftFollowerMotor = 3;
		public static final int kRightMasterMotor = 1;
		public static final int kRightFollowerMotor = 2;

		public static final double kMaxSpeedFPS = 10.0; // feet per second
		public static final double kMaxSpeedMPS = kMaxSpeedFPS * UnitsConstants.kF2M; // meters per second
		public static final double kMinSpeedMPS = -kMaxSpeedMPS; // meters per second
		public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

		public static final double kTrackWidth = 1.5 * UnitsConstants.kF2M * 2; // meters
		public static final double kWheelDiameter = 6.0 * UnitsConstants.kF2M;
		public static final double kWheelRadius = kWheelDiameter / 2.0;
		public static final int kEncoderResolution = 4096;
		public static final double kVelFactor = (Math.PI * ChassisConstants.kWheelDiameter) / 4096;

		public static final double kP = 1;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;

		public static final int kCompressorChannel = 0;
		public static final int kHiPressureChannel = 0;
		public static final int kLoPressureChannel = 1;
		public static final double kInputVoltage = 5.0;
	}

	public final static class ShooterConstants {
		public static final int kShooterMotor = 10;
		public static final int kAngleMotor = 11;
		public static final int kTiltMotor = 12;

		public static final double kP = 0.00005;
		public static final double kI = 0.000001;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMin = -1;
		public static final double kMax = 1;

		public static final double kTicsPerRev = 4096;
		public static final double kWheelDiameter = 4.0;
		public static final double kVelFactor = (SpinnerConstants.kWheelDiameter * Math.PI)
				/ SpinnerConstants.kTicsPerRev;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = 0.0;
		public static final double kMaxRPM = 5700.0;

		public static final double shootSpeed = 1.0;
		public static final double shootRPMs = 100.0;

		public static final double shooterShootTime = 4;
	}

	public final static class SpinnerConstants {
		public static final int kSpinnerMotor = 20;

		public static final double kP = 0.00005;
		public static final double kI = 0.000001;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = 0.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = 0.0;
		public static final double kMaxRPM = 5700.0;

		public static final double kTicsPerRev = 4096;
		public static final double kWheelDiameter = 3.0;
		public static final double kVelFactor = (SpinnerConstants.kWheelDiameter * Math.PI)
				/ SpinnerConstants.kTicsPerRev;
		public static final double kStopOnColorRPMs = 1000.0;
		public static final double kCountRevRPMs = 7680.0;
		public static final double kStopRPMs = 0.0;

		public static enum COLOR {
			GREEN, BLUE, YELLOW, RED, UNKNOWN;
		}

		/**
		 * Note: Any example colors should be calibrated as the user needs, these are
		 * here as a basic example.
		 */
		public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
		public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
		public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
		public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	}

	public static final class ClimberConstants {
		public static final int kClimberExtendSolenoid = 0;
		public static final int kClimberRetractSolenoid = 1;

		public static final DoubleSolenoid.Value ClimberExtend = DoubleSolenoid.Value.kForward;
		public static final DoubleSolenoid.Value ClimberRetract = DoubleSolenoid.Value.kReverse;
	}

	public static final class CollecterConstants {
		public static final int kCollecterExtendSolenoid = 2;
		public static final int kCollecterRetractSolenoid = 3;

		public static final DoubleSolenoid.Value CollecterExtend = DoubleSolenoid.Value.kForward;
		public static final DoubleSolenoid.Value CollecterRetract = DoubleSolenoid.Value.kReverse;
	}

}
