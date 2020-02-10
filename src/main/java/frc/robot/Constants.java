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
		public static final double kI2M = kF2M / 12;
		public static final double kC2F = (9.0 / 5.0) + 32.0;
	}

	public static final class CANidConstants {
		public static final int kPDP = 0;
		public static final int kCompressor = 0;

		public static final int kSlotIdx = 0;
		public static final int kPIDLoopIdx = 0;
		public static final int kTimeoutMs = 30;

		public static final int kRightMasterMotor = 1;
		public static final int kRightFollowerMotor = 2;
		public static final int kLeftMasterMotor = 4;
		public static final int kLeftFollowerMotor = 3;

		public static final int kShooterMotor = 10;
		public static final int kAngleMotor = 11;
		public static final int kTiltMotor = 12;

		public static final int kSpinnerMotor = 20;

		public static final int kCollectorMotor = 30;

		public static final int kTopMasterMotor = 40;
		public static final int kBottomFollowerMotor = 41;
		public static final int kLeftEjectorMotor = 42;
		public static final int kRightEjectorMotor = 43;

		public static final int kClimberMotor = 50;
		public static final int kLevelerMotor = 51;
	}

	public static final class PneumaticConstants {
		public static final int kClimberExtendSolenoid = 0;
		public static final int kClimberRetractSolenoid = 1;

		public static final int kCollectorExtendSolenoid = 2;
		public static final int kCollectorRetractSolenoid = 3;
	}

	public static final class DigitalIOConstants {
		public static final int kCenterDigital = 0;
		public static final int kLowerSensor = 1;
		public static final int kUpperSensor = 2;
	}

	public static final class AnalogIOConstants {
		public static final int kHiPressureChannel = 0;
		public static final int kLoPressureChannel = 1;

		public static final double kInputVoltage = 5.0;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
	}

	public static final class ChassisConstants {
		public static final double kMaxSpeedFPS = 0.5; // feet per second
		public static final double kMaxSpeedMPS = kMaxSpeedFPS * UnitsConstants.kF2M; // meters per second
		public static final double kMinSpeedMPS = -kMaxSpeedMPS; // meters per second
		public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

		public static final double kTrackWidth = 26.341 * UnitsConstants.kI2M; // meters
		public static final double kWheelCirc = Math.PI * 8.0 * UnitsConstants.kI2M;
		public static final int kEncoderResolution = 42; // not required
		public static final double kGearBoxRatio = 10.71;
		// The NEO's native units are rotations.
		public static final double kPosFactor = kWheelCirc / kGearBoxRatio; // Meters per Motor Revolution
		public static final double kVelFactor = kWheelCirc / kGearBoxRatio / 60.0; //

		public static final double kP = 0.15;
		public static final double kI = 0.0;
		public static final double kD = 0;

		public static final double kS = 1;
		public static final double kV = 3;
		public static final double kA = 0;

		// Ramsete Command constants - same for all robots
		public static final double kRamseteB = 2.0;
		public static final double kRamseteZeta = 0.7;

		public static final double kDistP = 0.15;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0;

		public static final double kRotP = 0.15;
		public static final double kRotI = 0.0;
		public static final double kRotD = 0;
	}

	public final static class ShooterConstants {
		public static final double kP = 1.0;
		public static final double kI = 0.05;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = 0;
		public static final double kMaxOutput = 1;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = 0.0;
		public static final double kMaxRPM = 4500.0;

		public static final double kShooterShootRPMs = kMaxRPM;

		public static final double kCameraAngle = 0.0;
		public static final double kCameraHeight = 48.0;
		public static final double kTargetHeight = 85.0;
	}

	public final static class AngleConstants {
		public static final double kP = 0.05;
		public static final double kI = 0.0001;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = 0;
		public static final double kMaxOutput = 1;

		public static final double kAngleFindSpeed = 0.8;
		public static final double kAngleCenterSpeed = 0.5;

		public static final double kTicsPerMotorRev = 4096;
		public static final double kGBRatio = 10;
		public static final double kTicsPerGBRev = kTicsPerMotorRev * kGBRatio;

		public static final double kRGTeeth = 170.0; // Ring Gear
		public static final double kGBTeeth = 15.0; // Gear Box
		public static final double kGBRevPerRGRev = (kRGTeeth / kGBTeeth);
		public static final double kPosFactor = (kGBRevPerRGRev * kTicsPerGBRev) / 360.0; // Tics / Deg @ Ring Gear
	}

	public final static class TiltConstants {
		public static final int kTiltPowerIndex = 1;
		public static final double kTiltAmps = 1.0;

		public static final double kP = 0.05;
		public static final double kI = 0.0001;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = 0;
		public static final double kMaxOutput = 1;

		public static final double kTicsPerMotorRev = 4096;
		public static final double kGBRatio = 10;
		public static final double kTicsPerGBRev = kTicsPerMotorRev * kGBRatio;

		public static final double kInPerRevLS = 0.2; // Lead Screw (inches/rev)
		public static final double kLengthLS = 5.55; // Lead Screw max travel inches
		public static final double kTiltDegLS = 35.0; // Tilt Degrees by max Lead Screw travel
		public static final double kInchPerTic = kInPerRevLS / kTicsPerGBRev;
		public static final double kDegPerInch = kTiltDegLS / kLengthLS;
		// public static final double kPosFactor = kInchPerDeg * kInchPerTic; // Tic /
		// Deg }
		public static final double kInchPerDeg = kInPerRevLS / kTiltDegLS;
		public static final double kRevPerIn = 1 / kInPerRevLS;
		public static final double kPosFactor = kInchPerDeg * kRevPerIn * kTicsPerGBRev; // Tic / Deg }

		public static final double kATiltFindSpeed = 0.5;
	}

	public final static class SpinnerConstants {
		public static final double kP = 1.0;
		public static final double kI = 0.005;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = 0.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = 0;
		public static final double kMaxRPM = 15000; // 80% of 775 Free Spin RPMs

		public static final double kCPMaxRPM = 60; // Rules limit CP to 1 rps
		public static final double kCPDiameter = 32; // inches (2' 8")
		public static final double kWheelDiameter = 2.25;
		public static final double kWheelRPM = (kCPDiameter / kWheelDiameter) * kCPMaxRPM;

		public static final double kTicsPerRev = 12; // * 4; // RS7 quad encoder on 775 motor
		public static final double kGearBoxRatio = 16;
		public static final double k100msPerMin = 600; // constant
		public static final double kVelFactor = (kTicsPerRev / k100msPerMin) * kGearBoxRatio;

		public static final double kStopOnColorRPMs = 250; // Guess, so far
		public static final double kCountRevRPMs = kWheelRPM; // Given 2.25" wheel
		public static final double kStopRPMs = 0; // Stop spinning

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

	public static final class CollectorConstants {
		public static final DoubleSolenoid.Value CollectorExtend = DoubleSolenoid.Value.kForward;
		public static final DoubleSolenoid.Value CollectorRetract = DoubleSolenoid.Value.kReverse;

		public static final double kP = 1.0;
		public static final double kI = 0.05;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = 0;
		public static final double kMaxOutput = 1;

		public static final double kTicsPerRev = 12; // * 4; // RS7 quad encoder on 775 motor
		public static final double kGearBoxRatio = 16;
		public static final double k100msPerMin = 600; // constant
		public static final double kVelFactor = (kTicsPerRev / k100msPerMin) * kGearBoxRatio;

		public static final double kCollectRPMs = 500.0;
		public static final double kRejectRPMs = -200.0;
	}

	public final static class HopperConstants {
		public static final double kP = 1.0;
		public static final double kI = 0.005;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = 0.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = 0;
		public static final double kMaxRPM = 15000; // 80% of 775 Free Spin RPMs

		public static final double kCPMaxRPM = 60; // Rules limit CP to 1 rps
		public static final double kCPDiameter = 32; // inches (2' 8")
		public static final double kWheelDiameter = 2.25;
		public static final double kWheelRPM = (kCPDiameter / kWheelDiameter) * kCPMaxRPM;

		public static final double kTicsPerRev = 12; // * 4; // RS7 quad encoder on 775 motor
		public static final double kGearBoxRatio = 16;
		public static final double k100msPerMin = 600; // constant
		public static final double kVelFactor = (kTicsPerRev / k100msPerMin) * kGearBoxRatio;

		public static final double kCollectRPMs = 250; // Guess, so far
		public static final double kLoadRPMs = kWheelRPM; // Given 2.25" wheel
		public static final double kShootRPMs = 0; // Stop spinning
		public static final double kStopRPMs = 0; // Stop spinning

		public static final double kHopperShootRPMs = 500.0; // Guess
		public static final double kHopperLoadRPMs = 250.0; // Guess
	}

	public final static class EjectorConstants {
		public static final double kP = 1.0;
		public static final double kI = 0.005;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = 0.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = 0;
		public static final double kMaxRPM = 15000; // 80% of 775 Free Spin RPMs

		public static final double kCPMaxRPM = 60; // Rules limit CP to 1 rps
		public static final double kCPDiameter = 32; // inches (2' 8")
		public static final double kWheelDiameter = 2.25;
		public static final double kWheelRPM = (kCPDiameter / kWheelDiameter) * kCPMaxRPM;

		public static final double kTicsPerRev = 12; // * 4; // RS7 quad encoder on 775 motor
		public static final double kGearBoxRatio = 16;
		public static final double k100msPerMin = 600; // constant
		public static final double kVelFactor = (kTicsPerRev / k100msPerMin) * kGearBoxRatio;

		public static final double kCollectRPMs = 250; // Guess, so far
		public static final double kLoadRPMs = kWheelRPM; // Given 2.25" wheel
		public static final double kShootRPMs = 0; // Stop spinning
		public static final double kStopRPMs = 0; // Stop spinning

		public static final double kEjectorShootRPMs = 250.0; // Guess
	}

	public static final class ClimberConstants {
		public static final DoubleSolenoid.Value ClimberExtend = DoubleSolenoid.Value.kForward;
		public static final DoubleSolenoid.Value ClimberRetract = DoubleSolenoid.Value.kReverse;

		public static final double kP = 1.0;
		public static final double kI = 0.005;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = 0.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = 0;
		public static final double kMaxRPM = 15000; // 80% of 775 Free Spin RPMs

		public static final double kCPMaxRPM = 60; // Rules limit CP to 1 rps
		public static final double kCPDiameter = 32; // inches (2' 8")
		public static final double kWheelDiameter = 2.25;
		public static final double kWheelRPM = (kCPDiameter / kWheelDiameter) * kCPMaxRPM;

		public static final double kTicsPerRev = 12; // * 4; // RS7 quad encoder on 775 motor
		public static final double kGearBoxRatio = 16;
		public static final double k100msPerMin = 600; // constant
		public static final double kVelFactor = (kTicsPerRev / k100msPerMin) * kGearBoxRatio;

		public static final double kStopOnColorRPMs = 250; // Guess, so far
		public static final double kCountRevRPMs = kWheelRPM; // Given 2.25" wheel
		public static final double kStopRPMs = 0; // Stop spinning
	}

	public static final class LevelerConstants {
		public static final double kP = 1.0;
		public static final double kI = 0.005;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = 0.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = 0;
		public static final double kMaxRPM = 15000; // 80% of 775 Free Spin RPMs

		public static final double kCPMaxRPM = 60; // Rules limit CP to 1 rps
		public static final double kCPDiameter = 32; // inches (2' 8")
		public static final double kWheelDiameter = 2.25;
		public static final double kWheelRPM = (kCPDiameter / kWheelDiameter) * kCPMaxRPM;

		public static final double kTicsPerRev = 12; // * 4; // RS7 quad encoder on 775 motor
		public static final double kGearBoxRatio = 16;
		public static final double k100msPerMin = 600; // constant
		public static final double kVelFactor = (kTicsPerRev / k100msPerMin) * kGearBoxRatio;

		public static final double kStopOnColorRPMs = 250; // Guess, so far
		public static final double kCountRevRPMs = kWheelRPM; // Given 2.25" wheel
		public static final double kStopRPMs = 0; // Stop spinning
	}
}
