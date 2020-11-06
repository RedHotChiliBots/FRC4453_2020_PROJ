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
import edu.wpi.first.wpilibj.util.Units;

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
		public static final int kCollectArmMotor = 31;

		public static final int kTopMasterMotor = 40;
		public static final int kBottomFollowerMotor = 41;
		public static final int kLeftEjectorMotor = 42;
		// public static final int kRightEjectorMotor = 43;

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
		public static final double kMaxSpeedMPS = Units.feetToMeters(0.5); // meters per second
		public static final double kMinSpeedMPS = -kMaxSpeedMPS; // meters per second
		public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

		public static final double kTrackWidth = Units.inchesToMeters(26.341); // meters
		public static final double kWheelCirc = Units.inchesToMeters(Math.PI * 8.0); // meters
		public static final int kEncoderResolution = 42; // not used, NEO's native units are rotations
		public static final double kGearBoxRatio = 10.71;
		public static final double kPosFactor = kWheelCirc / kGearBoxRatio; // Meters per Revolution Revolution
		public static final double kVelFactor = kWheelCirc / kGearBoxRatio / 60.0; // Meters per Second

		// Constants for Drive PIDs
		public static final double kP = 0.2;
		public static final double kI = 0.0;
		public static final double kD = 0.0;

		// Ramsete Command constants - same for all robots - do not change
		public static final double kRamseteB = 2.0;
		public static final double kRamseteZeta = 0.7;

		public static final double kDistP = 0.15;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0.0;

		public static final double kRotP = 0.15;
		public static final double kRotI = 0.0;
		public static final double kRotD = 0.0;

		// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
		// These characterization values MUST be determined either experimentally or
		// theoretically for *your* robot's drive.
		// The Robot Characterization Toolsuite provides a convenient tool for obtaining
		// these values for your robot.
		public static final double ksVolts = 0.22;
		public static final double kvVoltSecondsPerMeter = 0.3;
		public static final double kaVoltSecondsSquaredPerMeter = 0.01; // 0.2;

		public static final double kMaxSpeedMetersPerSecond = 1;
		public static final double kMaxAccelerationMetersPerSecondSquared = 0.7;;
	}

	public final static class ShooterConstants {
		public static final double kP = 0.00008;
		// public static final double kI = 0.0;
		public static final double kI = 0.0000004;

		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -.5;
		public static final double kMaxOutput = .5;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = -4540.0;
		public static final double kMaxRPM = 4540.8;

		public static final double kShooterShootRPMs = kMaxRPM;

		public static final double kCameraAngle = 0.0;
		public static final double kCameraHeight = 48.0;
		public static final double kTargetHeight = 85.0;
		public static final double kDistToTarget = kTargetHeight - kCameraHeight;
	}

	public final static class YawConstants {
		public static final double kP = 0.01;
		public static final double kI = 0.000015;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = 0.0;
		public static final double kMaxOutput = 0.5;

		public static final double kMinDeg = -140.0;
		public static final double kMaxDeg = 140.0;
		public static final double kMaxSec = 2.0;
		public static final double kRangeDeg = kMaxDeg - kMinDeg;
		public static final double kRateDpS = kRangeDeg / kMaxSec / 50.0;	// 50x per second

		public static final double kAngleFindSpeed = 0.15;
		public static final double kAngleCenterSpeed = 0.05;

		public static final double kTicsPerMotorRev = 4096;
		public static final double kGearBoxRatio = 21;
		public static final double kTicsPerGearboxRev = kTicsPerMotorRev * kGearBoxRatio;

		public static final double kRGTeeth = 170.0; // Ring Gear
		public static final double kGBTeeth = 18.0; // Gear Box
		public static final double kRingGearboxRatio = kRGTeeth / kGBTeeth;
		public static final double kTicsPerRingRev = kTicsPerGearboxRev * kRingGearboxRatio;
		public static final double kTicsPerDegree = kTicsPerRingRev / 360;

		public static final double kJoystickDeadZone = 0.025;
	}

	public final static class TiltConstants {
		public static final double kP = 0.04;
		public static final double kI = 0.000001;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;

		public static final double kMinDeg = 10.0;
		public static final double kMaxDeg = 45.0;
		public static final double kMaxSec = 1.0;
		public static final double kRangeDeg = kMaxDeg - kMinDeg;
		public static final double kRateDpS = kRangeDeg / kMaxSec / 50.0;	// 50x per second

		public static final double kATiltFindSpeed = 0.3;
		public static final double kTiltAmps = 10.0;

		public static final double kTicsPerMotorRev = 4096;
		public static final double kGearBoxRatio = 12;
		public static final double kTicsPerPinionRev = kTicsPerMotorRev * kGearBoxRatio;

		public static final double kPinionDia = 2.5; // inches
		public static final double kRackDia = 19.0; // inches
		public static final double kRackPinionRatio = kRackDia / kPinionDia;
		public static final double kTicsPerRackRev = kTicsPerPinionRev * kRackPinionRatio;
		public static final double kTicsPerDegree = kTicsPerRackRev / 360.0;

//		public static final double kPinionTeeth = 50;
//		public static final double kDegreesPerRev = 360;
//		public static final double kTeethPerDegree = kPinionTeeth / kDegreesPerRev;
//		public static final double kTeethPerRev = kPinionTeeth / kGBRatio;
//		public static final double kTicsPerTeeth = kTicsPerMotorRev / kTeethPerRev;
//		public static final double kTicsPerDegree = -kTicsPerTeeth * kTeethPerDegree;
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

		public static final double kCollectRPMs = 200.0;
		public static final double kRejectRPMs = -200.0;
	}

	public static final class CollectArmConstants {
		public static final int kCollectArmPowerIndex = 7;

		public static final double kArmFindSpeed = 0.4;
		public static final double kCollectArmAmps = 15.0;

		public static final double kP = 0.0005;
		public static final double kI = 0.0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinOutput = -0.5;
		public static final double kMaxOutput = 0.5;

		public static final double kStopPos = 0.0;
		public static final double kMinPos = 0.0;
		public static final double kMaxPos = 90.0;

		public static final double kGearBoxRatio = 100.0;
		public static final double kDegreesToRotations = kGearBoxRatio / 360.0;

		public static final double kMaxVelocity = 35.0 / 3.0; // inches per second
		public static final double kStopRPMs = 0.0; // inches per second
		public static final double kShooterShootRPMs = kMaxPos;

		public static final double kExtendPos = kMaxPos;
		public static final double kRetractPos = kMinPos;
	}

	public final static class HopperConstants {
		public static final double kP = 0.00008;
		// public static final double kI = 0.0;
		public static final double kI = 0.0000004;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = -4540.8;
		public static final double kMaxRPM = 4540.8; // 80% of Neo Free Spin RPMs

		// public static final double kCPMaxRPM = 60; // Rules limit CP to 1 rps
		// public static final double kCPDiameter = 32; // inches (2' 8")
		// public static final double kWheelDiameter = 2.25;
		// public static final double kWheelRPM = (kCPDiameter / kWheelDiameter) *
		// kCPMaxRPM;

		// public static final double kTicsPerRev = 12; // * 4; // RS7 quad encoder on
		// 775 motor
		// public static final double kGearBoxRatio = 16;
		// public static final double k100msPerMin = 600; // constant
		// public static final double kVelFactor = (kTicsPerRev / k100msPerMin) *
		// kGearBoxRatio;

		public static final double kCollectRPMs = 250; // Guess, so far
		public static final double kLoadRPMs = 1000; // Given 2.25" wheel
		public static final double kShootRPMs = 1200; // Stop spinning
		public static final double kStopRPMs = 0; // Stop spinning

		public static final double kReverseRPMS = -1000;

		// public static final double kHopperShootRPMs = 500.0; // Guess
		// public static final double kHopperLoadRPMs = 250.0; // Guess

		// public static final double kMaxSpeedMPS = Units.feetToMeters(0.5); // meters
		// per second
		// public static final double kMinSpeedMPS = -kMaxSpeedMPS; // meters per second
		// public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation
		// per second

		// public static final double kTrackWidth = Units.inchesToMeters(26.341); //
		// meters
		// public static final double kWheelCirc = Units.inchesToMeters(Math.PI * 8.0);
		// // meters
		// public static final int kEncoderResolution = 42; // not used, NEO's native
		// units are rotations
		// public static final double kGearBoxRatio = 10.71;
		// public static final double kPosFactor = kWheelCirc / kGearBoxRatio; // Meters
		// per Revolution Revolution
		// public static final double kVelFactor = kWheelCirc / kGearBoxRatio / 60.0; //
		// Meters per Second
	}

	public final static class EjectorConstants {
		public static final double kP = 0.08;
		public static final double kI = 0.0000000;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = 0.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = 0;
		public static final double kMaxRPM = 11200; // 80% of Bag Free Spin RPMs

		// public static final double kCPMaxRPM = 60; // Rules limit CP to 1 rps
		// public static final double kCPDiameter = 32; // inches (2' 8")
		// public static final double kWheelDiameter = 2.25;
		// public static final double kWheelRPM = (kCPDiameter / kWheelDiameter) *
		// kCPMaxRPM;

		public static final double kTicsPerRev = 4096; // * 4; // RS7 quad encoder on 775 motor
		public static final double kGearBoxRatio = 9;
		public static final double k100msPerMin = 600; // constant
		public static final double kVelFactor = (kTicsPerRev / k100msPerMin) * kGearBoxRatio;

		public static final double kCollectRPMs = 250; // Guess, so far
		// public static final double kLoadRPMs = kWheelRPM; // Given 2.25" wheel
		public static final double kShootRPMs = 0; // Stop spinning
		public static final double kStopRPMs = 0; // Stop spinning

		public static final double kEjectorShootRPMs = 11000.0; // Guess
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

		public static final double kClimbVelocity = 200.0;
		public static final double kLevelVelocity = 200.0;
	}

	public static final class LevelerConstants {
		public static final double kP = 1.0;
		public static final double kI = 0.005;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;
		public static final double kMinRPM = 0;
		public static final double kMaxRPM = 4500; // 80% of Neo Free Spin RPMs

		public static final double kWheelCirc = 2.5 * Math.PI; // inches
		public static final int kEncoderResolution = 42; // not used, NEO's native units are rotations
		public static final double kGearBoxRatio = 20.0;
		public static final double kVelFactor = kWheelCirc / kGearBoxRatio / 60.0; // Meters per Second

		public static final double kMaxVelocity = 35.0 / 3.0; // inches per second
		public static final double kStopRPMs = 0.0; // inches per second
	}
}
