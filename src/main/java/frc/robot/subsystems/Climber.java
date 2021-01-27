/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj2.d
//import edu.wpi.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LevelerConstants;
import frc.robot.Library;

/**
 * Add your docs here.
 */

public class Climber extends SubsystemBase {

	// DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticConstants.kClimberExtendSolenoid,
	// 		PneumaticConstants.kClimberRetractSolenoid);

	private final CANSparkMax climbMotor = new CANSparkMax(CANidConstants.kClimberMotor, MotorType.kBrushless);

	private final CANPIDController climbPIDController = climbMotor.getPIDController();
	// new CANPIDController(climbMotor);
	private final CANEncoder climbEncoder = climbMotor.getEncoder();
	// new CANEncoder(climbMotor);

	private double climbSetPoint;

	private final CANSparkMax levelMotor = new CANSparkMax(CANidConstants.kLevelerMotor, MotorType.kBrushless);

	private final CANPIDController levelPIDController = levelMotor.getPIDController();
	// new CANPIDController(levelMotor);
	private final CANEncoder levelEncoder = levelMotor.getEncoder();
	// new CANEncoder(levelMotor);

	private double levelSetPoint;

	private final Library lib = new Library();

	private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
	private NetworkTableEntry sbClimbVel = climberTab.addPersistent("Climb Velocity", 0).getEntry();
	private NetworkTableEntry sbLevelVel = climberTab.addPersistent("Level Velocity", 0).getEntry();
	private NetworkTableEntry sbClimbTgt = climberTab.addPersistent("Climb Target", 0).getEntry();
	private NetworkTableEntry sbLevelTgt = climberTab.addPersistent("Level Target", 0).getEntry();


	// climberSolenoid.set(kOff);
	// climberSolenoid.set(kForward);
	// climberSolenoid.set(kReverse);

	public Climber() {
		System.out.println("+++++ Climber Constructor starting ...");

		// Define Climber motor
		climbMotor.restoreFactoryDefaults();
		climbMotor.clearFaults();

		climbMotor.setIdleMode(IdleMode.kBrake);
		climbMotor.setInverted(true);

		climbPIDController.setP(ClimberConstants.kP);
		climbPIDController.setI(ClimberConstants.kI);
		climbPIDController.setD(ClimberConstants.kD);
		climbPIDController.setIZone(ClimberConstants.kIz);
		climbPIDController.setFF(ClimberConstants.kFF);
		climbPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

		// Define Leveler motor
		levelMotor.restoreFactoryDefaults();
		levelMotor.clearFaults();

		levelMotor.setIdleMode(IdleMode.kBrake);
		levelMotor.setInverted(true);

		levelPIDController.setP(LevelerConstants.kP);
		levelPIDController.setI(LevelerConstants.kI);
		levelPIDController.setD(LevelerConstants.kD);
		levelPIDController.setIZone(LevelerConstants.kIz);
		levelPIDController.setFF(LevelerConstants.kFF);
		levelPIDController.setOutputRange(LevelerConstants.kMinOutput, LevelerConstants.kMaxOutput);

		setClimbVelocity(ClimberConstants.kStopRPMs);
		setLevelVelocity(LevelerConstants.kStopRPMs);

		System.out.println("----- Climber Constructor finished ...");
	}

	public void periodic() {
		sbClimbVel.setDouble(getClimbVelocity());
		sbLevelVel.setDouble(getLevelVelocity());
		sbClimbTgt.setDouble(climbSetPoint);
		sbLevelTgt.setDouble(levelSetPoint);

		// String str = "";
		// if (climberSolenoid.get() == ClimberConstants.ClimberExtend) {
		// 	str = "Extend";
		// } else if (climberSolenoid.get() == ClimberConstants.ClimberRetract) {
		// 	str = "Retract";
		// } else {
		// 	str = "Unknown";
		// }
		// sbClimbSolenoid.setString(str);
	}

	public double getClimbVelocity() {
		return climbEncoder.getVelocity();
	}

	public void setClimbVelocity(double rpm) {
		this.climbSetPoint = lib.Clip(rpm, ClimberConstants.kMaxRPM, ClimberConstants.kMinRPM);
		climbPIDController.setReference(climbSetPoint, ControlType.kVelocity);
	}

	public double getLevelVelocity() {
		return levelEncoder.getVelocity();
	}

	public void setLevelVelocity(double rpm) {
		this.levelSetPoint = lib.Clip(rpm, LevelerConstants.kMaxRPM, LevelerConstants.kMinRPM);
		levelPIDController.setReference(levelSetPoint, ControlType.kVelocity);
	}

	public void levelTeleop(double speed) {
		levelMotor.set(speed);
	}

	// public boolean isLevel() {	//TODO Unlevel navx
	// return Robot.Subsystems.Chassis.getPitch() < 5;
	// }

	// public void climb(double setPoint) {
	// // climbMotor.set(ClimberConstants.climbSpeed);
	// // double setPoint = m_stick.getY() * maxRPM;
	// climbPIDController.setReference(setPoint, ControlType.kVelocity);
	// }

	public void stopClimb() {
		climbMotor.set(0);
	}

	public void stopLevel() {
		levelMotor.set(0);
	}

	// public void climberExtend() {
	// 	climberSolenoid.set(ClimberConstants.ClimberExtend);
	// }

	// public void climberRetract() {
	// 	climberSolenoid.set(ClimberConstants.ClimberRetract);
	// }
}
