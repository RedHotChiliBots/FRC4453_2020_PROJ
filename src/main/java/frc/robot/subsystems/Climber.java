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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.d
//import edu.wpi.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LevelerConstants;

/**
 * Add your docs here.
 */

public class Climber extends SubsystemBase {

	DoubleSolenoid climberSolenoid = new DoubleSolenoid(ClimberConstants.kClimberExtendSolenoid,
			ClimberConstants.kClimberRetractSolenoid);

	private final CANSparkMax climbMotor = new CANSparkMax(ClimberConstants.kClimberMotor, MotorType.kBrushless);

	private final CANPIDController climbPIDController = new CANPIDController(climbMotor);
	private final CANEncoder climbEncoder = new CANEncoder(climbMotor);

	private double climbSetPoint;

	private final CANSparkMax levelMotor = new CANSparkMax(LevelerConstants.kLevelerMotor, MotorType.kBrushless);

	private final CANPIDController levelPIDController = new CANPIDController(levelMotor);
	private final CANEncoder levelEncoder = new CANEncoder(levelMotor);

	private double levelSetPoint;

	private final Library lib = new Library();

	// climberSolenoid.set(kOff);
	// climberSolenoid.set(kForward);
	// climberSolenoid.set(kReverse);

	public Climber() {
		System.out.println("+++++ Climber Constructor starting ...");

		SmartDashboard.putData("Climber Solenoid", climberSolenoid);

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
		SmartDashboard.putNumber("ClimbVelocity", getClimbVelocity());
		SmartDashboard.putNumber("LevelVelocity", getLevelVelocity());
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

	// public void climb(double setPoint) {
	// // climbMotor.set(ClimberConstants.climbSpeed);
	// // double setPoint = m_stick.getY() * maxRPM;
	// climbPIDController.setReference(setPoint, ControlType.kVelocity);
	// }

	public void stopClimb() {
		climbMotor.set(0);
	}

	public void climberExtend() {
		climberSolenoid.set(ClimberConstants.ClimberExtend);
	}

	public void climberRetract() {
		climberSolenoid.set(ClimberConstants.ClimberRetract);
	}
}
