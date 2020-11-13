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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Library;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CANidConstants;

/**
 * Add your docs here.
 */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final CANSparkMax shootMotor = new CANSparkMax(CANidConstants.kShooterMotor, MotorType.kBrushless);

  private final CANPIDController shootPIDController = new CANPIDController(shootMotor);
  private final CANEncoder shootEncoder = new CANEncoder(shootMotor);

  private double shootSetPoint = 0.0;


  Library lib = new Library();

  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private NetworkTableEntry sbShootVel = shooterTab.addPersistent("ShootVelocity", 0).getEntry();
  private NetworkTableEntry sbShootSetPoint = shooterTab.addPersistent("Shoot SetPoint", 0).getEntry();

  public Shooter() {
    System.out.println("+++++ Shooter Constructor starting ...");
    // ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  
    // Define Shooter motor
    shootMotor.restoreFactoryDefaults();
    shootMotor.clearFaults();

    shootMotor.setIdleMode(IdleMode.kBrake);
    shootMotor.setInverted(true);

    shootPIDController.setP(ShooterConstants.kP);
    shootPIDController.setI(ShooterConstants.kI);
    shootPIDController.setD(ShooterConstants.kD);
    shootPIDController.setIZone(ShooterConstants.kIz);
    shootPIDController.setFF(ShooterConstants.kFF);
    shootPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // shootEncoder.setVelocityConversionFactor(ShooterConstants.kVelFactor);

    setShootVelocity(ShooterConstants.kStopRPMs);

    System.out.println("----- Shooter Constructor finished ...");
  }

  public void periodic() {
		sbShootSetPoint.setDouble(shootSetPoint);
    sbShootVel.setDouble(getShootVelocity());
  }

  public double getShootVelocity() {
    return shootEncoder.getVelocity();
  }

  public void setShootVelocity(double rpm) {
    this.shootSetPoint = lib.Clip(-rpm, ShooterConstants.kMaxRPM, ShooterConstants.kMinRPM);
    shootPIDController.setReference(shootSetPoint, ControlType.kVelocity);
  }

  // public void shoot(double setPoint) {
  // // shootMotor.set(ShooterConstants.shootSpeed);
  // // double setPoint = m_stick.getY() * maxRPM;
  // shootPIDController.setReference(setPoint, ControlType.kVelocity);
  // }

  public void stopShoot() {
    shootMotor.set(0);
  }
}
