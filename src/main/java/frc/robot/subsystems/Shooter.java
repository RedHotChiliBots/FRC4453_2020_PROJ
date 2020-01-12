/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.SpeedController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // @Override
  // public void initDefaultCommand() {
  // // Set the default command for a subsystem here.
  // // setDefaultCommand(new MySpecialCommand());
  // }
  // private final SpeedController m_leftMaster = leftMaster;
  private final CANSparkMax shooter = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private final SpeedController shootMotor = shooter;
  private final CANSparkMax angler = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private final SpeedController angleMotor = angler;
  private final CANSparkMax tilter = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private final SpeedController tiltMotor = tilter;

  private final PIDController PIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
      ShooterConstants.kD);

  public Shooter() {

  }

  public void shooterShoot() {

  }

  public void moveToAngle() {

  }

  public void setAngle() {

  }

  public void setTilt() {

  }

  public void moveToTilt() {

  }
}
