/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

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
  private static final CANSparkMax shooter = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private static final SpeedController shootMotor = shooter;
  private final CANSparkMax angler = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private final SpeedController angleMotor = angler;
  // private final CANSparkMaxSendable angleMotor = angler;
  private final CANSparkMax tilter = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private final SpeedController tiltMotor = tilter;

  private final PIDController PIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
      ShooterConstants.kD);

  public CANEncoder angleEncoder = null;

  // private CANDigitalInput angleLim;
  // angleLim=null;
  // angleLim=angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  // m_reverseLimit.enableLimitSwitch(false);

  // public Shooter() {
  // //angleLim.enableLimitSwitch(false);
  // angleEncoder = angleMotor.getEncoder();
  // }

  public static void shoot() {
    shootMotor.set(ShooterConstants.shootSpeed);
  }

  public static void stopShoot() {
    shootMotor.set(0);
  }

  public void moveToAngle(double angle) {
    // angle = ;
    // angleMotor.set(speed);
  }

  // public boolean isLimit() {
  // return
  // }

  // public void resetEncoders() {
  // angleEncoder.rese
  // }
  // public void zeroAngle() {
  // if (isLimit() == true) {
  // angleEncoder.setPosition(0);
  // }
  // }

  public void moveToTilt() {
    // TiltMotor.set();
  }
}
