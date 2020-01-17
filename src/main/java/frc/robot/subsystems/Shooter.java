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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.smartdashboard.SmartDashboard;

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
  public CANEncoder tiltEncoder = null;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  // read values periodically
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);

  // post to smart dashboard periodically
  // SmartDashboard.putNumber("LimelightX", x);
  // SmartDashboard.putNumber("LimelightY", y);
  // SmartDashboard.putNumber("LimelightArea", area);

  // private CANDigitalInput angleLim;
  // angleLim=null;
  // angleLim=angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  // m_reverseLimit.enableLimitSwitch(false);

  public Shooter() {
    System.out.println("Shooter starting ...");
    // //angleLim.enableLimitSwitch(false);
    // angleEncoder = angleMotor.getEncoder();
  }

  public static void shoot() {
    shootMotor.set(ShooterConstants.shootSpeed);
  }

  public static void stopShoot() {
    shootMotor.set(0);
  }

  public void moveToAngle(double angle, double speed) {
    angleMotor.set(speed);
    angleEncoder.setPosition(angle); // TODO set constants to adjust from ticks
  }

  public double getAngle() {
    return angleEncoder.getPosition();
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

  public void moveToTilt(double position, double speed) { // TODO can I just set a postion or do I need to do other
                                                          // things to?
    tiltEncoder.setPosition(position);
    tiltMotor.set(speed);
  }

  public void visionTilt() {

  }
  // public double tiltAngle(){
  // return y;
  // }
}
