/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

/**
 * Add your docs here.
 */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final CANSparkMax shootMotor = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private final CANSparkMax angleMotor = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private final CANSparkMax tiltMotor = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);

  private final CANPIDController shootPIDController = new CANPIDController(shootMotor);
  private final CANEncoder shootEncoder = new CANEncoder(shootMotor);
  private final CANPIDController anglePIDController = new CANPIDController(angleMotor);
  private final CANEncoder angleEncoder = new CANEncoder(angleMotor);
  private final CANPIDController tiltPIDController = new CANPIDController(tiltMotor);
  private final CANEncoder tiltEncoder = new CANEncoder(tiltMotor);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  // read values periodically
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);

  // private ShuffleboardTab shooterTab;

  // post to smart dashboard periodically
  // SmartDashboard.putNumber("LimelightX", x);
  // SmartDashboard.putNumber("LimelightY", y);
  // SmartDashboard.putNumber("LimelightArea", area);

  // private CANDigitalInput angleLim;
  // angleLim=null;
  // angleLim=angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  // m_reverseLimit.enableLimitSwitch(false);

  public Shooter() {
    System.out.println("+++++ Shooter Constructor starting ...");
    // ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

    shootPIDController.setP(Constants.ShooterConstants.kP);
    shootPIDController.setI(Constants.ShooterConstants.kI);
    shootPIDController.setD(Constants.ShooterConstants.kD);
    shootPIDController.setIZone(Constants.ShooterConstants.kIz);
    shootPIDController.setFF(Constants.ShooterConstants.kFF);
    shootPIDController.setOutputRange(Constants.ShooterConstants.kMin, Constants.ShooterConstants.kMax);

    shootEncoder.setPositionConversionFactor(2 * Math.PI * ShooterConstants.kWheelRadius / 4096);

    // SmartDashboard.putNumber("SetPoint", setPoint);
    // SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());

    // shooterTab = Shuffleboard.getTab("Shooter");
    // shooterTab.add("Velocity", shootEncoder.getVelocity());
    // shooterTab.add("Position", shootEncoder.getPosition());

    // shooterTab.add("AHRS Angle", ahrs);

    // //angleLim.enableLimitSwitch(false);
    // angleEncoder = angleMotor.getEncoder();
    System.out.println("----- Shooter Constructor finished ...");
  }

  public void periodic() {
    // shooterTab.add("Velocity", shootEncoder.getVelocity());
    // shooterTab.add("Position", shootEncoder.getPosition());
    SmartDashboard.putNumber("Velocity", shootEncoder.getVelocity());
    SmartDashboard.putNumber("Position", shootEncoder.getPosition());
  }

  public void shoot(double setPoint) {
    // shootMotor.set(ShooterConstants.shootSpeed);
    // double setPoint = m_stick.getY() * maxRPM;
    shootPIDController.setReference(setPoint, ControlType.kVelocity);
  }

  public void stopShoot() {
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
