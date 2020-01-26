/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
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

import frc.robot.Library;
import frc.robot.Constants.ShooterConstants;

/**
 * Add your docs here.
 */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final CANSparkMax shootMotor = new CANSparkMax(ShooterConstants.kShooterMotor, MotorType.kBrushless);
  private final TalonSRX angleMotor = new TalonSRX(ShooterConstants.kAngleMotor);
  private final TalonSRX tiltMotor = new TalonSRX(ShooterConstants.kTiltMotor);

  private final CANPIDController shootPIDController = new CANPIDController(shootMotor);
  private final CANEncoder shootEncoder = new CANEncoder(shootMotor);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  // read values periodically
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);

  private double setPoint = 0.0;

  // private ShuffleboardTab shooterTab;

  // post to smart dashboard periodically
  // SmartDashboard.putNumber("LimelightX", x);
  // SmartDashboard.putNumber("LimelightY", y);
  // SmartDashboard.putNumber("LimelightArea", area);

  // private CANDigitalInput angleLim;
  // angleLim=null;
  // angleLim=angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  // m_reverseLimit.enableLimitSwitch(false);

  Library lib = new Library();

  public Shooter() {
    System.out.println("+++++ Shooter Constructor starting ...");
    // ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

    // Define Shooter motor
    shootMotor.restoreFactoryDefaults();
    shootMotor.clearFaults();

    shootMotor.setIdleMode(IdleMode.kBrake);
    shootMotor.setInverted(false);

    shootPIDController.setP(ShooterConstants.kP);
    shootPIDController.setI(ShooterConstants.kI);
    shootPIDController.setD(ShooterConstants.kD);
    shootPIDController.setIZone(ShooterConstants.kIz);
    shootPIDController.setFF(ShooterConstants.kFF);
    shootPIDController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax);

    shootEncoder.setVelocityConversionFactor(ShooterConstants.kVelFactor);

    // Define Angle motor
    angleMotor.configFactoryDefault();
    angleMotor.clearStickyFaults();

    // Configure Motor
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.setInverted(false);
    angleMotor.setSensorPhase(true);

    /* Config sensor used for Primary PID [Velocity] */
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    angleMotor.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
    angleMotor.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
    angleMotor.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    angleMotor.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

    /* Config the PID values */
    angleMotor.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kFF, ShooterConstants.kTimeoutMs);
    angleMotor.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, ShooterConstants.kTimeoutMs);
    angleMotor.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.kI, ShooterConstants.kTimeoutMs);
    angleMotor.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.kD, ShooterConstants.kTimeoutMs);

    // Define Tilt motor
    tiltMotor.configFactoryDefault();
    tiltMotor.clearStickyFaults();

    // Configure Motor
    tiltMotor.setNeutralMode(NeutralMode.Brake);
    tiltMotor.setInverted(false);
    tiltMotor.setSensorPhase(true);

    /* Config sensor used for Primary PID [Velocity] */
    tiltMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    tiltMotor.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
    tiltMotor.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
    tiltMotor.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    tiltMotor.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

    /* Config PID Values */
    tiltMotor.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kFF, ShooterConstants.kTimeoutMs);
    tiltMotor.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, ShooterConstants.kTimeoutMs);
    tiltMotor.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.kI, ShooterConstants.kTimeoutMs);
    tiltMotor.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.kD, ShooterConstants.kTimeoutMs);

    setShootSetPoint(ShooterConstants.kStopRPMs);

    System.out.println("----- Shooter Constructor finished ...");
  }

  public void periodic() {
    // SmartDashboard.putNumber("ShootVelocity", shootEncoder.getVelocity());
    // SmartDashboard.putNumber("ShootPosition", shootEncoder.getPosition());
    // SmartDashboard.putNumber("AngleVelocity", angleEncoder.getVelocity());
    // SmartDashboard.putNumber("AnglePosition", angleEncoder.getPosition());
  }

  public void setShootSetPoint(double rpm) {
    this.setPoint = lib.Clip(rpm, ShooterConstants.kMaxRPM, ShooterConstants.kMinRPM);
    shootPIDController.setReference(setPoint, ControlType.kVelocity);
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
    // angleMotor.set(speed);
    // angleEncoder.setPosition(angle); // TODO set constants to adjust from ticks
    // anglePIDController.setReference(angle, ControlType.kVelocity);
  }

  public double getAngle() {
    return 0;// angleEncoder.getPosition();
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
    // tiltEncoder.setPosition(position);
    // tiltMotor.set(speed);
  }

  public void visionTilt() {

  }
  // public double tiltAngle(){
  // return y;
  // }
}
