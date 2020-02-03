/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Library;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DigitalIOConstants;

/**
 * Add your docs here.
 */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final PowerDistributionPanel pdp = new PowerDistributionPanel(0);

  private final CANSparkMax shootMotor = new CANSparkMax(CANidConstants.kShooterMotor, MotorType.kBrushless);
  private final TalonSRX angleMotor = new TalonSRX(CANidConstants.kAngleMotor);
  private final TalonSRX tiltMotor = new TalonSRX(CANidConstants.kTiltMotor);

  private final CANPIDController shootPIDController = new CANPIDController(shootMotor);
  private final CANEncoder shootEncoder = new CANEncoder(shootMotor);

  private final DigitalInput angleCenterPos = new DigitalInput(DigitalIOConstants.kCenterDigital);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  // read values periodically
  public double x = tx.getDouble(0.0);
  public double y = ty.getDouble(0.0);
  public double area = ta.getDouble(0.0);

  private double shootSetPoint = 0.0;
  private double tiltSetPoint = 0.0;
  private double angleSetPoint = 0.0;

  // private ShuffleboardTab shooterTab;

  // post to smart dashboard periodically
  // SmartDashboard.putNumber("LimelightX", x);
  // SmartDashboard.putNumber("LimelightY", y);
  // SmartDashboard.putNumber("LimelightArea", area);

  // private CANDigitalInput angleLim;
  // angleLim = null;
  // angleLim =
  // angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  // m_reverseLimit.enableLimitSwitch(false);

  Library lib = new Library();

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

    // Define Angle motor
    angleMotor.configFactoryDefault();
    angleMotor.clearStickyFaults();

    // Configure Motor
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.setInverted(false);
    angleMotor.setSensorPhase(false);

    /* Config sensor used for Primary PID [Velocity] */
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, AngleConstants.kPIDLoopIdx,
        AngleConstants.kTimeoutMs);

    angleMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    angleMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    /* Config the peak and nominal outputs */
    angleMotor.configNominalOutputForward(0, AngleConstants.kTimeoutMs);
    angleMotor.configNominalOutputReverse(0, AngleConstants.kTimeoutMs);
    angleMotor.configPeakOutputForward(1, AngleConstants.kTimeoutMs);
    angleMotor.configPeakOutputReverse(-1, AngleConstants.kTimeoutMs);

    /* Config the PID values */
    angleMotor.config_kF(ShooterConstants.kPIDLoopIdx, AngleConstants.kFF, AngleConstants.kTimeoutMs);
    angleMotor.config_kP(ShooterConstants.kPIDLoopIdx, AngleConstants.kP, AngleConstants.kTimeoutMs);
    angleMotor.config_kI(ShooterConstants.kPIDLoopIdx, AngleConstants.kI, AngleConstants.kTimeoutMs);
    angleMotor.config_kD(ShooterConstants.kPIDLoopIdx, AngleConstants.kD, AngleConstants.kTimeoutMs);

    angleMotor.getSensorCollection().setQuadraturePosition(0, AngleConstants.kTimeoutMs);

    // Define Tilt motor
    tiltMotor.configFactoryDefault();
    tiltMotor.clearStickyFaults();

    // Configure Motor
    tiltMotor.setNeutralMode(NeutralMode.Brake);
    tiltMotor.setInverted(false);
    tiltMotor.setSensorPhase(false);

    /* Config sensor used for Primary PID [Velocity] */
    tiltMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TiltConstants.kPIDLoopIdx,
        TiltConstants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    tiltMotor.configNominalOutputForward(0, TiltConstants.kTimeoutMs);
    tiltMotor.configNominalOutputReverse(0, TiltConstants.kTimeoutMs);
    tiltMotor.configPeakOutputForward(1, TiltConstants.kTimeoutMs);
    tiltMotor.configPeakOutputReverse(-1, TiltConstants.kTimeoutMs);

    /* Config PID Values */
    tiltMotor.config_kF(ShooterConstants.kPIDLoopIdx, TiltConstants.kFF, TiltConstants.kTimeoutMs);
    tiltMotor.config_kP(ShooterConstants.kPIDLoopIdx, TiltConstants.kP, TiltConstants.kTimeoutMs);
    tiltMotor.config_kI(ShooterConstants.kPIDLoopIdx, TiltConstants.kI, TiltConstants.kTimeoutMs);
    tiltMotor.config_kD(ShooterConstants.kPIDLoopIdx, TiltConstants.kD, TiltConstants.kTimeoutMs);

    tiltMotor.getSensorCollection().setQuadraturePosition(0, TiltConstants.kTimeoutMs);

    setShootVelocity(ShooterConstants.kStopRPMs);

    System.out.println("----- Shooter Constructor finished ...");
  }

  public void periodic() {
    SmartDashboard.putNumber("ShootVelocity", getShootVelocity());
    SmartDashboard.putNumber("Angle Position", getAnglePosition());
    SmartDashboard.putNumber("Tilt Position", getTiltPosition());
    SmartDashboard.putNumber("Angle SetPoint", shootSetPoint);
    SmartDashboard.putNumber("Angle SetPoint", angleSetPoint);
    SmartDashboard.putNumber("Tilt SetPoint", tiltSetPoint);
  }

  public double getShootVelocity() {
    return shootEncoder.getVelocity();
  }

  public void setShootVelocity(double rpm) {
    this.shootSetPoint = lib.Clip(rpm, ShooterConstants.kMaxRPM, ShooterConstants.kMinRPM);
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

  public void stopAngle() {
    angleMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopTilt() {
    tiltMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public double getAnglePosition() {
    return angleMotor.getSelectedSensorPosition() / AngleConstants.kPosFactor;
  }

  public void setAnglePosition(double pos) {
    angleSetPoint = pos;
    angleMotor.set(ControlMode.Position, pos * AngleConstants.kPosFactor);
  }

  public void moveAngleLeft(double spd) {
    angleMotor.set(ControlMode.PercentOutput, spd);
  }

  public void moveAngleRight(double spd) {
    angleMotor.set(ControlMode.PercentOutput, -spd);
  }

  public boolean getAngleLeftLimit() {
    return angleMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getAngleCenterPos() {
    return angleCenterPos.get();
  }

  public boolean getAngleRightLimit() {
    return angleMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void setAngleZeroPos() {
    angleMotor.getSensorCollection().setQuadraturePosition(0, TiltConstants.kTimeoutMs);
  }

  public double getTiltPosition() {
    return tiltMotor.getSelectedSensorPosition() / TiltConstants.kPosFactor;
  }

  public void setTiltPosition(double pos) {
    tiltSetPoint = pos;
    tiltMotor.set(ControlMode.Position, pos * TiltConstants.kPosFactor);
  }

  public void moveTiltDown(double spd) {
    tiltMotor.set(ControlMode.PercentOutput, spd);
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  // public boolean isLimit() {
  // return
  // }

  public void setTiltZeroPos() {
    tiltMotor.getSensorCollection().setQuadraturePosition(0, TiltConstants.kTimeoutMs);
  }

  public double getTiltAmps() {
    return pdp.getCurrent(TiltConstants.kTiltPowerIndex);
  }

  public void moveToAngle(double angle, double speed) {
    // angleMotor.set(speed);
    // angleEncoder.setPosition(angle); // TODO set constants to adjust from ticks
    // anglePIDController.setReference(angle, ControlType.kVelocity);
  }

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
