/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DigitalIOConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.Constants.YawConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Turret extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final TalonSRX angleMotor = new TalonSRX(CANidConstants.kAngleMotor);
  private final TalonSRX tiltMotor = new TalonSRX(CANidConstants.kTiltMotor);

  private final DigitalInput angleCenterPos = new DigitalInput(DigitalIOConstants.kCenterDigital);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tv = table.getEntry("tv");

  // read values periodically
  private double x = 0.0; // +-29.8.0 degrees from crosshair to target
  private double y = 0.0; // +-24.85 degrees from crosshair to target
  private double area = 0.0; // 0% to 100% of image
  private double skew = 0.0; // Rotation, -90 deg to 0 deg
  private double valid = 0.0; // 0-1 has valid targets

  private double[] cmd = { 0.0, 0.0, 0.0 };

  private final ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");
  private NetworkTableEntry sbTiltAmps = turretTab.addPersistent("Tilt Amps", 0).getEntry();
  private NetworkTableEntry sbTiltPos = turretTab.addPersistent("Tilt Position", 0).getEntry();
  private NetworkTableEntry sbTiltVelocity = turretTab.addPersistent("Tilt Velocity", 0).getEntry();
  private NetworkTableEntry sbTiltSetPoint = turretTab.addPersistent("Tilt SetPoint", 0).getEntry();
  private NetworkTableEntry sbAnglePos = turretTab.addPersistent("Angle Position", 0).getEntry();
  private NetworkTableEntry sbAngleVelocity = turretTab.addPersistent("Angle Velocity", 0).getEntry();
  private NetworkTableEntry sbAngleSetPoint = turretTab.addPersistent("Angle SetPoint", 0).getEntry();
  public NetworkTableEntry sbLeftPos = turretTab.addPersistent("Angle Center Left Pos", 0).getEntry();
  public NetworkTableEntry sbRightPos = turretTab.addPersistent("Angle Center Right Pos", 0).getEntry();
  public NetworkTableEntry sbCenterPos = turretTab.addPersistent("Angle Center Center Pos", 0).getEntry();

  private NetworkTableEntry sbLLValid = turretTab.addPersistent("LL Valid", 0).getEntry();
  private NetworkTableEntry sbLLX = turretTab.addPersistent("LL X", 0).getEntry();
  private NetworkTableEntry sbLLY = turretTab.addPersistent("LL Y", 0).getEntry();
  private NetworkTableEntry sbLLArea = turretTab.addPersistent("LL Area", 0).getEntry();
  private NetworkTableEntry sbLLSkew = turretTab.addPersistent("LL Skew", 0).getEntry();
  private NetworkTableEntry sbTurretYaw = turretTab.addPersistent("Vision Yaw", 0).getEntry();
  private NetworkTableEntry sbTurretTilt = turretTab.addPersistent("Vision Tilt", 0).getEntry();
  private NetworkTableEntry sbTurretDist = turretTab.addPersistent("Vision Dist", 0).getEntry();

  private NetworkTableEntry sbTgtValid = turretTab.addPersistent("Target Valid", 0).getEntry();
  private NetworkTableEntry sbTiltOnTgt = turretTab.addPersistent("Tilt On Target", 0).getEntry();
  private NetworkTableEntry sbYawOnTgt = turretTab.addPersistent("Yaw On Target", 0).getEntry();
  private NetworkTableEntry sbTracking = turretTab.addPersistent("Tracking", 0).getEntry();

  private Library lib = new Library();
  private Chassis chassis = null;

  public Turret(Chassis chassis) {
    System.out.println("+++++ Vision Constructor starting ...");
    this.chassis = chassis;

    // Configure Motor
    angleMotor.configFactoryDefault();
    angleMotor.clearStickyFaults();
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.setInverted(false);
    // angleMotor.setSensorPhase(false);

    /* Config sensor used for Primary PID [Velocity] */
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CANidConstants.kPIDLoopIdx,
        CANidConstants.kTimeoutMs);

    angleMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    angleMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    /* Config the peak and nominal outputs */
    angleMotor.configNominalOutputForward(0, CANidConstants.kTimeoutMs);
    angleMotor.configNominalOutputReverse(0, CANidConstants.kTimeoutMs);
    angleMotor.configPeakOutputForward(1, CANidConstants.kTimeoutMs);
    angleMotor.configPeakOutputReverse(-1, CANidConstants.kTimeoutMs);

    /* Config the PID values */
    angleMotor.config_kF(CANidConstants.kPIDLoopIdx, YawConstants.kFF, CANidConstants.kTimeoutMs);
    angleMotor.config_kP(CANidConstants.kPIDLoopIdx, YawConstants.kP, CANidConstants.kTimeoutMs);
    angleMotor.config_kI(CANidConstants.kPIDLoopIdx, YawConstants.kI, CANidConstants.kTimeoutMs);
    angleMotor.config_kD(CANidConstants.kPIDLoopIdx, YawConstants.kD, CANidConstants.kTimeoutMs);

    angleMotor.getSensorCollection().setQuadraturePosition(0, CANidConstants.kTimeoutMs);

    // Configure Motor
    tiltMotor.configFactoryDefault();
    tiltMotor.clearStickyFaults();
    tiltMotor.setNeutralMode(NeutralMode.Brake);
    tiltMotor.setInverted(false);
    // tiltMotor.setSensorPhase(false);

    /* Config sensor used for Primary PID [Velocity] */
    tiltMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CANidConstants.kPIDLoopIdx,
        CANidConstants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    tiltMotor.configNominalOutputForward(0, CANidConstants.kTimeoutMs);
    tiltMotor.configNominalOutputReverse(0, CANidConstants.kTimeoutMs);
    tiltMotor.configPeakOutputForward(1, CANidConstants.kTimeoutMs);
    tiltMotor.configPeakOutputReverse(-1, CANidConstants.kTimeoutMs);

    /* Config PID Values */
    tiltMotor.config_kF(CANidConstants.kPIDLoopIdx, TiltConstants.kFF, CANidConstants.kTimeoutMs);
    tiltMotor.config_kP(CANidConstants.kPIDLoopIdx, TiltConstants.kP, CANidConstants.kTimeoutMs);
    tiltMotor.config_kI(CANidConstants.kPIDLoopIdx, TiltConstants.kI, CANidConstants.kTimeoutMs);
    tiltMotor.config_kD(CANidConstants.kPIDLoopIdx, TiltConstants.kD, CANidConstants.kTimeoutMs);

    tiltMotor.getSensorCollection().setQuadraturePosition(0, CANidConstants.kTimeoutMs);

    stopTilt();
    stopAngle();

    System.out.println("----- Vision Constructor finished ...");
  }

  public void periodic() {
    sbTiltAmps.setDouble(getTiltAmps());
    sbTiltSetPoint.setDouble(getTiltTarget());
    sbTiltPos.setDouble(getTiltPosition());
    sbTiltVelocity.setDouble(getTiltVelocity());

    sbAngleSetPoint.setDouble(getAngleTarget());
    sbAnglePos.setDouble(getAnglePosition());
    sbAngleVelocity.setDouble(getAngleVelocity());

    SmartDashboard.putBoolean("Left Limit Switch", getAngleLeftLimit());
    SmartDashboard.putBoolean("Right Limit Switch", getAngleRightLimit());
    SmartDashboard.putBoolean("Center Limit Switch", getAngleCenterPos());

    x = tx.getDouble(0.0); // +-29.8.0 degrees from crosshair to target
    y = ty.getDouble(0.0); // +-24.85 degrees from crosshair to target
    area = ta.getDouble(0.0); // 0% to 100% of image
    skew = ts.getDouble(0.0); // Rotation, -90 deg to 0 deg
    valid = tv.getDouble(0.0); // 0-1 has valid targets

    if (isTgtValid()) {
      cmd = lib.calcTgtCmd(x, y, skew);
    }

    sbLLValid.setBoolean(isTgtValid());
    sbLLX.setDouble(x);
    sbLLY.setDouble(y);
    sbLLArea.setDouble(area);
    sbLLSkew.setDouble(lib.calcSkewAngle(skew));

    sbTurretYaw.setDouble(cmd[0]);
    sbTurretTilt.setDouble(cmd[1]);
    sbTurretDist.setDouble(cmd[2]);

    sbTgtValid.setBoolean(isTgtValid());
    sbTiltOnTgt.setBoolean(isTiltOnTarget());
    sbYawOnTgt.setBoolean(isYawOnTarget());
    sbTracking.setBoolean(isTracking());
  }

  public void stopAngle() {
    angleMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopTilt() {
    tiltMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public double getAngleVelocity() {
    return angleMotor.getSelectedSensorVelocity() / YawConstants.kTicsPerDegree;
  }

  public double getAnglePosition() {
    return angleMotor.getSelectedSensorPosition() / YawConstants.kTicsPerDegree;
  }

  public void setAngleTarget(double deg) {
    angleMotor.set(ControlMode.Position, deg * YawConstants.kTicsPerDegree);
  }

  public double getAngleTarget() {
    return angleMotor.getClosedLoopTarget() / YawConstants.kTicsPerDegree;
  }

  public void moveAngleLeft(double spd) {
    angleMotor.set(ControlMode.PercentOutput, -spd);
  }

  public void moveAngleRight(double spd) {
    angleMotor.set(ControlMode.PercentOutput, spd);
  }

  public boolean getAngleRightLimit() {
    return angleMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getAngleCenterPos() {
    return !angleCenterPos.get();
  }

  public boolean getAngleLeftLimit() {
    return angleMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void setAngleZeroPos() {
    angleMotor.getSensorCollection().setQuadraturePosition(0, CANidConstants.kTimeoutMs);
    setAngleTarget(0.0);
  }

  public double getTiltVelocity() {
    return tiltMotor.getSelectedSensorVelocity() / TiltConstants.kTicsPerDegree;
  }

  public double getTiltPosition() {
    return tiltMotor.getSelectedSensorPosition() / TiltConstants.kTicsPerDegree;
  }

  public void setTiltTarget(double deg) {
    tiltMotor.set(ControlMode.Position, deg * TiltConstants.kTicsPerDegree);
  }

  public double getTiltTarget() {
    return tiltMotor.getClosedLoopTarget() / TiltConstants.kTicsPerDegree;
  }

  public void setTiltZeroPos() {
    // tilt angle at initialization is not Zero. Set to kMinDeg
    int tics = (int) Math.round(TiltConstants.kMinDeg * TiltConstants.kTicsPerDegree);
    tiltMotor.getSensorCollection().setQuadraturePosition(tics, CANidConstants.kTimeoutMs);
    setTiltTarget(TiltConstants.kMinDeg);
  }

  public void moveTiltDown(double spd) {
    tiltMotor.set(ControlMode.PercentOutput, -spd);
  }

  public double getTiltAmps() {
    return tiltMotor.getStatorCurrent();
    // return pdp.getCurrent(TiltConstants.kTiltPowerIndex);
  }

  public double getTgtX() {
    return tx.getDouble(0.0);
  }

  public double getTgtY() {
    return ty.getDouble(0.0);
  }

  public double getTgtArea() {
    return ta.getDouble(0.0);
  }

  public double getTgtSkew() {
    return ts.getDouble(0.0);
  }

  public double getYawCmd() {
    return cmd[0];
  }

  public double getTiltCmd() {
    return cmd[1];
  }

  public double getTgtDist() {
    return cmd[2];
  }

  public boolean isLLTgtValid() {
    return valid == 1;
  }

  public boolean isTgtValid() {
    return (isLLTgtValid() && (Math.abs(180.0 - getAnglePosition() + chassis.getYaw()) < 10.0));
  }

  public boolean isTiltOnTarget() {
    return (Math.abs(tiltMotor.getClosedLoopError() / TiltConstants.kTicsPerDegree)) < TiltConstants.kOnTgtDegree;
  }

  public boolean isYawOnTarget() {
    return (Math.abs(angleMotor.getClosedLoopError() / YawConstants.kTicsPerDegree)) < YawConstants.kOnTgtDegree;
  }

  public boolean isTracking() {
    return isTgtValid() && isTiltOnTarget() && isYawOnTarget();
  }
}
