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
  private final TalonSRX yawMotor = new TalonSRX(CANidConstants.kYawMotor);
  private final TalonSRX tiltMotor = new TalonSRX(CANidConstants.kTiltMotor);

  private final DigitalInput yawCenterPos = new DigitalInput(DigitalIOConstants.kCenterDigital);

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
  private NetworkTableEntry sbYawPos = turretTab.addPersistent("Yaw Position", 0).getEntry();
  private NetworkTableEntry sbYawVelocity = turretTab.addPersistent("Yaw Velocity", 0).getEntry();
  private NetworkTableEntry sbYawSetPoint = turretTab.addPersistent("Yaw SetPoint", 0).getEntry();
  public NetworkTableEntry sbLeftPos = turretTab.addPersistent("Yaw Center Left Pos", 0).getEntry();
  public NetworkTableEntry sbRightPos = turretTab.addPersistent("Yaw Center Right Pos", 0).getEntry();
  public NetworkTableEntry sbCenterPos = turretTab.addPersistent("Yaw Center Center Pos", 0).getEntry();

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
    yawMotor.configFactoryDefault();
    yawMotor.clearStickyFaults();
    yawMotor.setNeutralMode(NeutralMode.Brake);
    yawMotor.setInverted(false);
    // yawMotor.setSensorPhase(false);

    /* Config sensor used for Primary PID [Velocity] */
    yawMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CANidConstants.kPIDLoopIdx,
        CANidConstants.kTimeoutMs);

    yawMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    yawMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    /* Config the peak and nominal outputs */
    yawMotor.configNominalOutputForward(0, CANidConstants.kTimeoutMs);
    yawMotor.configNominalOutputReverse(0, CANidConstants.kTimeoutMs);
    yawMotor.configPeakOutputForward(1, CANidConstants.kTimeoutMs);
    yawMotor.configPeakOutputReverse(-1, CANidConstants.kTimeoutMs);

    /* Config the PID values */
    yawMotor.config_kF(CANidConstants.kPIDLoopIdx, YawConstants.kFF, CANidConstants.kTimeoutMs);
    yawMotor.config_kP(CANidConstants.kPIDLoopIdx, YawConstants.kP, CANidConstants.kTimeoutMs);
    yawMotor.config_kI(CANidConstants.kPIDLoopIdx, YawConstants.kI, CANidConstants.kTimeoutMs);
    yawMotor.config_kD(CANidConstants.kPIDLoopIdx, YawConstants.kD, CANidConstants.kTimeoutMs);

    yawMotor.getSensorCollection().setQuadraturePosition(0, CANidConstants.kTimeoutMs);

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

    initTilt();
    initYaw();

    System.out.println("----- Vision Constructor finished ...");
  }

  public void periodic() {
    sbTiltAmps.setDouble(getTiltAmps());
    sbTiltSetPoint.setDouble(getTiltTarget());
    sbTiltPos.setDouble(getTiltPosition());
    sbTiltVelocity.setDouble(getTiltVelocity());

    sbYawSetPoint.setDouble(getYawTarget());
    sbYawPos.setDouble(getYawPosition());
    sbYawVelocity.setDouble(getYawVelocity());

    SmartDashboard.putBoolean("Left Limit Switch", getYawLeftLimit());
    SmartDashboard.putBoolean("Right Limit Switch", getYawRightLimit());
    SmartDashboard.putBoolean("Center Limit Switch", getYawCenterPos());

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

  public void stopYaw() {
    yawMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopTilt() {
    tiltMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void initTilt() {
    int state = 0;
    switch (state) {
      case 0: // initialize
        // nothing
        state++;
        break;
      case 1: // execute
        // command tilt motor down
        moveTiltDown(TiltConstants.kATiltFindSpeed);
        break;
      case 2: // finished & end
        // stop tilt motor when amp draw indicates jammed
        if (Math.abs(getTiltAmps()) > TiltConstants.kTiltAmps) {
          stopTilt();
        }
        break;
        
      default:
    }
  }

  public void initYaw() {
    int state = 0;
    boolean movingLeft = true;
    boolean thisFound = false;
    boolean lastFound = false;
    double leftPos = 0;
    double rightPos = 0;

    switch (state) {
      case 0: // initialize
        moveYawLeft(YawConstants.kYawFindSpeed);
        movingLeft = true;
        state++;
        break;

      case 1: // execute / finished
        if (!getYawCenterPos()) {
          if (movingLeft && getYawLeftLimit()) {
            movingLeft = false;
            moveYawRight(YawConstants.kYawFindSpeed);
          } else if (!movingLeft && getYawRightLimit()) {
            movingLeft = true;
            moveYawLeft(YawConstants.kYawFindSpeed);
          }
        }

        if (getYawCenterPos()) {
          state++;
        }
        break;

      case 2: // initialize
        moveYawLeft(YawConstants.kYawCenterSpeed);
        movingLeft = true;
        thisFound = getYawCenterPos();
        lastFound = getYawCenterPos();
        leftPos = 0;
        rightPos = 0;
        state++;
        break;

      case 3: // execute
        thisFound = getYawCenterPos();

        // if transition from not seeing Center to seeing Center, capture angle
        if (thisFound && (thisFound != lastFound)) {
          if (movingLeft) {
            leftPos = getYawPosition();
          } else {
            rightPos = getYawPosition();
          }
        }

        // if transition off Center position, switch direction
        if (!thisFound && (thisFound != lastFound)) {
          if (movingLeft) {
            movingLeft = false;
            moveYawRight(YawConstants.kYawCenterSpeed);
          } else if (!movingLeft) {
            movingLeft = true;
            moveYawLeft(YawConstants.kYawCenterSpeed);
          }
        }

        // reset last found
        lastFound = thisFound;

        sbLeftPos.setDouble(leftPos);
        sbRightPos.setDouble(rightPos);

        if (leftPos != 0 && rightPos != 0) {
          state++;
        }
        break;

      case 4: // end
        double centerPos = (leftPos + rightPos) / 2.0;
        sbCenterPos.setDouble(centerPos);
        setYawTarget(centerPos);
        setYawZeroPos();
        break;
 
      default:
    }
  }

  public double getYawVelocity() {
    return yawMotor.getSelectedSensorVelocity() / YawConstants.kTicsPerDegree;
  }

  public double getYawPosition() {
    return yawMotor.getSelectedSensorPosition() / YawConstants.kTicsPerDegree;
  }

  public void setYawTarget(double deg) {
    yawMotor.set(ControlMode.Position, deg * YawConstants.kTicsPerDegree);
  }

  public double getYawTarget() {
    return yawMotor.getClosedLoopTarget() / YawConstants.kTicsPerDegree;
  }

  public void moveYawLeft(double spd) {
    yawMotor.set(ControlMode.PercentOutput, -spd);
  }

  public void moveYawRight(double spd) {
    yawMotor.set(ControlMode.PercentOutput, spd);
  }

  public boolean getYawRightLimit() {
    return yawMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getYawCenterPos() {
    return !yawCenterPos.get();
  }

  public boolean getYawLeftLimit() {
    return yawMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void setYawZeroPos() {
    yawMotor.getSensorCollection().setQuadraturePosition(0, CANidConstants.kTimeoutMs);
    setYawTarget(0.0);
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
    // tilt yaw at initialization is not Zero. Set to kMinDeg
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
    return (isLLTgtValid() && (Math.abs(180.0 - getYawPosition() + chassis.getYaw()) < 10.0));
  }

  public boolean isTiltOnTarget() {
    return (Math.abs(tiltMotor.getClosedLoopError() / TiltConstants.kTicsPerDegree)) < TiltConstants.kOnTgtDegree;
  }

  public boolean isYawOnTarget() {
    return (Math.abs(yawMotor.getClosedLoopError() / YawConstants.kTicsPerDegree)) < YawConstants.kOnTgtDegree;
  }

  public boolean isTracking() {
    return isTgtValid() && isTiltOnTarget() && isYawOnTarget();
  }
}
