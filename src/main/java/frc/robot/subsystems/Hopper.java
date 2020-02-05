/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DigitalIOConstants;
import frc.robot.Constants.EjectorConstants;
import frc.robot.Constants.HopperConstants;

/**
 * Add your docs here.
 */
public class Hopper extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Define the Hopper motors, master and follower
  private final CANSparkMax topMaster = new CANSparkMax(CANidConstants.kTopMasterMotor, MotorType.kBrushless);
  private final SpeedController m_topMaster = topMaster;
  private final CANSparkMax bottomFollower = new CANSparkMax(CANidConstants.kBottomFollowerMotor, MotorType.kBrushless);
  private final SpeedController m_bottomFollower = bottomFollower;

  // Group the top and bottom motors
  private final SpeedControllerGroup m_hopperGroup = new SpeedControllerGroup(m_topMaster, m_bottomFollower);

  // Identify top encoder
  public final CANEncoder m_topEncoder = new CANEncoder(topMaster);

  // Identify top PID controller
  private final CANPIDController m_topPIDController = new CANPIDController(topMaster);

  // Define the Ejector motors, master and follower
  private final TalonSRX leftEjector = new TalonSRX(CANidConstants.kLeftEjectorMotor);
  private final TalonSRX rightEjector = new TalonSRX(CANidConstants.kRightEjectorMotor);

  private final DigitalInput upperSensor = new DigitalInput(DigitalIOConstants.kUpperSensor);
  private final DigitalInput lowerSensor = new DigitalInput(DigitalIOConstants.kLowerSensor);

  private double hopperSetPoint;
  private double ejectorSetPoint;

  public Hopper() {
    System.out.println("+++++ Spinner Constructor starting ...");

    // Define Shooter motor
    topMaster.restoreFactoryDefaults();
    topMaster.clearFaults();

    topMaster.setIdleMode(IdleMode.kBrake);

    bottomFollower.restoreFactoryDefaults();
    bottomFollower.clearFaults();

    bottomFollower.setIdleMode(IdleMode.kBrake);

    m_hopperGroup.setInverted(false);

    m_topPIDController.setP(HopperConstants.kP);
    m_topPIDController.setI(HopperConstants.kI);
    m_topPIDController.setD(HopperConstants.kD);
    m_topPIDController.setIZone(HopperConstants.kIz);
    m_topPIDController.setFF(HopperConstants.kFF);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_topEncoder.setPositionConversionFactor(HopperConstants.kVelFactor);

    // Define Angle motor
    leftEjector.configFactoryDefault();
    leftEjector.clearStickyFaults();

    // Configure Motor
    leftEjector.setNeutralMode(NeutralMode.Brake);
    leftEjector.setInverted(false);
    leftEjector.setSensorPhase(false);

    /* Config sensor used for Primary PID [Velocity] */
    leftEjector.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CANidConstants.kPIDLoopIdx,
        CANidConstants.kTimeoutMs);

    // Define Angle motor
    rightEjector.configFactoryDefault();
    rightEjector.clearStickyFaults();

    // Configure Motor
    rightEjector.setNeutralMode(NeutralMode.Brake);
    rightEjector.setInverted(false);
    rightEjector.setSensorPhase(false);

    rightEjector.follow(leftEjector);

    /* Config the Velocity closed loop gains in slot0 */
    leftEjector.config_kF(CANidConstants.kPIDLoopIdx, EjectorConstants.kFF, CANidConstants.kTimeoutMs);
    leftEjector.config_kP(CANidConstants.kPIDLoopIdx, EjectorConstants.kP, CANidConstants.kTimeoutMs);
    leftEjector.config_kI(CANidConstants.kPIDLoopIdx, EjectorConstants.kI, CANidConstants.kTimeoutMs);
    leftEjector.config_kD(CANidConstants.kPIDLoopIdx, EjectorConstants.kD, CANidConstants.kTimeoutMs);

    stopHopper();
    stopEjector();

    System.out.println("----- Spinner Constructor finished ...");
  }

  // Called once per Robot execution loop - 50Hz
  public void periodic() {
    SmartDashboard.putNumber("Hopper SetPoint (rpm)", hopperSetPoint);
    SmartDashboard.putNumber("Hopper Target (rpm)", getHopperVelocity());
    SmartDashboard.putNumber("Ejector SetPoint (rpm)", ejectorSetPoint);
    SmartDashboard.putNumber("Ejector Target (rpm)", getEjectorVelocity());
  }

  /**
   * Get current speed (rpms) of the Spinner motor
   * 
   * @return rpm - scaled speed to rpms
   */
  public double getHopperVelocity() {
    return m_topEncoder.getVelocity();
  }

  /**
   * Set speed (rpms) of Spinner motor/gearbox.
   * 
   * @param rpm - desired speed (rpms) of motor/gearbox
   */
  public void setHopperVelocity(double rpm) {
    hopperSetPoint = rpm;
    m_topPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void stopHopper() {
    m_hopperGroup.set(0.0);
  }

  /**
   * Get current speed (rpms) of the Spinner motor
   * 
   * @return rpm - scaled speed to rpms
   */
  public double getEjectorVelocity() {
    return leftEjector.getSelectedSensorVelocity(CANidConstants.kPIDLoopIdx) / EjectorConstants.kVelFactor;
  }

  /**
   * Set speed (rpms) of Spinner motor/gearbox.
   * 
   * @param rpm - desired speed (rpms) of motor/gearbox
   */
  public void setEjectorVelocity(double rpm) {
    ejectorSetPoint = rpm;
    leftEjector.set(ControlMode.Velocity, rpm * EjectorConstants.kVelFactor);
  }

  public void stopEjector() {
    leftEjector.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean getUpperSensor() {
    return upperSensor.get();
  }

  public boolean getLowerSensor() {
    return lowerSensor.get();
  }

  public void moveBallsUp() {
    if (getLowerSensor() == true) {
      setHopperVelocity(Constants.HopperConstants.kLoadRPMs);
    }
    if (getUpperSensor() == true) {
      stopHopper();
    } else {
      stopHopper();
    }
  }
}
