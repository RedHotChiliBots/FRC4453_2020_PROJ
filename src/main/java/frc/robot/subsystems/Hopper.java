/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HopperConstants;

/**
 * Add your docs here.
 */
public class Hopper extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final TalonSRX hopperMotor = new TalonSRX(HopperConstants.kHopperMotor);
  private double hopperSetPoint;

  public Hopper() {
    System.out.println("+++++ Spinner Constructor starting ...");

    /* Factory Default all hardware to prevent unexpected behaviour */
    hopperMotor.configFactoryDefault();
    hopperMotor.clearStickyFaults();

    /* Config sensor used for Primary PID [Velocity] */
    hopperMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, HopperConstants.kPIDLoopIdx,
        HopperConstants.kTimeoutMs);

    // Conifigure motor controller
    hopperMotor.setSensorPhase(false); // Positive Sensor Reading should match Green (blinking) Leds on Talon
    hopperMotor.setNeutralMode(NeutralMode.Brake); // Brake motor on neutral input
    hopperMotor.setInverted(false); // Run motor in normal rotation with positive input

    /* Config the peak and nominal outputs */
    hopperMotor.configNominalOutputForward(0, HopperConstants.kTimeoutMs);
    hopperMotor.configNominalOutputReverse(0, HopperConstants.kTimeoutMs);
    hopperMotor.configPeakOutputForward(1, HopperConstants.kTimeoutMs);
    hopperMotor.configPeakOutputReverse(-1, HopperConstants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    hopperMotor.config_kF(HopperConstants.kPIDLoopIdx, HopperConstants.kFF, HopperConstants.kTimeoutMs);
    hopperMotor.config_kP(HopperConstants.kPIDLoopIdx, HopperConstants.kP, HopperConstants.kTimeoutMs);
    hopperMotor.config_kI(HopperConstants.kPIDLoopIdx, HopperConstants.kI, HopperConstants.kTimeoutMs);
    hopperMotor.config_kD(HopperConstants.kPIDLoopIdx, HopperConstants.kD, HopperConstants.kTimeoutMs);
    setRPMs(HopperConstants.kStopRPMs);

    System.out.println("----- Spinner Constructor finished ...");
  }

  // Called once per Robot execution loop - 50Hz
  public void periodic() {
    SmartDashboard.putNumber("Spin SetPoint (rpm)", hopperSetPoint);
    SmartDashboard.putNumber("Spin Target (rpm)", getRPMs());
  }

  /**
   * Get current speed (rpms) of the Spinner motor
   * 
   * @return rpm - scaled speed to rpms
   */
  public double getRPMs() {
    return hopperMotor.getSelectedSensorVelocity(HopperConstants.kPIDLoopIdx) / HopperConstants.kVelFactor;
  }

  /**
   * Set speed (rpms) of Spinner motor/gearbox.
   * 
   * @param rpm - desired speed (rpms) of motor/gearbox
   */
  public void setRPMs(double rpm) {
    hopperSetPoint = rpm;
    hopperMotor.set(ControlMode.Velocity, rpm * HopperConstants.kVelFactor);
  }

  public void stopSpin() {
    hopperMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
