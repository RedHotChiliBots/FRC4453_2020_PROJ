/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Library;
import frc.robot.Constants.SpinnerConstants;
import frc.robot.Constants.SpinnerConstants.COLOR;

/**
 * Add your docs here.
 */
public class Spinner extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final CANSparkMax spinMotor = new CANSparkMax(SpinnerConstants.kSpinnerMotor, MotorType.kBrushless);
  private CANPIDController m_spinPIDController;
  private CANEncoder m_spinEncoder;

  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
   * The device will be automatically initialized with default parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Array of counter variables for each color defineded in COLOR
   */
  private Map<COLOR, Integer> colorCounter = new HashMap<>();
  private Map<Character, COLOR> stopOnColor = new HashMap<>();
  private double setPoint = 0.0;
  private COLOR oldColor = COLOR.UNKNOWN;

  private Color detectedColor;
  private ColorMatchResult match;
  private COLOR colorString = COLOR.UNKNOWN;

  public Spinner() {
    System.out.println("+++++ Spinner Constructor starting ...");

    // Configure Motor
    spinMotor.restoreFactoryDefaults();
    spinMotor.clearFaults();

    spinMotor.setIdleMode(IdleMode.kBrake);
    spinMotor.setInverted(false);

    m_spinPIDController = spinMotor.getPIDController();
    m_spinEncoder = spinMotor.getEncoder();

    // Configure PID Controller
    m_spinPIDController.setP(SpinnerConstants.kP);
    m_spinPIDController.setI(SpinnerConstants.kI);
    m_spinPIDController.setD(SpinnerConstants.kD);
    m_spinPIDController.setIZone(SpinnerConstants.kIz);
    m_spinPIDController.setFF(SpinnerConstants.kFF);
    m_spinPIDController.setOutputRange(SpinnerConstants.kMinRPM, SpinnerConstants.kMaxRPM);

    // Configure Encoder
    m_spinEncoder.setVelocityConversionFactor(SpinnerConstants.kVelFactor);

    // Initialize color detection
    m_colorMatcher.addColorMatch(SpinnerConstants.kGreenTarget);
    m_colorMatcher.addColorMatch(SpinnerConstants.kBlueTarget);
    m_colorMatcher.addColorMatch(SpinnerConstants.kYellowTarget);
    m_colorMatcher.addColorMatch(SpinnerConstants.kRedTarget);

    // Initialize Game Color to Stop On Color
    stopOnColor.put('G', COLOR.YELLOW);
    stopOnColor.put('B', COLOR.RED);
    stopOnColor.put('Y', COLOR.GREEN);
    stopOnColor.put('R', COLOR.BLUE);

    // Initialize local setup
    initColorCounter();
    setSetPoint(SpinnerConstants.kStopRPMs);

    System.out.println("----- Spinner Constructor finished ...");
  }

  // Called once per Robot execution loop - 50Hz
  public void periodic() {
    colorString = getColor();

    SmartDashboard.putNumber("Spin Detected Red", detectedColor.red);
    SmartDashboard.putNumber("Spin Detected Green", detectedColor.green);
    SmartDashboard.putNumber("Spin Detected Blue", detectedColor.blue);

    SmartDashboard.putNumber("Spin Confidence", match.confidence);
    SmartDashboard.putString("Spin Color Detected", colorString.toString());

    String cntString = "";
    for (COLOR color : COLOR.values()) {
      cntString += color.toString().charAt(0) + ":" + colorCounter.get(color).toString() + " ";
    }
    cntString += sumColor();
    SmartDashboard.putString("Spin Color Counters", cntString);

    SmartDashboard.putNumber("Spin SetPoint", setPoint);
    SmartDashboard.putNumber("Spin RPMs", m_spinEncoder.getVelocity());
  }

  /**
   * Set the target RPMs
   * 
   * @param rpm - Target RPMs
   */
  public void setSetPoint(double rpm) {
    this.setPoint = Library.Clip(rpm, SpinnerConstants.kMaxRPM, SpinnerConstants.kMinRPM);
    m_spinPIDController.setReference(this.setPoint, ControlType.kVelocity);
  }

  /**
   * Convert Game Color from FMS to Control Panel Color to detect and stop. The
   * color sensor is two colors away from where the Game Panel needs to stop.
   * 
   * @param gameColor - from FMS
   * @return color to detect
   */
  public COLOR getStopOnColor(char gameColor) {
    COLOR color = stopOnColor.get(gameColor);
    String strColor = gameColor + ":" + color.toString();
    SmartDashboard.putString("Spin Stop On Color", strColor);
    return color;
  }

  /**
   * Read the color from the color sensor. If the color sensed is not one of four
   * known colors, then return UNKNOWN.
   * 
   * @return color sensed by sensor
   */
  public COLOR getColor() {
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);
    colorString = COLOR.UNKNOWN;

    if (match.color == SpinnerConstants.kGreenTarget) {
      colorString = COLOR.GREEN;
    } else if (match.color == SpinnerConstants.kBlueTarget) {
      colorString = COLOR.BLUE;
    } else if (match.color == SpinnerConstants.kYellowTarget) {
      colorString = COLOR.YELLOW;
    } else if (match.color == SpinnerConstants.kRedTarget) {
      colorString = COLOR.RED;
    }

    return colorString;
  }

  /**
   * Count each color the first time it is sensed as the control panel rotates.
   * 
   * @param init - true initializes counters first
   * @return Nothing.
   */
  public void countColor(final boolean init) {
    if (init)
      initColorCounter();

    COLOR color = getColor();
    if (color != oldColor) {
      oldColor = color;
      int i = colorCounter.get(color);
      colorCounter.put(color, ++i);
    }
  }

  /**
   * Initialize the color counters to zero.
   * 
   * @return Nothing.
   */
  public void initColorCounter() {
    for (final COLOR color : COLOR.values()) {
      colorCounter.put(color, 0);
    }
  }

  /**
   * Add all the individual color counters to count the number of revolutions made
   * by the control panel. There are eight color panels on the control panel hence
   * eight counts per revolution or 24 counts per three revolutions.
   * 
   * @return int This returns sum of all the color counters.
   */
  public int sumColor() {
    int sum = 0;
    int num = 0;
    for (final COLOR color : COLOR.values()) {
      if (color != COLOR.UNKNOWN) {
        num = colorCounter.get(color);
        SmartDashboard.putNumber(color.toString(), num);
        sum += num;
      }
    }
    return sum;
  }

  // public void spin(double setPoint) {
  // // shootMotor.set(ShooterConstants.shootSpeed);
  // // double setPoint = m_stick.getY() * maxRPM;
  // m_spinPIDController.setReference(setPoint, ControlType.kVelocity);
  // }

  public void stopSpin() {
    spinMotor.set(0);
  }

  // @Override
  // public void initDefaultCommand() {
  // Set the default command for a subsystem here.
  // setDefaultCommand(new MySpecialCommand());
  // }
}
