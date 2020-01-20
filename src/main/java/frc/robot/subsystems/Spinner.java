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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private final CANPIDController m_spinPIDController = new CANPIDController(spinMotor);
  private final CANEncoder m_spinEncoder = spinMotor.getEncoder();

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

  private Color detectedColor = m_colorSensor.getColor();
  private ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
  private COLOR colorString = COLOR.UNKNOWN;

  // private ShuffleboardTab spinnerTab;

  public Spinner() {
    System.out.println("+++++ Spinner Constructor starting ...");
    // spinnerTab = Shuffleboard.getTab("Spinner");

    spinMotor.restoreFactoryDefaults();

    // set PID coefficients
    m_spinPIDController.setP(SpinnerConstants.kP);
    m_spinPIDController.setI(SpinnerConstants.kI);
    m_spinPIDController.setD(SpinnerConstants.kD);
    m_spinPIDController.setIZone(SpinnerConstants.kIz);
    m_spinPIDController.setFF(SpinnerConstants.kFF);
    m_spinPIDController.setOutputRange(-1.0, 1.0);

    m_colorMatcher.addColorMatch(SpinnerConstants.kBlueTarget);
    m_colorMatcher.addColorMatch(SpinnerConstants.kGreenTarget);
    m_colorMatcher.addColorMatch(SpinnerConstants.kRedTarget);
    m_colorMatcher.addColorMatch(SpinnerConstants.kYellowTarget);

    stopOnColor.put('G', COLOR.YELLOW);
    stopOnColor.put('B', COLOR.RED);
    stopOnColor.put('Y', COLOR.GREEN);
    stopOnColor.put('R', COLOR.BLUE);

    initColorCounter();
    setSetPoint(0.0);

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */

    // spinnerTab.add("Red", detectedColor.red);
    // spinnerTab.add("Green", detectedColor.green);
    // spinnerTab.add("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);

    // spinnerTab.add("Color Sensor", m_colorSensor);
    System.out.println("----- Spinner Constructor finished ...");
  }

  public void Periodic() {
    getColor();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */

    // spinnerTab.add("Confidence", match.confidence);
    // spinnerTab.add("Detected Color", colorString.toString());

    // spinnerTab.add("SetPoint", setPoint);
    // spinnerTab.add("Spin RPMs", m_spinEncoder.getVelocity());
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString.toString());

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Spin RPMs", m_spinEncoder.getVelocity());
  }

  public void setSetPoint(double rpm) {
    this.setPoint = Library.Clip(rpm, SpinnerConstants.kMinRPM, SpinnerConstants.kMaxRPM);
    m_spinPIDController.setReference(setPoint, ControlType.kVelocity);
  }

  public COLOR getStopOnColor(char color) {
    return stopOnColor.get(color);
  }

  public COLOR getColor() {
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);
    colorString = COLOR.UNKNOWN;

    if (match.color == SpinnerConstants.kBlueTarget) {
      colorString = COLOR.BLUE;
    } else if (match.color == SpinnerConstants.kRedTarget) {
      colorString = COLOR.RED;
    } else if (match.color == SpinnerConstants.kGreenTarget) {
      colorString = COLOR.GREEN;
    } else if (match.color == SpinnerConstants.kYellowTarget) {
      colorString = COLOR.YELLOW;
    } else {
      colorString = COLOR.UNKNOWN;
    }

    return colorString;
  }

  /**
   * This method counts each color the first time it is seen as the control panel
   * rotates.
   * 
   * @return Nothing.
   */
  public void countColor(final boolean init) {
    if (init)
      initColorCounter();

    COLOR color = getColor();
    if (color != oldColor) {
      oldColor = color;
      int i = colorCounter.get(color);
      colorCounter.put(color, i++);
    }
  }

  /**
   * This method initializes the color counters to zero.
   * 
   * @return Nothing.
   */
  public void initColorCounter() {
    for (final COLOR color : COLOR.values()) {
      colorCounter.put(color, 0);
    }
  }

  /**
   * This method adds all the individual color counters and used to count the
   * number of revolutions made by the control panel. There are eight color panels
   * on the control panel hence counts counts per revolution.
   * 
   * @return int This returns sum of all the color counters.
   */
  public int sumColor() {
    int sum = 0;
    int num = 0;
    for (final COLOR color : COLOR.values()) {
      if (color != COLOR.UNKNOWN) {
        num = colorCounter.get(color);
        // spinnerTab.add(color.toString(), num);
        sum += num;
      }
    }
    return sum;
  }

  // @Override
  // public void initDefaultCommand() {
  // Set the default command for a subsystem here.
  // setDefaultCommand(new MySpecialCommand());
  // }
}
