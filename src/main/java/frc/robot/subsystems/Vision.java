/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Add your docs here.
 */
public class Vision extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tv = table.getEntry("tv");

  // read values periodically
  private double x = 0.0;  // +-29.8.0 degrees from crosshair to target
  private double y = 0.0;  // +-24.85 degrees from crosshair to target
  private double area = 0.0; // 0% to 100% of image
  private double skew = 0.0; // Rotation, -90 deg to 0 deg
  private double valid = 0.0;  // 0-1 has valid targets

  private double[] cmd = { 0.0, 0.0, 0.0 };

  // post to smart dashboard periodically
  // SmartDashboard.putNumber("LimelightX", x);
  // SmartDashboard.putNumber("LimelightY", y);
  // SmartDashboard.putNumber("LimelightArea", area);
  // SmartDashboard.putNumber("LimelightSkew", skew);
  // SmartDashboard.putNumber("LimelightValid", valid);

  private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
  private NetworkTableEntry sbVisionValid = visionTab.addPersistent("LL Valid", 0).getEntry();
  private NetworkTableEntry sbVisionX = visionTab.addPersistent("LL X", 0).getEntry();
  private NetworkTableEntry sbVisionY = visionTab.addPersistent("LL Y", 0).getEntry();
  private NetworkTableEntry sbVisionArea = visionTab.addPersistent("LL Area", 0).getEntry();
  private NetworkTableEntry sbVisionSkew = visionTab.addPersistent("LL Skew", 0).getEntry();
  private NetworkTableEntry sbVisionYaw = visionTab.addPersistent("Vision Yaw", 0).getEntry();
  private NetworkTableEntry sbVisionTilt = visionTab.addPersistent("Vision Tilt", 0).getEntry();
  private NetworkTableEntry sbVisionDist = visionTab.addPersistent("Vision Dist", 0).getEntry();

  private Library lib = new Library();
  
  public Vision() {
    System.out.println("+++++ Vision Constructor starting ...");
    System.out.println("----- Vision Constructor finished ...");
  }

  public void periodic() {
    x = tx.getDouble(0.0); // +-29.8.0 degrees from crosshair to target
    y = ty.getDouble(0.0); // +-24.85 degrees from crosshair to target
    area = ta.getDouble(0.0); // 0% to 100% of image
    skew = ts.getDouble(0.0); // Rotation, -90 deg to 0 deg
    valid = tv.getDouble(0.0); // 0-1 has valid targets
    
    if (getTgtValid()) {
      cmd = lib.calcTgtCmd(x, y, skew);
    }

    sbVisionValid.setBoolean(getTgtValid());
    sbVisionX.setDouble(x);
    sbVisionY.setDouble(y);
    sbVisionArea.setDouble(area);

    sbVisionSkew.setDouble(lib.calcSkewAngle(skew));

    sbVisionYaw.setDouble(cmd[0]);
    sbVisionTilt.setDouble(cmd[1]);
    sbVisionDist.setDouble(cmd[2]);
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

  public boolean getTgtValid() {
    return (valid == 0 ? false : true);
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
}
