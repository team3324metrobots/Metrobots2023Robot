// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public static void DisplayVisionValues() {

    ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
    CameraServer.startAutomaticCapture();
    tab.add("Limelight Camera", CameraServer.getVideo());

    SmartDashboard.putNumber("Targets", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
    SmartDashboard.putNumber("Horizontal Offset", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    SmartDashboard.putNumber("Vertical Offset", NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));

    SmartDashboard.putNumber("Latency", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0) + NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
