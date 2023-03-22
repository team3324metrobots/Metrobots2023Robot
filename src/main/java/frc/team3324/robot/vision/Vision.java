// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  public static NetworkTable limelight; 
  public static LoggedDashboardNumber visionLatency;
 
  public Vision(){
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    visionLatency = new LoggedDashboardNumber("vision Latency");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    visionLatency.set(limelight.getEntry("tl").getDouble(0));
    
  }
}
