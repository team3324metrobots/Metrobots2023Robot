// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.vision.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.drivetrain.Drivetrain;
import frc.team3324.robot.vision.Vision;

public class AlignWithVision extends CommandBase {
  
  Vision vision;
  Drivetrain drivetrain;

  /** Creates a new AlignWithVision. */
  public AlignWithVision(Vision vision, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, drivetrain);
    this.drivetrain = drivetrain;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Target ID", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0));

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv == 1){

    float Kp = -0.1f;  // Proportional control constant
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);

    double steering_adjust = Kp * tx;

    double target = drivetrain.getGyroHeading() - steering_adjust;
    double speed = drivetrain.getPIDSpeed(target);

    drivetrain.curvatureDrive(0, -speed);

    } else {

      drivetrain.setMaxOutput(1);
      drivetrain.curvatureDrive(0, 0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.setOutputVolts(0, 0);
    drivetrain.setMaxOutput(1);
    drivetrain.curvatureDrive(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
