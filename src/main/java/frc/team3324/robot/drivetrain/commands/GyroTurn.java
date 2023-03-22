// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.drivetrain.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.drivetrain.Drivetrain;

public class GyroTurn extends CommandBase {
  Drivetrain drivetrain; 
  private double goal;
  private double angle;

  /** Creates a new GyroTurn. */
  public GyroTurn(Drivetrain drivetrain, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    
    this.drivetrain = drivetrain;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setPIDTolerance(1.0);
    angle = Preferences.getDouble("GyroTurn Angle Target", 90.0);
    SmartDashboard.putNumber("GyroTurn Start", drivetrain.getGyroAngle360());
    goal = drivetrain.getGyroAngle() + angle;
    SmartDashboard.putNumber("GyroTurn Target", goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = drivetrain.getPIDSpeed(goal);
    SmartDashboard.putNumber("PID Speed", speed);
    drivetrain.curvatureDrive(0.0, -speed);
    SmartDashboard.putNumber("GyroTurn End", drivetrain.getGyroAngle360());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
