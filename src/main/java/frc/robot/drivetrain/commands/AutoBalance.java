// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.Drivetrain;

public class AutoBalance extends CommandBase {
  Drivetrain drivetrain;

  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.PIDControl.setTolerance(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if  (drivetrain.getGyroPitch() <= 1) {
      drivetrain.curvatureDrive(0.5, 0);
    }
    if (drivetrain.getGyroPitch() < 0) {
      drivetrain.curvatureDrive(-0.2, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
