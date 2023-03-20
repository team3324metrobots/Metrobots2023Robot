// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.intake.Intake;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeCone extends CommandBase {
  Intake intake;
  double speed;
  
  @Log
  double conePosition;

  /** Creates a new RunIntake. */
  public IntakeCone(Intake intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setConeIntakeSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setConeIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
