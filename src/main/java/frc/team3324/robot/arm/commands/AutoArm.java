// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.arm.Arm;

public class AutoArm extends CommandBase {
  Arm arm;
  double position;

  /** Creates a new AutoArm. */
  public AutoArm(Arm arm, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = arm.getFeedForwardSpeed();
    arm.setArmSpeed(speed);
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
