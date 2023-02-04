// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.arm.Arm;

public class ControlArm extends CommandBase {
  /** Creates a new ControlArm. */
  double position;

  public ControlArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  public ControlArm(Arm arm, double position) {
    // second constructor for auto use
    addRequirements(arm);

    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.arm.setArmSpeed(RobotContainer.primaryDriver.getRightY() * 0.5);
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
