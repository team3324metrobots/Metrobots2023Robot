// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.arm.commands;

// import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.arm.Arm;

public class TelescopeArm extends CommandBase {
  Arm arm;
  double speed;

  /** Creates a new TelescopeDebug. */
  public TelescopeArm(Arm arm, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    this.arm = arm;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setTeleSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setTeleSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
