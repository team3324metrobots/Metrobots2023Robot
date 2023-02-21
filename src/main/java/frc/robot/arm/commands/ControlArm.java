// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;
import frc.robot.util.Constants;

public class ControlArm extends CommandBase {
  /** Creates a new ControlArm. */
  Arm arm;
  DoubleSupplier armSpeedSupplier;

  public ControlArm(Arm arm, DoubleSupplier armSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    this.arm = arm;
    this.armSpeedSupplier = armSpeedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armSpeed = armSpeedSupplier.getAsDouble();

    if (armSpeed < Constants.Arm.ARM_CONTROLLER_DEADZONE && armSpeed > -Constants.Arm.ARM_CONTROLLER_DEADZONE) {
      armSpeed = 0;
    }

    arm.setArmSpeed(armSpeed * 0.5);
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
