// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.Drivetrain;

public class AutoBalance extends CommandBase {
  Drivetrain drivetrain;
  double speed;
  final double setpoint = 0.0;

  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setPIDPitchTolerance(0.5);;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setPitchP(Preferences.getDouble("AutoBal P", 0.0));
    drivetrain.setPitchI(Preferences.getDouble("AutoBal I", 0.0));
    drivetrain.setPitchD(Preferences.getDouble("AutoBal D", 0.0));
    SmartDashboard.putNumber("PID Speed", speed);
    speed = drivetrain.getPIDPitchSpeed(setpoint)* 0.25;
    drivetrain.curvatureDrive(-speed, 0); // i have zero clue why speed needs to be negative but it works
    if (drivetrain.pitchAtSetpoint()) {
      drivetrain.curvatureDrive(0, 0);
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
