// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.drivetrain.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.drivetrain.Drivetrain;

public class TrajectoryDrive extends CommandBase {
  Drivetrain drivetrain;
  PathPlannerTrajectory trajectory;
  private Command pathCommand;
 



  /** Creates a new TrajectoryDrive. */
  public TrajectoryDrive(Drivetrain drivetrain, PathPlannerTrajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pathCommand = drivetrain.followPath(trajectory);
    pathCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }
}
