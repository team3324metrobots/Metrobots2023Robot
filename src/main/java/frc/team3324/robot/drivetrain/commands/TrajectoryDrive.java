// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.drivetrain.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3324.robot.drivetrain.Drivetrain;

public class TrajectoryDrive extends CommandBase {
  Drivetrain drivetrain;
  Trajectory trajectory;

  RamseteController ramseteController = new RamseteController();


  /** Creates a new TrajectoryDrive. */
  public TrajectoryDrive(Drivetrain drivetrain, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speed = ramseteController.calculate(drivetrain.getPose(), trajectory.sample(3.4));
    drivetrain.curvatureDrive(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
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
