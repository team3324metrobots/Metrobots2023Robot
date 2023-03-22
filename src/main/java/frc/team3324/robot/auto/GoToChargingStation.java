// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3324.robot.drivetrain.commands.DriveStraight;
import frc.team3324.robot.arm.Arm;
import frc.team3324.robot.drivetrain.Drivetrain;
import frc.team3324.robot.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToChargingStation extends SequentialCommandGroup {
  PathPlannerTrajectory goToStation = PathPlanner.loadPath("name", new PathConstraints(2, 2));
  /** Creates a new GoToChargingStation. */
  public GoToChargingStation(Drivetrain drivetrain, Arm arm, Intake intake, Trajectory trajectory) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand())

    addCommands(
      
      drivetrain.FollowPath(goToStation, drivetrain)
      // new AutoBalance(drivetrain)
    );
  }
}