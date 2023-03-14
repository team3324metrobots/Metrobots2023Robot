// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3324.robot.drivetrain.commands.AutoBalance;
import frc.team3324.robot.drivetrain.commands.DriveStraight;
import frc.team3324.robot.arm.Arm;
import frc.team3324.robot.drivetrain.Drivetrain;
import frc.team3324.robot.drivetrain.commands.TrajectoryDrive;
import frc.team3324.robot.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToChargingStation extends SequentialCommandGroup {
  /** Creates a new GoToChargingStation. */
  public GoToChargingStation(Drivetrain drivetrain, Arm arm, Intake intake, Trajectory trajectory) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand())

    addCommands(
      new DriveStraight(drivetrain, 1.5).withTimeout(2.1)
      // new AutoBalance(drivetrain)
    );
  }
}