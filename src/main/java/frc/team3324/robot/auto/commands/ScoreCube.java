// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3324.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3324.robot.arm.commands.AutoArm;
import frc.team3324.robot.arm.commands.AutoTelescope;
import frc.team3324.robot.intake.commands.IntakeCube;
import frc.team3324.robot.arm.Arm;
import frc.team3324.robot.arm.Arm.ArmPreset;
import frc.team3324.robot.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCube extends SequentialCommandGroup {
  /** Creates a new ScoreCube. */
  public ScoreCube(Arm arm, Intake intake, ArmPreset position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoArm(arm, 2.0).alongWith(new AutoTelescope(arm, 2.0)),
      new IntakeCube(intake, 1.0) 
    );
  }
}
